/* ESP32-S3 + MAX30102(心率血氧) + ADS1292R(ECG) + SSD1306 OLED 例程
 * 由 STM32F103C8T6 例程移植（ESP-IDF 原生框架）。
 *
 * 硬件连接（ESP32-S3-DevKitC-1）：
 *   MAX30102  VCC->3V3 GND->GND  SCL->GPIO6  SDA->GPIO5  INT->GPIO7
 *   OLED      VCC->3V3 GND->GND  SCL->GPIO9  SDA->GPIO8
 *   ADS1292R  SCLK->GPIO12 MOSI->GPIO11 MISO->GPIO13 CS0->GPIO10
 *             PWDN->GPIO21 START->GPIO47 DRDY->GPIO14
 *             模块 5V -> 外部 5V（该开发板 5V 为 VIN 输入脚），GND -> GND
 *   状态 LED  -> GPIO2
 *   串口监视   板载 UART 口（GPIO43 TX / GPIO44 RX），115200
 *
 * OLED 布局：y=0 心率，y=16 血氧，y=32 ECG 的 BPM，y=40~63 ECG 滚动波形
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "oled.h"
#include "max30102.h"
#include "ads1292.h"

/* 状态 LED：对应原例程的 PC13 */
#define STATUS_LED_PIN       2

/* 排查 ADS1292 接线专用开关：置 1 后重新编译烧录，程序不再采集数据，
 * 而是逐个翻转 ADS1292 的信号脚并打印脚名，便于用万用表在模块侧
 * 确认每根线是否真的通到芯片。排查完改回 0。 */
#define ADS1292_SELFTEST     0

/* OLED 显存与 I2C 总线被两个任务共用，必须加互斥锁 */
static SemaphoreHandle_t s_oled_mutex;
#define OLED_LOCK()     xSemaphoreTake(s_oled_mutex, portMAX_DELAY)
#define OLED_UNLOCK()   xSemaphoreGive(s_oled_mutex)

/* 波形数据（每列的 min/max 与 BPM）由采集任务写、显示任务读，需加锁。
 * 这样刷屏这种耗时的 I2C 操作就被挪出采集任务，250SPS 不再丢点。 */
static SemaphoreHandle_t s_wave_mutex;
#define WAVE_LOCK()     xSemaphoreTake(s_wave_mutex, portMAX_DELAY)
#define WAVE_UNLOCK()   xSemaphoreGive(s_wave_mutex)

static void board_led_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << STATUS_LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(STATUS_LED_PIN, 0);   // 与原例程一致：初始化后点亮
}

/*==================== 发给 PC 上位机的二进制数据流 ====================*/
/* MAX30102 任务把最新 IR 原始值放到这里，由 ECG 任务统一打包发出 */
static volatile int32_t g_ppg_ir = 0;

/* 帧格式（16 字节，小端）：
 *   AA 55 | CH1 int32 | CH2 int32 | PPG_IR int32 | LOFF u8 | SUM u8
 *   SUM = b[2..14] 逐字节相加 & 0xFF
 * 每个 ECG 采样点（ADS1292 的 500SPS）发一帧。 */
static void send_frame(int32_t ch1, int32_t ch2, int32_t ppg, uint8_t loff)
{
    uint8_t b[16];
    b[0] = 0xAA; b[1] = 0x55;
    b[2]  = (uint8_t)(ch1);       b[3]  = (uint8_t)(ch1 >> 8);
    b[4]  = (uint8_t)(ch1 >> 16); b[5]  = (uint8_t)(ch1 >> 24);
    b[6]  = (uint8_t)(ch2);       b[7]  = (uint8_t)(ch2 >> 8);
    b[8]  = (uint8_t)(ch2 >> 16); b[9]  = (uint8_t)(ch2 >> 24);
    b[10] = (uint8_t)(ppg);       b[11] = (uint8_t)(ppg >> 8);
    b[12] = (uint8_t)(ppg >> 16); b[13] = (uint8_t)(ppg >> 24);
    b[14] = loff;
    uint8_t sum = 0;
    for (int i = 2; i < 15; i++) sum = (uint8_t)(sum + b[i]);
    b[15] = sum;
    /* 直写 USB-Serial-JTAG，超时 0 = 不阻塞：发送缓冲满了就丢掉这一帧，
     * 绝不能让采集任务卡在这里（用 fwrite/fflush 会永久卡死）。 */
    usb_serial_jtag_write_bytes(b, sizeof(b), 0);
}

/*=============================== ECG 任务 ===============================*/

#define ECG_W           128
#define ECG_Y0          40
#define ECG_Y1          63
#define ECG_MID         ((ECG_Y0 + ECG_Y1) / 2)
#define ECG_HALF        11          /* 波形纵向半幅（像素） */
#define ECG_EMPTY       0xFF        /* 该列没有数据 */

#define ECG_DECIM       16          /* 500SPS / 16 = 31.25 列/秒，128 列约 4 秒 */

/* 纵向缩放包络的下限。增益 6、内部 2.42V 基准时 1 个 LSB ≈ 48nV，
 * 30000 计数 ≈ 1.4mV，正好是一个正常 QRS 的幅度量级。
 * 有了这个下限，噪声（通常 <0.1mV）就不会被放大撑满整个波形区。 */
#define ECG_MIN_PEAK    30000

/* 24bit 满量程；超过 8000000 认为输入的 ADC 已经饱和（电极悬空/超量程） */
#define ADS1292_FULL    8000000

static uint8_t s_ecg_min[ECG_W];
static uint8_t s_ecg_max[ECG_W];
static int     s_disp_bpm = 0;      /* 显示任务要画的 BPM */
static volatile bool s_ads_ok = false;  /* ADS1292 初始化时 ID 是否正常 */

static uint8_t ecg_clamp(int32_t y)
{
    if (y < ECG_Y0) y = ECG_Y0;
    if (y > ECG_Y1) y = ECG_Y1;
    return (uint8_t)y;
}

/* 先清掉波形区（保留上方文字），再按每列的 min~max 画竖线 */
static void ecg_redraw(const uint8_t *mn, const uint8_t *mx)
{
    for (uint8_t x = 0; x < ECG_W; x++) {
        for (uint8_t y = ECG_Y0; y <= ECG_Y1; y++) {
            OLED_DrawPoint(x, y, 0);
        }
    }
    for (uint8_t x = 0; x < ECG_W; x++) {
        if (mn[x] == ECG_EMPTY) continue;
        for (uint8_t y = mn[x]; y <= mx[x]; y++) {
            OLED_DrawPoint(x, y, 1);
        }
    }
    OLED_Refresh();
}

/* 整幅图左移一列 */
static void ecg_scroll(void)
{
    memmove(s_ecg_min, s_ecg_min + 1, ECG_W - 1);
    memmove(s_ecg_max, s_ecg_max + 1, ECG_W - 1);
    s_ecg_min[ECG_W - 1] = ECG_EMPTY;
    s_ecg_max[ECG_W - 1] = ECG_EMPTY;
}

static void ecg_task(void *arg)
{
    (void)arg;

    int32_t  ch1 = 0, ch2 = 0;
    int32_t  baseline = 0;
    int32_t  peak = 1;              /* 自适应幅度包络，用于纵向缩放 */
    bool     have_base = false;
    uint32_t decim_cnt = 0;
    uint32_t last_beat_ms = 0;
    uint32_t beat_ms[4] = {0};
    int      beat_idx = 0;
    int      bpm = 0;
    uint8_t  cur_min = ECG_EMPTY, cur_max = ECG_EMPTY;

    memset(s_ecg_min, ECG_EMPTY, sizeof(s_ecg_min));
    memset(s_ecg_max, ECG_EMPTY, sizeof(s_ecg_max));

    while (1) {
        /* 节奏由 ADS1292 的 DRDY 决定（500SPS）：ads1292_read_data 内部
         * 先等 DRDY 下降沿再读整帧，等不到就超时返回 false，不会卡死任务。 */
        bool ok = ads1292_read_data(&ch1, &ch2);
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (ok) {
            /* 导联脱落状态直接从每帧的状态字里取，不再读寄存器：
             * 读寄存器必须先发 SDATAC 退出连续转换，会把数据流打断、
             * 产生错位的帧，正是"ECG 突然巨大"的元凶。 */
            uint8_t loff_stat = ads1292_get_loff();

            /* 心电主通道是 CH2：ProtoCentral 模块上 LA/RA 接 IN2P/IN2N，
             * CH2SET=0x00 增益 6；CH1 是呼吸通道（增益 2，RESP 解调），
             * 只随帧发给上位机，不参与波形与 BPM。
             * （STM32 原版显示的也是 CH2；曾经误写成 ch1，幅度差 3 倍。） */
            int32_t x = ch2;

            if (!have_base) {
                baseline = x;
                have_base = true;
            }
            /* 基线跟踪（一阶低通），去掉直流漂移 */
            baseline += (x - baseline) >> 6;
            int32_t v = x - baseline;

            /* 自适应幅度包络：涨得快、落得慢 */
            int32_t a = (v >= 0) ? v : -v;
            if (a > peak) peak += (a - peak) >> 4;
            else          peak -= (peak - a) >> 6;
            /* 关键：给包络设下限。否则信号很小时（电极没接好、只剩噪声）
             * peak 会被压到接近 0，任意一点噪声都会被除以极小的 peak
             * 而放大到满屏，看起来就像"ECG 特别之大"。 */
            if (peak < ECG_MIN_PEAK) peak = ECG_MIN_PEAK;

            /* R 波检测：超过包络的 5/8 且过了 250ms 不应期算一次心跳 */
            if (a > (peak * 5 / 8) && (now - last_beat_ms) > 250) {
                if (last_beat_ms != 0) {
                    uint32_t rr = now - last_beat_ms;
                    beat_ms[beat_idx & 3] = rr;
                    beat_idx++;
                    if (beat_idx >= 4) {
                        uint32_t sum = beat_ms[0] + beat_ms[1] + beat_ms[2] + beat_ms[3];
                        sum /= 4;
                        if (sum > 0) bpm = (int)(60000UL / sum);
                    }
                }
                last_beat_ms = now;
            }

            /* 每 ECG_DECIM 个样本压成一列 */
            if (++decim_cnt >= ECG_DECIM) {
                decim_cnt = 0;

                int32_t y = ECG_MID - (v * ECG_HALF) / peak;
                uint8_t yy = ecg_clamp(y);
                if (cur_min == ECG_EMPTY) {
                    cur_min = cur_max = yy;
                } else {
                    if (yy < cur_min) cur_min = yy;
                    if (yy > cur_max) cur_max = yy;
                }

                /* 只更新数据，不碰 OLED；刷屏交给 display_task，
                 * 采集任务里绝不做耗时 I2C，否则 250SPS 会丢点。 */
                WAVE_LOCK();
                ecg_scroll();
                s_ecg_min[ECG_W - 1] = cur_min;
                s_ecg_max[ECG_W - 1] = cur_max;
                s_disp_bpm = bpm;
                WAVE_UNLOCK();

                cur_min = cur_max = ECG_EMPTY;
            }

            /* 把这一拍的原始数据打成二进制帧发给 PC 上位机（250Hz）。
             * LOFF 字节最高位借用为"ADC 饱和"标志：ADS1292 输入悬空或
             * 超量程时 24bit 输出会贴到 ±8388607，此时置位让上位机提示。 */
            if (ch1 > ADS1292_FULL || ch1 < -ADS1292_FULL ||
                ch2 > ADS1292_FULL || ch2 < -ADS1292_FULL) {
                loff_stat |= 0x80;
            }
            send_frame(ch1, ch2, g_ppg_ir, loff_stat);
        } else {
            /* ADS1292 没给数据（DRDY 不翻转 / 状态字一直错位）。
             * 仍然发一帧，让上位机分得清"板子在跑、只是心电模块没应答"
             * 和"一个字节都没收到"。bit6 = CH1/CH2 无效。 */
            send_frame(0, 0, g_ppg_ir, 0x40);
        }
    }
}

/*============================= 显示任务 =============================*/
/* 只负责把波形数据搬到 OLED 上，优先级最低：
 * OLED 整屏刷新走 I2C 约 20ms，放在这里阻塞也没关系。 */
static void display_task(void *arg)
{
    (void)arg;

    uint8_t mn[ECG_W];
    uint8_t mx[ECG_W];
    int     bpm_shown = -1;

    while (1) {
        WAVE_LOCK();
        memcpy(mn, s_ecg_min, ECG_W);
        memcpy(mx, s_ecg_max, ECG_W);
        int bpm_now = s_disp_bpm;
        WAVE_UNLOCK();

        OLED_LOCK();
        if (!s_ads_ok) {
            /* 心电模块没应答：波形区直接给出提示，避免看成"这种情况正常" */
            for (uint8_t x = 0; x < ECG_W; x++)
                for (uint8_t y = ECG_Y0; y <= ECG_Y1; y++)
                    OLED_DrawPoint(x, y, 0);
            OLED_ShowString(0, 40, "ADS1292 NO ACK", 8, 1);
            OLED_Refresh();
            OLED_UNLOCK();
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }
        ecg_redraw(mn, mx);
        if (bpm_now != bpm_shown) {
            bpm_shown = bpm_now;
            OLED_ShowNum(30, 32, (uint32_t)bpm_now, 3, 8, 1);
        }
        OLED_UNLOCK();

        vTaskDelay(pdMS_TO_TICKS(80));
    }
}

/*=============================== MAX30102 任务 ===============================*/

/* IR LED 原始数据，用于检测血液 */
static uint32_t aun_ir_buffer[500];
static int32_t  n_ir_buffer_length;
/* Red LED 原始数据，用于检测血氧浓度以及心率 */
static uint32_t aun_red_buffer[500];
static int32_t  n_sp02;             // SPO2 值
static int8_t   ch_spo2_valid;      // SPO2 数据有效指示
static int32_t  n_heart_rate;       // 心率值
static int8_t   ch_hr_valid;        // 心率数据有效指示

static uint8_t  temp[6];
static int      i;
static uint8_t  dis_hr = 0, dis_spo2 = 0;

/* 阻塞等待 MAX30102 INT 拉低（新样本就绪），期间让出 CPU 防看门狗 */
static void wait_sample_ready(void)
{
    while (max30102_get_int_level() == 1) {
        vTaskDelay(1);
    }
}

static void ppg_task(void *arg)
{
    (void)arg;

    n_ir_buffer_length = 500;   // 数据长度 500，可存储 100sps 下 5 秒数据

    /* 读取前 500 个数据，确定信号脉搏波 */
    for (i = 0; i < n_ir_buffer_length; i++) {
        wait_sample_ready();

        max30102_fifo_read_bytes(temp);
        aun_red_buffer[i] = ((long)((long)(temp[0] & 0x03) << 16)) | ((long)temp[1] << 8) | (long)temp[2];
        aun_ir_buffer[i]  = ((long)((long)(temp[3] & 0x03) << 16)) | ((long)temp[4] << 8) | (long)temp[5];
        g_ppg_ir = (int32_t)aun_ir_buffer[i];   /* 供上位机画光电容积波 */
    }

    /* 用前 500 个数据进行前 5 秒第一次心率和血氧饱和度计算 */
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer,
                                           &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);

    while (1) {
        /* 删去前 100 个数据，把剩下 400 个数据平移到数组 0~400 位置 */
        for (i = 100; i < 500; i++) {
            aun_red_buffer[i - 100] = aun_red_buffer[i];
            aun_ir_buffer[i - 100]  = aun_ir_buffer[i];
        }

        /* 在数组末尾读取 100 个新数据，覆盖 400~500 位置 */
        for (i = 400; i < 500; i++) {
            wait_sample_ready();
            max30102_fifo_read_bytes(temp);
            aun_red_buffer[i] = ((long)((long)(temp[0] & 0x03) << 16)) | ((long)temp[1] << 8) | (long)temp[2];
            aun_ir_buffer[i]  = ((long)((long)(temp[3] & 0x03) << 16)) | ((long)temp[4] << 8) | (long)temp[5];
            g_ppg_ir = (int32_t)aun_ir_buffer[i];   /* 供上位机画光电容积波 */

            /* 与原例程一致：仅在心率有效且低于 120 时更新显示值 */
            if (ch_hr_valid == 1 && n_heart_rate < 120) {
                dis_hr = (uint8_t)n_heart_rate;
                dis_spo2 = (uint8_t)n_sp02;
            } else {
                dis_hr = 0;
                dis_spo2 = 0;
            }
        }

        maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer,
                                               &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);

        /* 注意：屏幕心率沿用原例程的经验偏移 -20，串口输出为未修正原值 */
        OLED_LOCK();
        if (dis_hr == 0)
            OLED_ShowNum(47, 0, 0, 3, 16, 1);
        else
            OLED_ShowNum(47, 0, dis_hr - 20, 3, 16, 1);

        OLED_ShowNum(47, 16, dis_spo2, 2, 16, 1);
        OLED_UNLOCK();
    }
}

/*=============================== 主程序 ===============================*/

void app_main(void)
{
    board_led_init();

    /* 直接用 usb_serial_jtag_write_bytes 高速吐数据，必须先安装驱动
     * （控制台默认没装这个驱动），否则会刷 "driver is not initialized yet"。
     * 已安装时会返回错误，忽略即可。 */
    usb_serial_jtag_driver_config_t uj_cfg = {
        .tx_buffer_size = 4096,
        .rx_buffer_size = 256,
    };
    usb_serial_jtag_driver_install(&uj_cfg);

#if ADS1292_SELFTEST
    ads1292_selftest();     /* 接线自检，不返回 */
#endif

    s_oled_mutex = xSemaphoreCreateMutex();
    s_wave_mutex = xSemaphoreCreateMutex();

    OLED_Init();
    vTaskDelay(pdMS_TO_TICKS(50));
    OLED_Clear();

    OLED_LOCK();
    /* 显示“心率:” */
    OLED_ShowChinese(0, 0, 0, 16, 1);
    OLED_ShowChinese(16, 0, 1, 16, 1);
    OLED_ShowChar(40, 0, ':', 16, 1);
    OLED_ShowString(80, 0, "BMP", 16, 1);

    /* 显示“血氧:” */
    OLED_ShowChinese(0, 16, 2, 16, 1);
    OLED_ShowChinese(16, 16, 3, 16, 1);
    OLED_ShowChar(40, 16, ':', 16, 1);
    OLED_ShowChar(80, 16, '%', 16, 1);

    /* ECG 标题与单位 */
    OLED_ShowString(0, 32, "ECG:", 8, 1);
    OLED_ShowString(48, 32, "BPM", 8, 1);
    OLED_UNLOCK();

    ESP_ERROR_CHECK(max30102_init());
    ads1292_init();
    s_ads_ok = (ads1292_get_id() == ADS1292_ID_EXPECTED);

    xTaskCreate(ecg_task,     "ecg_task",     4096, NULL, 6, NULL);  /* 采集，最高 */
    xTaskCreate(ppg_task,     "ppg_task",     4096, NULL, 4, NULL);
    xTaskCreate(display_task, "display_task", 4096, NULL, 3, NULL);  /* 刷屏，最低 */
}