#include "ads1292.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_attr.h"
#include "esp_log.h"

static const char *TAG = "ADS1292";

static spi_device_handle_t s_spi;
static SemaphoreHandle_t   s_drdy_sem;
static uint8_t             s_id;
static uint8_t             s_last_status[3];
static uint8_t             s_loff;          /* 最近一帧状态字里的导联脱落位 */
static uint32_t            s_bad_frames;    /* 被丢弃的错位帧计数 */

/*============================= 底层 SPI 事务 ==============================
 * 全部走单次 spi_device_transmit：CS 由硬件在整个事务期间保持拉低，
 * 事务内部字节背靠背发送。因此 SCLK 必须慢到「一个字节 > tSDECODE」，
 * 见 ads1292.h 里 ADS1292_SPI_FREQ_HZ 的说明。
 *=======================================================================*/

/* 单字节命令：CS 拉低 -> 发命令 -> CS 拉高 */
static void ads1292_send_cmd(uint8_t cmd)
{
    spi_transaction_t t = {0};
    t.length    = 8;
    t.tx_buffer = &cmd;
    ESP_ERROR_CHECK(spi_device_transmit(s_spi, &t));
}

/* 连续写寄存器：WREG|addr, len-1, 数据... */
static void ads1292_write_regs(uint8_t addr, uint8_t len, const uint8_t *vals)
{
    uint8_t tx[2 + 12] = {0};
    tx[0] = (uint8_t)(ADS1292_CMD_WREG | addr);
    tx[1] = (uint8_t)(len - 1);
    memcpy(&tx[2], vals, len);

    spi_transaction_t t = {0};
    t.length    = (2 + len) * 8;
    t.tx_buffer = tx;
    ESP_ERROR_CHECK(spi_device_transmit(s_spi, &t));
}

/* 连续读寄存器：RREG|addr, len-1, 再读回 len 字节 -> out[] */
static void ads1292_read_regs(uint8_t addr, uint8_t len, uint8_t *out)
{
    uint8_t tx[2 + 12] = {0};
    uint8_t rx[2 + 12] = {0};
    tx[0] = (uint8_t)(ADS1292_CMD_RREG | addr);
    tx[1] = (uint8_t)(len - 1);

    spi_transaction_t t = {0};
    t.length    = (2 + len) * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    ESP_ERROR_CHECK(spi_device_transmit(s_spi, &t));

    memcpy(out, &rx[2], len);
}

static uint8_t ads1292_read_reg(uint8_t addr)
{
    uint8_t v = 0;
    ads1292_read_regs(addr, 1, &v);
    return v;
}

/* 24bit 二进制补码 -> int32（符号扩展） */
static int32_t ads1292_to_signed24(const uint8_t *p)
{
    int32_t v = ((int32_t)p[0] << 16) | ((int32_t)p[1] << 8) | (int32_t)p[2];
    if (v & 0x00800000) v |= (int32_t)0xFF000000;
    return v;
}

/*============================= DRDY 中断 =============================*/
/* 数据就绪时 DRDY 出下降沿，只给信号量，真正的 SPI 读取交给任务做 */
static void IRAM_ATTR ads1292_drdy_isr(void *arg)
{
    (void)arg;
    BaseType_t hp = pdFALSE;
    xSemaphoreGiveFromISR(s_drdy_sem, &hp);
    if (hp) portYIELD_FROM_ISR();
}

/*============================= 对外接口 =============================*/

uint8_t ads1292_get_id(void)
{
    return s_id;
}

void ads1292_get_status(uint8_t out[3])
{
    memcpy(out, s_last_status, 3);
}

uint8_t ads1292_get_loff(void)
{
    return s_loff;
}

uint32_t ads1292_get_bad_frames(void)
{
    return s_bad_frames;
}

/* 读一帧：先等 DRDY 下降沿，再把 9 字节（3 状态 + CH1 + CH2）整帧读回。
 * 这与 STM32 版一致：工作在 RDATAC 连续读取模式，不用 RDATA 命令，
 * 于是就不存在「命令与数据之间的 tSDECODE 时序」问题。
 * wait 超时返回 false（模块没接好时不会把任务卡死）。 */
bool ads1292_read_data(int32_t *ch1, int32_t *ch2)
{
    if (xSemaphoreTake(s_drdy_sem, pdMS_TO_TICKS(20)) != pdTRUE) {
        s_bad_frames++;
        return false;
    }

    uint8_t tx[9] = {0};
    uint8_t rx[9] = {0};
    spi_transaction_t t = {0};
    t.length    = 9 * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    if (spi_device_transmit(s_spi, &t) != ESP_OK) {
        s_bad_frames++;
        return false;
    }

    /* 状态字 24bit = 1100 | LOFF_STAT[4:0] | GPIO[1:0] | 13 个 0：
     * rx[0] 高 4 位恒 1100、rx[1] 低 5 位恒 0、rx[2] 恒 00，
     * 三者一起校验即可确认这一帧字节对齐（错位帧直接丢弃）。 */
    if ((rx[0] & 0xF0) != 0xC0 || (rx[1] & 0x1F) != 0x00 || rx[2] != 0x00) {
        s_bad_frames++;
        return false;
    }

    memcpy(s_last_status, rx, 3);
    s_loff = (uint8_t)(((rx[0] & 0x0F) << 1) | ((rx[1] >> 7) & 0x01));

    *ch1 = ads1292_to_signed24(&rx[3]);
    *ch2 = ads1292_to_signed24(&rx[6]);
    return true;
}

/*==================== 临时诊断：信号脚逐根翻转 ====================*/
/* 用法见 main.c 的 ADS1292_SELFTEST 开关。
 * 逐个把 SCLK/MOSI/CS0/PWDN/START 拉高再拉低（各 2 秒）并打印脚名；
 * 用万用表直流电压档，黑表笔接模块 GND、红表笔测该脚焊孔：
 *   随打印在 0V <-> 3.3V 之间跳变 -> 这根线通了
 *   电压一直不动（恒 0V 或恒 3.3V）-> 线断 / 插错孔 / 接触不良
 * MISO 与 DRDY 是输入脚，已开内部上拉，悬空时应读到 1。 */
void ads1292_selftest(void)
{
    static const struct { int pin; const char *name; } outs[] = {
        { ADS1292_PIN_SCLK,  "SCLK"  },
        { ADS1292_PIN_MOSI,  "MOSI"  },
        { ADS1292_PIN_CS,    "CS0"   },
        { ADS1292_PIN_PWDN,  "PWDN"  },
        { ADS1292_PIN_START, "START" },
    };
    const int n = (int)(sizeof(outs) / sizeof(outs[0]));

    for (int i = 0; i < n; i++) {
        gpio_config_t oc = {
            .pin_bit_mask = 1ULL << outs[i].pin,
            .mode = GPIO_MODE_INPUT_OUTPUT,
        };
        gpio_config(&oc);
        gpio_set_level(outs[i].pin, 0);
    }
    gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL << ADS1292_PIN_MISO) | (1ULL << ADS1292_PIN_DRDY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in_cfg);

    printf("\r\n===== ADS1292 接线自检（每档 2 秒）=====\r\n");
    printf("黑表笔接模块 GND，红表笔顶住该信号的焊孔看读数\r\n");

    uint32_t round = 0;
    while (1) {
        round++;
        for (int i = 0; i < n; i++) {
            gpio_set_level(outs[i].pin, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            printf("[第%lu轮 %d/%d] %s 拉高：模块该孔应为 3.3V，ESP32 回读=%d\r\n",
                   (unsigned long)round, i + 1, n, outs[i].name,
                   gpio_get_level(outs[i].pin));
            vTaskDelay(pdMS_TO_TICKS(1900));

            gpio_set_level(outs[i].pin, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            printf("[第%lu轮 %d/%d] %s 拉低：模块该孔应为 0V，  ESP32 回读=%d\r\n",
                   (unsigned long)round, i + 1, n, outs[i].name,
                   gpio_get_level(outs[i].pin));
            vTaskDelay(pdMS_TO_TICKS(1900));
        }
        printf("[第%lu轮] MISO(GPIO13)=%d DRDY(GPIO14)=%d（悬空时都应为 1）\r\n",
               (unsigned long)round,
               gpio_get_level(ADS1292_PIN_MISO),
               gpio_get_level(ADS1292_PIN_DRDY));
    }
}

/*============================= 初始化 =============================*/

/* 与 STM32 版逐字节一致的寄存器表（CONFIG1 ~ GPIO 共 11 个寄存器）。
 * 各值含义见 ads1292.h 的「寄存器配置值」注释。 */
static const uint8_t k_regs[11] = {
    ADS1292_CONFIG1_500SPS,
    ADS1292_CONFIG2_VALUE,
    ADS1292_LOFF_VALUE,
    ADS1292_CH1SET_VALUE,
    ADS1292_CH2SET_VALUE,
    ADS1292_RLD_SENS_VALUE,
    ADS1292_LOFF_SENS_VALUE,
    ADS1292_LOFF_STAT_VALUE,
    ADS1292_RESP1_VALUE,
    ADS1292_RESP2_VALUE,
    ADS1292_GPIO_VALUE,
};

/*==================== 临时诊断：线路电平（排查完删除） ====================*/
/* 把「芯片无应答」拆成两种可能：
 *   A) 芯片根本没在工作（没供电 / 一直复位 / 没时钟） -> 模块侧问题
 *   B) 芯片在工作，但 MISO 这根线不通             -> 接线问题
 * 判据（全部不需要万用表）：
 *   · MISO/DRDY 开内部上拉后仍读到 0 -> 有东西把线拉低（正常应读 1，表示悬空）
 *   · 5 根输出脚先置 1 再置 0，回读值不跟随 -> 该线对地/对电源短路或引脚损坏
 *   · 初始化完成后 DRDY 若为 0 -> 芯片真的在按 500SPS 转换（活着） */
static void ads1292_line_diag(void)
{
    static const int out_pins[] = { ADS1292_PIN_SCLK, ADS1292_PIN_MOSI, ADS1292_PIN_CS,
                                    ADS1292_PIN_PWDN, ADS1292_PIN_START };
    static const char *out_names[] = { "SCLK(12)", "MOSI(11)", "CS0(10)",
                                       "RST/PWDN(21)", "START(47)" };
    const int n = (int)(sizeof(out_pins) / sizeof(out_pins[0]));

    for (int i = 0; i < n; i++) {
        gpio_config_t c = { .pin_bit_mask = 1ULL << out_pins[i],
                            .mode = GPIO_MODE_INPUT_OUTPUT };
        gpio_config(&c);
    }
    gpio_set_level(ADS1292_PIN_PWDN, 1);    /* 芯片使能 */
    gpio_set_level(ADS1292_PIN_START, 0);

    gpio_config_t in = {
        .pin_bit_mask = (1ULL << ADS1292_PIN_MISO) | (1ULL << ADS1292_PIN_DRDY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in);
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_LOGW(TAG, "[线路] MISO(13)=%d DRDY(14)=%d  (开上拉后应为1，读到0=被拉低)",
             gpio_get_level(ADS1292_PIN_MISO), gpio_get_level(ADS1292_PIN_DRDY));

    for (int i = 0; i < n; i++) {
        gpio_set_level(out_pins[i], 1);
        vTaskDelay(pdMS_TO_TICKS(3));
        int hi = gpio_get_level(out_pins[i]);
        gpio_set_level(out_pins[i], 0);
        vTaskDelay(pdMS_TO_TICKS(3));
        int lo = gpio_get_level(out_pins[i]);
        ESP_LOGW(TAG, "[线路] %-12s 置1回读=%d 置0回读=%d %s",
                 out_names[i], hi, lo, (hi == 1 && lo == 0) ? "OK" : "异常(短接/损坏?)");
    }

    gpio_set_level(ADS1292_PIN_PWDN, 1);
    gpio_set_level(ADS1292_PIN_START, 0);
}

esp_err_t ads1292_init(void)
{
#if ADS1292_LINE_DIAG
    ads1292_line_diag();     /* 接线诊断，默认关闭，见 ads1292.h */
#endif

    /* 1. PWDN/RESET、START 输出：PWDN 先拉低做一次硬件复位，再释放 */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << ADS1292_PIN_PWDN) | (1ULL << ADS1292_PIN_START),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(ADS1292_PIN_PWDN, 0);
    gpio_set_level(ADS1292_PIN_START, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(ADS1292_PIN_PWDN, 1);
    vTaskDelay(pdMS_TO_TICKS(600));          /* 复位后需等 2^18 tCLK ≈ 512ms */

    /* 2. SPI 总线 + 设备（Mode 1，500kHz，见 ads1292.h 的速率说明） */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = ADS1292_PIN_MOSI,
        .miso_io_num = ADS1292_PIN_MISO,
        .sclk_io_num = ADS1292_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 16,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(ADS1292_SPI_HOST, &bus_cfg, SPI_DMA_DISABLED));

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = ADS1292_SPI_FREQ_HZ,
        .mode = 1,
        .spics_io_num = ADS1292_PIN_CS,
        .queue_size = 4,
        .cs_ena_pretrans = 2,
        .cs_ena_posttrans = 8,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(ADS1292_SPI_HOST, &dev_cfg, &s_spi));

    /* 3. 与 STM32 版相同：SDATAC -> 软复位 -> 再 SDATAC */
    ads1292_send_cmd(ADS1292_CMD_SDATAC);
    vTaskDelay(pdMS_TO_TICKS(100));
    ads1292_send_cmd(ADS1292_CMD_RESET);
    vTaskDelay(pdMS_TO_TICKS(600));
    ads1292_send_cmd(ADS1292_CMD_SDATAC);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 4. 读 ID 自检 */
    s_id = ads1292_read_reg(ADS1292_REG_ID);
    if (s_id == ADS1292_ID_EXPECTED) {
        ESP_LOGI(TAG, "ADS1292R 就绪，ID=0x%02X", s_id);
    } else {
        ESP_LOGE(TAG, "ADS1292 ID 异常：0x%02X（期望 0x%02X），请检查供电与接线",
                 s_id, ADS1292_ID_EXPECTED);
    }

    /* 5. 一次性写入 CONFIG1..GPIO，再整块回读核对是否写进去 */
    ads1292_write_regs(ADS1292_REG_CONFIG1, 11, k_regs);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t rb[12] = {0};
    ads1292_read_regs(ADS1292_REG_ID, 12, rb);
    ESP_LOGI(TAG, "回读 ID=%02X CFG1=%02X CFG2=%02X LOFF=%02X CH1=%02X CH2=%02X "
                  "RLD=%02X LOFFS=%02X R1=%02X R2=%02X GPIO=%02X",
             rb[0], rb[1], rb[2], rb[3], rb[4], rb[5],
             rb[6], rb[7], rb[9], rb[10], rb[11]);

    /* 6. 启动转换。顺序不能反（数据手册 8.5.2.7 RDATAC 原文）：
     *      "If the device is in RDATAC mode, a SDATAC command must be issued
     *       before any other commands can be sent to the device."
     *    => 进了 RDATAC 之后，START 这类命令会被当成数据流而忽略。
     *    因此：先在 SDATAC 下把 START 发出去，并给 START 脚一个上升沿
     *    （手册 8.5.1.9：「START 引脚拉高」或「发 START 命令」二者其一即可
     *    启动转换，这里两条路都走，互为备份），最后才进 RDATAC。
     *    —— STM32 版曾把顺序写成 RDATAC -> START，结果 START 被忽略、
     *       DRDY 永不来、一个样本都采不到，排查了很久才定位到这一行。 */
    gpio_set_level(ADS1292_PIN_START, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level(ADS1292_PIN_START, 1);        /* START 脚上升沿 */
    ads1292_send_cmd(ADS1292_CMD_START);         /* 仍在 SDATAC，命令有效 */
    vTaskDelay(pdMS_TO_TICKS(10));
    ads1292_send_cmd(ADS1292_CMD_RDATAC);        /* 最后才进连续读 */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 临时诊断：RDATAC 模式下芯片就绪时 DRDY 会被拉低并一直保持到数据被读走。
     * 这里隔 50ms 采样：读到 0 = 芯片真在按 500SPS 转换（活着）；
     * 读到 1 = 芯片没工作，或 DRDY 这根线不通。 */
    gpio_config_t drdy_diag = {
        .pin_bit_mask = 1ULL << ADS1292_PIN_DRDY,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&drdy_diag);
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGW(TAG, "[线路] 初始化后 DRDY(14)=%d  (0=芯片在转换/活着，1=没工作或线不通)",
             gpio_get_level(ADS1292_PIN_DRDY));

    /* 7. DRDY 下降沿中断（数据就绪），任务侧用信号量取数据 */
    s_drdy_sem = xSemaphoreCreateBinary();
    if (!s_drdy_sem) return ESP_ERR_NO_MEM;

    gpio_config_t drdy = {
        .pin_bit_mask = 1ULL << ADS1292_PIN_DRDY,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&drdy);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ADS1292_PIN_DRDY, ads1292_drdy_isr, NULL);

    return ESP_OK;
}
