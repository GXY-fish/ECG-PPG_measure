/* ADS1292R 独立调试工程（ESP32-S3）
 *
 * 只跑 ADS1292R：不含 MAX30102、不含 OLED。
 * 上电后先打印线路电平诊断、ID、寄存器回读，然后持续按 250SPS 读帧，
 * 每帧把 CH1/CH2 的原始值通过 USB 发给上位机；所有滤波都放在上位机做，
 * 这样调参数不用重新烧录，也方便随时关掉对比。
 *
 * 接线（模块丝印 -> ESP32-S3）：
 *   SCK -> GPIO12   SI -> GPIO11   SO -> GPIO13   CS0 -> GPIO10
 *   DN  -> GPIO21   RT -> GPIO47   DY -> GPIO14
 *   GND -> GND      5V  -> 3V3（不要接 5V）      CLK -> 不接
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "ads1292.h"

static const char *TAG = "MAIN";

/*==================== 上位机数据帧（16 字节，小端） ====================
 *   AA 55 | CH1 int32 | CH2 int32 | PPG int32 | LOFF u8 | SUM u8
 *   SUM = sum(b[2..14]) & 0xFF
 * 与主工程（含 MAX30102）的帧格式完全一致，上位机可通用。
 * 本工程无 PPG，该字段固定填 0；CH1/CH2 发送原始值，不做任何片上滤波。
 * 走 USB-Serial-JTAG（Type-C 原生 USB），与烧录共用同一根线。 */
static void put_i32(uint8_t *p, int32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static void send_frame(int32_t ch1, int32_t ch2, uint8_t loff)
{
    uint8_t b[16] = {0xAA, 0x55};
    put_i32(&b[2],  ch1);
    put_i32(&b[6],  ch2);
    put_i32(&b[10], 0);          /* PPG 占位 */
    b[14] = loff;
    uint8_t sum = 0;
    for (int i = 2; i < 15; i++) sum = (uint8_t)(sum + b[i]);
    b[15] = sum;
    /* 超时 0 = 不阻塞：缓冲满了就丢这一帧，不能把采集卡死 */
    usb_serial_jtag_write_bytes(b, sizeof(b), 0);
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(300));

    /* 装 USB-Serial-JTAG 驱动，才能用 write_bytes 高速发二进制帧；
     * 已由控制台安装过时会返回错误，忽略即可。 */
    usb_serial_jtag_driver_config_t uj_cfg = {
        .tx_buffer_size = 4096,
        .rx_buffer_size = 256,
    };
    usb_serial_jtag_driver_install(&uj_cfg);

    printf("\r\n===== ADS1292R 独立调试工程 =====\r\n");

    ads1292_init();

    bool ok = (ads1292_get_id() == ADS1292_ID_EXPECTED);
    if (!ok) {
        ESP_LOGE(TAG, "ID=0x%02X（期望 0x73），芯片无应答；"
                      "DRDY(14)=%d MISO(13)=%d",
                 ads1292_get_id(),
                 gpio_get_level(ADS1292_PIN_DRDY),
                 gpio_get_level(ADS1292_PIN_MISO));
    } else {
        ESP_LOGI(TAG, "开始采集，250SPS；发送原始值，滤波由上位机完成");
    }

    uint32_t frames = 0, fails = 0, printed = 0;
    int32_t  ch1 = 0, ch2 = 0;
    TickType_t t_last = xTaskGetTickCount();

    while (1) {
        if (ads1292_read_data(&ch1, &ch2)) {
            frames++;

            /* 每帧都发一包原始数据给上位机（250 帧/秒） */
            send_frame(ch1, ch2, ads1292_get_loff());

            /* 每 50 帧（约 0.2s）打印一次原始值，方便和上位机对照 */
            if (++printed >= 50) {
                printed = 0;
                uint8_t st[3];
                ads1292_get_status(st);
                printf("R1=%ld R2=%ld LOFF=0x%02X ST=%02X%02X%02X\r\n",
                       (long)ch1, (long)ch2,
                       ads1292_get_loff(), st[0], st[1], st[2]);
            }
        } else {
            fails++;
        }

        /* 每秒统计一次 */
        if ((xTaskGetTickCount() - t_last) >= pdMS_TO_TICKS(1000)) {
            t_last = xTaskGetTickCount();
            ESP_LOGI(TAG, "帧=%lu 失败=%lu 错位=%lu | DRDY=%d MISO=%d",
                     (unsigned long)frames, (unsigned long)fails,
                     (unsigned long)ads1292_get_bad_frames(),
                     gpio_get_level(ADS1292_PIN_DRDY),
                     gpio_get_level(ADS1292_PIN_MISO));
            frames = 0;
            fails  = 0;
        }
    }
}
