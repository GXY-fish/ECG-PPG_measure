#ifndef __ADS1292_H
#define __ADS1292_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

/***************** ADS1292R 三导联心电前端（ESP32-S3 / ESP-IDF 版） ******************
 * 独立调试工程：只跑 ADS1292R，不包含 MAX30102 / OLED。
 * 数据取法与 STM32 参考工程一致：
 *   · SPI Mode 1，RDATAC 连续读取，不进 SDATAC 反复切换；
 *   · DRDY 下降沿中断驱动，每帧读 9 字节 = 3 状态 + CH1(24bit) + CH2(24bit)；
 *   · 采样率 250 SPS（CONFIG1=0x01）；寄存器值其余与原版逐字节一致。
 * 硬件连接（ESP32-S3-DevKitC-1 -> 模块）：
 *   SCLK -> GPIO12   MOSI -> GPIO11   MISO -> GPIO13   CS0 -> GPIO10
 *   PWDN -> GPIO21   START -> GPIO47  DRDY -> GPIO14
 *   模块 5V 丝印脚 -> 3V3（不要接 5V），GND -> GND
 *   CLK 脚不接（模块内部时钟）
 **********************BEGIN***********************/

/*============================== SPI 引脚与速率 ==============================*/
#define ADS1292_SPI_HOST        SPI2_HOST
#define ADS1292_PIN_SCLK        12
#define ADS1292_PIN_MOSI        11
#define ADS1292_PIN_MISO        13
#define ADS1292_PIN_CS          10
#define ADS1292_PIN_PWDN        21
#define ADS1292_PIN_START       47
#define ADS1292_PIN_DRDY        14
#define ADS1292_SPI_FREQ_HZ     (500 * 1000)
/* SCLK 取 500kHz 的原因：寄存器读写必须在一次 CS 拉低的事务内连续发多个字节，
 * 手册要求命令字节与后续字节之间至少 tSDECODE = 4 个 tCLK ≈ 7.81us，
 * 而 ESP32 SPI 事务内字节是背靠背的，所以只能让一个字节的时长（16us）大于它。 */
/*==========================================================================*/

/*=============================== SPI 命令（Table 15） ===============================*/
#define ADS1292_CMD_WAKEUP      0x02
#define ADS1292_CMD_STANDBY     0x04
#define ADS1292_CMD_RESET       0x06
#define ADS1292_CMD_START       0x08
#define ADS1292_CMD_STOP        0x0A
#define ADS1292_CMD_RDATAC      0x10
#define ADS1292_CMD_SDATAC      0x11
#define ADS1292_CMD_RDATA       0x12
#define ADS1292_CMD_RREG        0x20
#define ADS1292_CMD_WREG        0x40
/*==========================================================================*/

/*=============================== 寄存器地址（Table 16） ===============================*/
#define ADS1292_REG_ID          0x00
#define ADS1292_REG_CONFIG1     0x01
#define ADS1292_REG_CONFIG2     0x02
#define ADS1292_REG_LOFF        0x03
#define ADS1292_REG_CH1SET      0x04
#define ADS1292_REG_CH2SET      0x05
#define ADS1292_REG_RLD_SENS    0x06
#define ADS1292_REG_LOFF_SENS   0x07
#define ADS1292_REG_LOFF_STAT   0x08
#define ADS1292_REG_RESP1       0x09
#define ADS1292_REG_RESP2       0x0A
#define ADS1292_REG_GPIO        0x0B
/*==========================================================================*/

/*===================== 寄存器配置值（与 STM32 版逐字节一致） =====================*/
#define ADS1292_CONFIG1_250SPS  0x01    /* DR=001 -> 250SPS 连续转换 */
#define ADS1292_CONFIG2_VALUE   0xE1    /* 导联脱落比较器开、内部基准开、内部时钟 */
#define ADS1292_LOFF_VALUE      0x10
#define ADS1292_CH1SET_VALUE    0x20    /* 通道1 上电，增益 2 */
#define ADS1292_CH2SET_VALUE    0x00    /* 通道2 上电，增益 6，正常电极输入 */
#define ADS1292_RLD_SENS_VALUE  0xEC
#define ADS1292_LOFF_SENS_VALUE 0x0C
#define ADS1292_LOFF_STAT_VALUE 0x00
#define ADS1292_RESP1_VALUE     0xF6
#define ADS1292_RESP2_VALUE     0x03
#define ADS1292_GPIO_VALUE      0x0C
#define ADS1292_ID_EXPECTED     0x73    /* ADS1292R 的 ID 复位值 */
/*==========================================================================*/

/*===================== 状态字（每帧数据自带 24bit） =====================*/
#define ADS1292_STATUS_HDR      0xC0
#define ADS1292_LOFF_IN1P       0x01
#define ADS1292_LOFF_IN1N       0x02
#define ADS1292_LOFF_IN2P       0x04
#define ADS1292_LOFF_IN2N       0x08
#define ADS1292_LOFF_RLD        0x10
/*==========================================================================*/

/* 启动时打印各信号脚的线路电平诊断（调试工程默认打开） */
#define ADS1292_LINE_DIAG       1

/* 初始化 ADS1292R：复位、写寄存器、进 RDATAC、启动转换、挂 DRDY 中断 */
esp_err_t ads1292_init(void);

/* 返回初始化时读到的 ID 寄存器值（正常应为 0x73） */
uint8_t   ads1292_get_id(void);

/* 读一帧：内部先等 DRDY 就绪，再把 3 状态 + CH1 + CH2 共 9 字节整帧读回。
 * 字节错位的帧直接丢弃并返回 false；等不到 DRDY（超时 20ms）同样返回 false。 */
bool      ads1292_read_data(int32_t *ch1, int32_t *ch2);

/* 取最近一帧状态字里的 3 字节原始内容（用于诊断 SPI 对齐） */
void      ads1292_get_status(uint8_t out[3]);

/* 取最近一帧状态字里的导联脱落状态（每帧自动更新，不占用 SPI）
 * bit0=IN1P bit1=IN1N bit2=IN2P bit3=IN2N bit4=RLD；为 1 表示该端脱落/开路 */
uint8_t   ads1292_get_loff(void);

/* 累计被丢弃的错位帧数 */
uint32_t  ads1292_get_bad_frames(void);

/* 诊断：逐个翻转信号脚，配合万用表在模块侧验证接线（此函数不返回） */
void      ads1292_selftest(void);

#endif
