#ifndef __ADS1292_H
#define __ADS1292_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

/***************** ADS1292R 三导联心电前端（ESP32-S3 / ESP-IDF 版） ******************
 * 移植自 STM32F103C8T6 版（模块-STM32/STM32-V2.0），取数方式完全对齐：
 *   · SPI Mode 1，工作在 RDATAC 连续读取模式，不进 SDATAC 反复切换；
 *   · 由 DRDY 下降沿中断驱动，每帧读 9 字节 = 3 状态 + CH1(24bit) + CH2(24bit)；
 *   · 采样率 500 SPS（CONFIG1=0x02）；寄存器值与原版逐字节一致。
 * 硬件连接（ESP32-S3-DevKitC-1 -> 模块）：
 *   SCLK -> GPIO12   MOSI -> GPIO11   MISO -> GPIO13   CS0 -> GPIO10
 *   PWDN -> GPIO21   START -> GPIO47  DRDY -> GPIO14
 *   模块 5V -> 外部 5V（该板 5V 为 VIN 输入脚），GND -> GND
 *   CLK 脚不接（模块内部时钟，与 STM32 版一致）
 *
 * 三导联接法（Protocentral ADS1292R ECG Breakout v4，3.5mm 立体声插头）：
 *   尖端(tip) = LA  左臂，或左上胸
 *   中环(ring)= RA  右臂，或右上胸
 *   尾套(sleeve)= RL  右腿，或右下腹   <-- 右腿驱动参考极
 * 芯片内部通道映射：CH2(IN2P/IN2N) 接 LA/RA 是心电主通道；
 *                   RLD 输出接 RL；CH1(IN1P/IN1N) 留给呼吸解调。
 * 要点：
 *   · RL 极是抑制工频干扰的关键，不接 RL 波形会明显变脏；
 *   · 贴电极前用酒精擦拭皮肤并晾干，手腕可做快速测试，
 *     胸贴幅度更大、呼吸信号也更明显；
 *   · 导联脱落状态每帧都随状态字下发，见 ads1292_get_loff()。
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
/* SCLK 为什么取 500kHz：寄存器读写必须在一次 CS 拉低的事务里连续发送多个字节，
 * 而手册要求命令字节与后续字节之间至少留 tSDECODE = 4 个 tCLK ≈ 7.81us
 * （内部时钟 512kHz）。ESP32 的 SPI 事务内字节是背靠背的，没有空隙可插，
 * 所以只能把 SCLK 降到「一个字节 16us」大于 tSDECODE。
 * 数据帧（RDATAC 连读 9 字节）没有这个限制，144us 读一帧对 500SPS 绰绰有余。 */
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
#define ADS1292_CONFIG1_500SPS  0x02    /* DR=010 -> 500SPS 连续转换，与原版一致 */
#define ADS1292_CONFIG2_VALUE   0xE1    /* bit7=1、导联脱落比较器开、内部基准开、2.42V、内部时钟 */
#define ADS1292_LOFF_VALUE      0x10    /* 导联脱落阈值与电流默认值 */
#define ADS1292_CH1SET_VALUE    0x20    /* 通道1 上电，增益 2（原版用于呼吸调制） */
#define ADS1292_CH2SET_VALUE    0x00    /* 通道2 上电，增益 6，正常电极输入（心电主通道） */
#define ADS1292_RLD_SENS_VALUE  0xEC    /* RLD 输出开，RLD2N/RLD2P 使能，RLD 脱落检测关 */
#define ADS1292_LOFF_SENS_VALUE 0x0C    /* 仅通道2 的正负端做导联脱落检测 */
#define ADS1292_LOFF_STAT_VALUE 0x00
#define ADS1292_RESP1_VALUE     0xF6    /* 通道1 呼吸解调/调制开，相位 0x0D，内部时钟 */
#define ADS1292_RESP2_VALUE     0x03    /* 内部 RLDREF，32kHz */
#define ADS1292_GPIO_VALUE      0x0C
#define ADS1292_ID_EXPECTED     0x73    /* ADS1292R 的 ID 复位值 */
/*==========================================================================*/

/*===================== 状态字（每帧数据自带 24bit） =====================*/
/* 数据手册 8.5.1.2：状态字格式 = 1100 | LOFF_STAT[4:0] | GPIO[1:0] | 13 个 0
 * 高 4 位恒为 1100、第 2 字节低 5 位恒 0、第 3 字节恒 00，可用来判断这一帧
 * 是否字节对齐。导联脱落状态就藏在状态字里，每帧都有，不需要另外读寄存器。 */
#define ADS1292_STATUS_HDR      0xC0    /* rx[0] 的高 4 位应恒为 1100 */
#define ADS1292_LOFF_IN1P       0x01    /* 位含义与 08h LOFF_STAT 一致 */
#define ADS1292_LOFF_IN1N       0x02
#define ADS1292_LOFF_IN2P       0x04
#define ADS1292_LOFF_IN2N       0x08
#define ADS1292_LOFF_RLD        0x10
/*==========================================================================*/

/* 置 1 时初始化过程中会跑一遍 ads1292_line_diag()：把 SCLK/MOSI/CS/PWDN/START
 * 逐个拉高再拉低并回读，用于在模块侧用万用表确认每根线是否真通到芯片。
 * 注意它会把 SPI 引脚临时改成普通 IO，正常采集必须为 0。 */
#define ADS1292_LINE_DIAG   0

/* 初始化 ADS1292R：复位、写寄存器、进 RDATAC、启动转换、挂 DRDY 中断 */
esp_err_t ads1292_init(void);

/* 返回初始化时读到的 ID 寄存器值（正常应为 0x73） */
uint8_t   ads1292_get_id(void);

/* 读一帧：内部先等 DRDY 下降沿，再把 3 状态 + CH1 + CH2 共 9 字节整帧读回。
 * 字节错位的帧直接丢弃并返回 false；等不到 DRDY（超时 20ms）同样返回 false，
 * 这样模块没接好时不会把调用任务卡死。
 * 成功返回 true 并填充 ch1/ch2（24bit 有符号，已符号扩展） */
bool      ads1292_read_data(int32_t *ch1, int32_t *ch2);

/* 取最近一帧状态字里的 3 字节原始内容（用于诊断 SPI 对齐） */
void      ads1292_get_status(uint8_t out[3]);

/* 取最近一帧状态字里的导联脱落状态（每帧自动更新，不占用 SPI）
 * bit0=IN1P  bit1=IN1N  bit2=IN2P  bit3=IN2N  bit4=RLD；为 1 表示该端脱落/开路 */
uint8_t   ads1292_get_loff(void);

/* 累计被丢弃的错位帧数，用于判断 SPI 时序是否稳定 */
uint32_t  ads1292_get_bad_frames(void);

/* 临时诊断：逐个翻转信号脚，配合万用表在模块侧验证接线是否通到芯片。
 * 仅在排查时调用，正常使用不要调用（此函数不返回）。 */
void      ads1292_selftest(void);

#endif