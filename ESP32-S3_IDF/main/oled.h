#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>
#include "driver/i2c_master.h"   /* I2C_NUM_1 */

/*************** OLED（SSD1306 128x64 I2C）引脚配置 ****************/
/* ESP32-S3-DevKitC-1：OLED 独占 I2C1（与 MAX30102 的 I2C0 分开，
 * 和原 STM32 例程"两路软件 I2C"的结构保持一致）。 */
#define OLED_I2C_PORT        I2C_NUM_1      /* I2C1 控制器 */
#define OLED_I2C_SCL_PIN     9              /* 原 STM32: PB11 */
#define OLED_I2C_SDA_PIN     8              /* 原 STM32: PB10 */
#define OLED_I2C_FREQ_HZ     400000
#define OLED_I2C_ADDR        0x3C           /* 原例程写地址 0x78 = 0x3C << 1 */
/*********************END**********************/

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Refresh(void);
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1, uint8_t mode);
void OLED_ShowString(uint8_t x, uint8_t y, const char *chr, uint8_t size1, uint8_t mode);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1, uint8_t mode);
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t num, uint8_t size1, uint8_t mode);

#endif
