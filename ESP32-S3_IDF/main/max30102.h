#ifndef __MAX30102_H
#define __MAX30102_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"   /* I2C_NUM_0 */
#include "driver/gpio.h"

/***************** MAX30102 驱动（ESP32-S3 / ESP-IDF 版） ******************
 * 由 STM32F103C8T6 例程移植：
 *   - 软件模拟 I2C  -> ESP-IDF 新 I2C 主驱动 driver/i2c_master.h
 *   - INT 中断引脚  -> GPIO 输入
 *   - Maxim 官方心率/血氧算法保持不变
 * 硬件连接（ESP32-S3-DevKitC-1）：
 *   MAX30102 SCL -> GPIO6
 *   MAX30102 SDA -> GPIO5
 *   MAX30102 INT -> GPIO7
 *   VCC -> 3V3，GND -> GND
 **********************BEGIN***********************/

/*==============================================MAX30102 硬件引脚==================================================*/
#define MAX30102_I2C_PORT        I2C_NUM_0      /* 使用 I2C0 控制器 */
#define MAX30102_I2C_SCL_PIN     6              /* 原 STM32: PB7 */
#define MAX30102_I2C_SDA_PIN     5              /* 原 STM32: PB8 */
#define MAX30102_INT_PIN         7              /* 原 STM32: PB9，低电平有效 */
#define MAX30102_I2C_FREQ_HZ     400000         /* MAX30102 最高支持 400kHz */
/*=============================================================================================================*/

/* MAX30102 7bit I2C 地址（原例程写地址 0xAE = 0x57 << 1） */
#define MAX30102_I2C_ADDR        0x57

//register addresses
#define REG_INTR_STATUS_1 0x00
#define REG_INTR_STATUS_2 0x01
#define REG_INTR_ENABLE_1 0x02
#define REG_INTR_ENABLE_2 0x03
#define REG_FIFO_WR_PTR 0x04
#define REG_OVF_COUNTER 0x05
#define REG_FIFO_RD_PTR 0x06
#define REG_FIFO_DATA 0x07
#define REG_FIFO_CONFIG 0x08
#define REG_MODE_CONFIG 0x09
#define REG_SPO2_CONFIG 0x0A
#define REG_LED1_PA 0x0C
#define REG_LED2_PA 0x0D
#define REG_PILOT_PA 0x10
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12
#define REG_TEMP_INTR 0x1F
#define REG_TEMP_FRAC 0x20
#define REG_TEMP_CONFIG 0x21
#define REG_PROX_INT_THRESH 0x30
#define REG_REV_ID 0xFE
#define REG_PART_ID 0xFF

/* 初始化 MAX30102：创建 I2C 总线、配置 INT 引脚、复位并写入寄存器配置 */
esp_err_t max30102_init(void);

/* 寄存器读写 */
esp_err_t max30102_write_reg(uint8_t uch_addr, uint8_t uch_data);
esp_err_t max30102_read_reg(uint8_t uch_addr, uint8_t *puch_data);

/* 读取一次 FIFO：清中断标志后读出 6 字节（红光 3 字节 + 红外 3 字节） */
void max30102_fifo_read_bytes(uint8_t data[6]);

/* 读取 INT 引脚电平：0 = 有新数据（对应原例程的 MAX30102_INT 宏） */
int max30102_get_int_level(void);

/* 血氧/心率算法参数（与原例程一致，不要随意修改） */
#define FS 100
#define BUFFER_SIZE  (FS* 5)
#define HR_FIFO_SIZE 7
#define MA4_SIZE  4
#define HAMMING_SIZE  5
#ifndef min
#define min(x,y) ((x) < (y) ? (x) : (y))
#endif

/* 血氧心率算法函数（Maxim 官方） */
void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer,  int32_t n_ir_buffer_length, uint32_t *pun_red_buffer,
                                           int32_t *pn_spo2, int8_t *pch_spo2_valid,
                                           int32_t *pn_heart_rate, int8_t  *pch_hr_valid);
void maxim_find_peaks(int32_t *pn_locs, int32_t *pn_npks,  int32_t *pn_x, int32_t n_size,
                      int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num);
void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x,
                                  int32_t n_size, int32_t n_min_height);
void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance);
void maxim_sort_ascend(int32_t *pn_x, int32_t n_size);
void maxim_sort_indices_descend(int32_t *pn_x, int32_t *pn_indx, int32_t n_size);

#endif
