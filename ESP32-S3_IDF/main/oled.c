#include "oled.h"
#include "oledfont.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "OLED";

static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_i2c_dev;

/* 显存，与原例程一致：128 列 x 8 页（原数组为 [144][8]，实际只用到 128 列） */
static uint8_t OLED_GRAM[128][8];

/*============================= I2C 底层 ==============================*/

/* 对应原例程 OLED_WR_Byte：向 SSD1306 写一个命令或数据字节 */
static void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    uint8_t buf[2];
    buf[0] = (mode == OLED_DATA) ? 0x40 : 0x00;
    buf[1] = dat;
    i2c_master_transmit(s_i2c_dev, buf, sizeof(buf), -1);
}

/* 把显存刷新到屏幕，整页批量发送，比逐字节软件 I2C 快很多 */
void OLED_Refresh(void)
{
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t cmds[4] = {
            0x00,
            0xb0 + i,   // 设置页起始地址
            0x00,       // 列低地址
            0x10        // 列高地址
        };
        i2c_master_transmit(s_i2c_dev, cmds, sizeof(cmds), -1);

        uint8_t buf[129];
        buf[0] = 0x40;
        for (uint8_t n = 0; n < 128; n++)
            buf[n + 1] = OLED_GRAM[n][i];
        i2c_master_transmit(s_i2c_dev, buf, sizeof(buf), -1);
    }
}

/* 清屏 */
void OLED_Clear(void)
{
    for (uint8_t i = 0; i < 8; i++)
        for (uint8_t n = 0; n < 128; n++)
            OLED_GRAM[n][i] = 0;
    OLED_Refresh();
}

/* 画点
 * x:0~127  y:0~63  t:1 点亮  0,熄灭 */
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t)
{
    uint8_t i, m, n;
    i = y / 8;
    m = y % 8;
    n = 1 << m;
    if (t) {
        OLED_GRAM[x][i] |= n;
    } else {
        OLED_GRAM[x][i] = ~OLED_GRAM[x][i];
        OLED_GRAM[x][i] |= n;
        OLED_GRAM[x][i] = ~OLED_GRAM[x][i];
    }
}

/* 在指定位置显示一个字符（字符集与原例程完全相同）
 * size1: 6x8/6x12/8x16/12x24 对应字号 8/12/16/24
 * mode : 0,白色背景；1,正常显示 */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1, uint8_t mode)
{
    uint8_t i, m, temp, size2, chr1;
    uint8_t x0 = x, y0 = y;
    if (size1 == 8) size2 = 6;
    else size2 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * (size1 / 2);
    chr1 = chr - ' ';
    for (i = 0; i < size2; i++) {
        if (size1 == 8)         { temp = asc2_0806[chr1][i]; }
        else if (size1 == 12)   { temp = asc2_1206[chr1][i]; }
        else if (size1 == 16)   { temp = asc2_1608[chr1][i]; }
        else if (size1 == 24)   { temp = asc2_2412[chr1][i]; }
        else return;
        for (m = 0; m < 8; m++) {
            if (temp & 0x01) OLED_DrawPoint(x, y, mode);
            else             OLED_DrawPoint(x, y, !mode);
            temp >>= 1;
            y++;
        }
        x++;
        if ((size1 != 8) && ((x - x0) == size1 / 2)) { x = x0; y0 = y0 + 8; }
        y = y0;
    }
    OLED_Refresh();
}

/* 显示字符串 */
void OLED_ShowString(uint8_t x, uint8_t y, const char *chr, uint8_t size1, uint8_t mode)
{
    while ((*chr >= ' ') && (*chr <= '~')) {
        OLED_ShowChar(x, y, (uint8_t)*chr, size1, mode);
        if (size1 == 8) x += 6;
        else            x += size1 / 2;
        chr++;
    }
    OLED_Refresh();
}

/* m^n */
static uint32_t OLED_Pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--) result *= m;
    return result;
}

/* 显示数字
 * len : 数字位数 */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1, uint8_t mode)
{
    uint8_t t, temp, m = 0;
    if (size1 == 8) m = 2;
    for (t = 0; t < len; t++) {
        temp = (num / OLED_Pow(10, len - t - 1)) % 10;
        OLED_ShowChar(x + (size1 / 2 + m) * t, y, (uint8_t)(temp + '0'), size1, mode);
    }
    OLED_Refresh();
}

/* 显示汉字（字模在 oledfont.h 中：0=心 1=率 2=血 3=氧） */
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t num, uint8_t size1, uint8_t mode)
{
    uint8_t m, temp;
    uint8_t x0 = x, y0 = y;
    uint16_t i, size3 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * size1;
    for (i = 0; i < size3; i++) {
        if (size1 == 16)         { temp = Hzk1[num][i]; }
        else if (size1 == 24)    { temp = Hzk2[num][i]; }
        else if (size1 == 32)    { temp = Hzk3[num][i]; }
        else if (size1 == 64)    { temp = Hzk4[num][i]; }
        else return;
        for (m = 0; m < 8; m++) {
            if (temp & 0x01) OLED_DrawPoint(x, y, mode);
            else             OLED_DrawPoint(x, y, !mode);
            temp >>= 1;
            y++;
        }
        x++;
        if ((x - x0) == size1) { x = x0; y0 = y0 + 8; }
        y = y0;
    }
    OLED_Refresh();
}

/*============================= 初始化 ==============================*/

void OLED_Init(void)
{
    /* 1. 创建 I2C 主总线（I2C1） */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = OLED_I2C_PORT,
        .sda_io_num = OLED_I2C_SDA_PIN,
        .scl_io_num = OLED_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,   /* OLED 模块一般自带上拉电阻 */
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_i2c_bus));

    /* 2. 挂接 SSD1306（地址 0x3C） */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OLED_I2C_ADDR,
        .scl_speed_hz = OLED_I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_i2c_dev));

    /* 3. 上电等待，原例程为 200ms */
    vTaskDelay(pdMS_TO_TICKS(200));

    /* 4. SSD1306 初始化序列，与原 STM32 例程完全一致 */
    OLED_WR_Byte(0xAE, OLED_CMD); //--turn off oled panel
    OLED_WR_Byte(0x00, OLED_CMD); //---set low column address
    OLED_WR_Byte(0x10, OLED_CMD); //---set high column address
    OLED_WR_Byte(0x40, OLED_CMD); //--set start line address
    OLED_WR_Byte(0x81, OLED_CMD); //--set contrast control register
    OLED_WR_Byte(0xCF, OLED_CMD); // Set SEG Output Current Brightness
    OLED_WR_Byte(0xA1, OLED_CMD); //--Set SEG/Column Mapping
    OLED_WR_Byte(0xC8, OLED_CMD); //Set COM/Row Scan Direction
    OLED_WR_Byte(0xA6, OLED_CMD); //--set normal display
    OLED_WR_Byte(0xA8, OLED_CMD); //--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3f, OLED_CMD); //--1/64 duty
    OLED_WR_Byte(0xD3, OLED_CMD); //-set display offset
    OLED_WR_Byte(0x00, OLED_CMD); //-not offset
    OLED_WR_Byte(0xd5, OLED_CMD); //--set display clock divide ratio/oscillator frequency
    OLED_WR_Byte(0x80, OLED_CMD); //--set divide ratio, Set Clock as 100 Frames/Sec
    OLED_WR_Byte(0xD9, OLED_CMD); //--set pre-charge period
    OLED_WR_Byte(0xF1, OLED_CMD); //Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    OLED_WR_Byte(0xDA, OLED_CMD); //--set com pins hardware configuration
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD); //--set vcomh
    OLED_WR_Byte(0x30, OLED_CMD); //Set VCOM Deselect Level
    OLED_WR_Byte(0x20, OLED_CMD); //-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_WR_Byte(0x02, OLED_CMD); //
    OLED_WR_Byte(0x8D, OLED_CMD); //--set Charge Pump enable/disable
    OLED_WR_Byte(0x14, OLED_CMD); //--set(0x10) disable
    OLED_Clear();
    OLED_WR_Byte(0xAF, OLED_CMD);

    ESP_LOGI(TAG, "SSD1306 ready at 0x%02X (SCL=%d, SDA=%d)",
             OLED_I2C_ADDR, OLED_I2C_SCL_PIN, OLED_I2C_SDA_PIN);
}
