#include "max30102.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "MAX30102";

static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_i2c_dev;

/*============================= I2C 基础操作 ==============================*/

esp_err_t max30102_write_reg(uint8_t uch_addr, uint8_t uch_data)
{
    uint8_t buf[2] = {uch_addr, uch_data};
    return i2c_master_transmit(s_i2c_dev, buf, sizeof(buf), -1);
}

esp_err_t max30102_read_reg(uint8_t uch_addr, uint8_t *puch_data)
{
    return i2c_master_transmit_receive(s_i2c_dev, &uch_addr, 1, puch_data, 1, -1);
}

/* 对应原例程 max30102_FIFO_ReadBytes：先读并清两个中断状态寄存器，
 * 再从 REG_FIFO_DATA 连续读出 6 字节（Red[3] + IR[3]）。 */
void max30102_fifo_read_bytes(uint8_t data[6])
{
    uint8_t status;
    max30102_read_reg(REG_INTR_STATUS_1, &status);
    max30102_read_reg(REG_INTR_STATUS_2, &status);

    uint8_t reg = REG_FIFO_DATA;
    esp_err_t ret = i2c_master_transmit_receive(s_i2c_dev, &reg, 1, data, 6, -1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "FIFO read failed: %s", esp_err_to_name(ret));
    }
}

int max30102_get_int_level(void)
{
    return gpio_get_level(MAX30102_INT_PIN);
}

/*============================= 芯片初始化 ==============================*/

static esp_err_t max30102_reset(void)
{
    esp_err_t ret;
    ret = max30102_write_reg(REG_MODE_CONFIG, 0x40);
    vTaskDelay(pdMS_TO_TICKS(20));
    return ret;
}

esp_err_t max30102_init(void)
{
    /* 1. 创建 I2C 主总线 */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = MAX30102_I2C_PORT,
        .sda_io_num = MAX30102_I2C_SDA_PIN,
        .scl_io_num = MAX30102_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,   /* 使能内部上拉；模块一般自带上拉电阻 */
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_i2c_bus));

    /* 2. 在总线上挂接 MAX30102（地址 0x57） */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MAX30102_I2C_ADDR,
        .scl_speed_hz = MAX30102_I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_i2c_dev));

    /* 3. INT 引脚配置为上拉输入（对应原例程 PB9） */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MAX30102_INT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    /* 4. 探测设备是否在线 */
    ESP_ERROR_CHECK(i2c_master_probe(s_i2c_bus, MAX30102_I2C_ADDR, -1));
    ESP_LOGI(TAG, "detected at 0x%02X (SCL=%d, SDA=%d, INT=%d)",
             MAX30102_I2C_ADDR, MAX30102_I2C_SCL_PIN,
             MAX30102_I2C_SDA_PIN, MAX30102_INT_PIN);

    /* 5. 复位芯片 */
    max30102_reset();

    /* 6. 寄存器配置序列，与原 STM32 例程完全一致 */
    max30102_write_reg(REG_INTR_ENABLE_1, 0xc0);   // INTR setting
    max30102_write_reg(REG_INTR_ENABLE_2, 0x00);
    max30102_write_reg(REG_FIFO_WR_PTR, 0x00);     // FIFO_WR_PTR[4:0]
    max30102_write_reg(REG_OVF_COUNTER, 0x00);     // OVF_COUNTER[4:0]
    max30102_write_reg(REG_FIFO_RD_PTR, 0x00);     // FIFO_RD_PTR[4:0]
    max30102_write_reg(REG_FIFO_CONFIG, 0x0f);     // sample avg = 1, fifo rollover=false, fifo almost full = 17
    max30102_write_reg(REG_MODE_CONFIG, 0x03);     // 0x02 for Red only, 0x03 for SpO2 mode, 0x07 multimode LED
    max30102_write_reg(REG_SPO2_CONFIG, 0x27);     // ADC range 4096nA, 100Hz, LED pulseWidth 400uS
    max30102_write_reg(REG_LED1_PA, 0x24);         // ~7mA for LED1
    max30102_write_reg(REG_LED2_PA, 0x24);         // ~7mA for LED2
    max30102_write_reg(REG_PILOT_PA, 0x7f);        // ~25mA for Pilot LED

    return ESP_OK;
}

/*============================= Maxim 官方心率血氧算法 ==============================
 * 以下内容直接移植自原 STM32 例程（源自 Maxim/SparkFun 官方参考算法），
 * 除数据类型改 stdint 外算法逻辑未改动。
 *============================================================================*/

const uint16_t auw_hamm[31] = { 41, 276, 512, 276, 41 }; // Hamm= long16(512*hamming(5)')
//uch_spo2_table is computed as  -45.060*ratioAverage* ratioAverage + 30.354 *ratioAverage + 94.845 ;
const uint8_t uch_spo2_table[184] = { 95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
                            99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
                            100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
                            97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
                            90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
                            80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
                            66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
                            49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
                            28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5,
                            3, 2, 1 };
static int32_t an_dx[BUFFER_SIZE - MA4_SIZE]; // delta
static int32_t an_x[BUFFER_SIZE];             // ir
static int32_t an_y[BUFFER_SIZE];             // red

void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer,  int32_t n_ir_buffer_length, uint32_t *pun_red_buffer,
                              int32_t *pn_spo2, int8_t *pch_spo2_valid,
                              int32_t *pn_heart_rate, int8_t  *pch_hr_valid)
/**
* \brief        Calculate the heart rate and SpO2 level
* \par          Details
*               By detecting  peaks of PPG cycle and corresponding AC/DC of red/infra-red signal, the ratio for the SPO2 is computed.
*               Since this algorithm is aiming for Arm M0/M3. formaula for SPO2 did not achieve the accuracy due to register overflow.
*               Thus, accurate SPO2 is precalculated and save longo uch_spo2_table[] per each ratio.
*
* \param[in]    *pun_ir_buffer           - IR sensor data buffer
* \param[in]    n_ir_buffer_length      - IR sensor data buffer length
* \param[in]    *pun_red_buffer          - Red sensor data buffer
* \param[out]    *pn_spo2                - Calculated SpO2 value
* \param[out]    *pch_spo2_valid         - 1 if the calculated SpO2 value is valid
* \param[out]    *pn_heart_rate          - Calculated heart rate value
* \param[out]    *pch_hr_valid           - 1 if the calculated heart rate value is valid
*
* \retval       None
*/
{
    uint32_t un_ir_mean, un_only_once;
    int32_t k, n_i_ratio_count;
    int32_t i, s, m, n_exact_ir_valley_locs_count, n_middle_idx;
    int32_t n_th1, n_npks, n_c_min;
    int32_t an_ir_valley_locs[15];
    int32_t an_exact_ir_valley_locs[15];
    int32_t an_dx_peak_locs[15];
    int32_t n_peak_interval_sum;

    int32_t n_y_ac, n_x_ac;
    int32_t n_spo2_calc;
    int32_t n_y_dc_max, n_x_dc_max;
    int32_t n_y_dc_max_idx = 0, n_x_dc_max_idx = 0;
    int32_t an_ratio[5], n_ratio_average;
    int32_t n_nume, n_denom;
    // remove DC of ir signal
    un_ir_mean = 0;
    for (k = 0; k < n_ir_buffer_length; k++) un_ir_mean += pun_ir_buffer[k];
    un_ir_mean = un_ir_mean / n_ir_buffer_length;
    for (k = 0; k < n_ir_buffer_length; k++)  an_x[k] = pun_ir_buffer[k] - un_ir_mean;

    // 4 pt Moving Average
    for (k = 0; k < BUFFER_SIZE - MA4_SIZE; k++) {
        n_denom = (an_x[k] + an_x[k + 1] + an_x[k + 2] + an_x[k + 3]);
        an_x[k] = n_denom / (int32_t)4;
    }

    // get difference of smoothed IR signal

    for (k = 0; k < BUFFER_SIZE - MA4_SIZE - 1;  k++)
        an_dx[k] = (an_x[k + 1] - an_x[k]);

    // 2-pt Moving Average to an_dx
    for (k = 0; k < BUFFER_SIZE - MA4_SIZE - 2; k++) {
        an_dx[k] = (an_dx[k] + an_dx[k + 1]) / 2;
    }

    // hamming window
    // flip wave form so that we can detect valley with peak detector
    for (i = 0; i < BUFFER_SIZE - HAMMING_SIZE - MA4_SIZE - 2; i++) {
        s = 0;
        for (k = i; k < i + HAMMING_SIZE; k++) {
            s -= an_dx[k] * auw_hamm[k - i];
        }
        an_dx[i] = s / (int32_t)1146; // divide by sum of auw_hamm
    }


    n_th1 = 0; // threshold calculation
    for (k = 0; k < BUFFER_SIZE - HAMMING_SIZE; k++) {
        n_th1 += ((an_dx[k] > 0) ? an_dx[k] : ((int32_t)0 - an_dx[k]));
    }
    n_th1 = n_th1 / (BUFFER_SIZE - HAMMING_SIZE);
    // peak location is acutally index for sharpest location of raw signal since we flipped the signal
    maxim_find_peaks(an_dx_peak_locs, &n_npks, an_dx, BUFFER_SIZE - HAMMING_SIZE, n_th1, 8, 5); //peak_height, peak_distance, max_num_peaks

    n_peak_interval_sum = 0;
    if (n_npks >= 2) {
        for (k = 1; k < n_npks; k++)
            n_peak_interval_sum += (an_dx_peak_locs[k] - an_dx_peak_locs[k - 1]);
        n_peak_interval_sum = n_peak_interval_sum / (n_npks - 1);
        *pn_heart_rate = (int32_t)(6000 / n_peak_interval_sum); // beats per minutes
        *pch_hr_valid  = 1;
    } else {
        *pn_heart_rate = -999;
        *pch_hr_valid  = 0;
    }

    for (k = 0; k < n_npks; k++)
        an_ir_valley_locs[k] = an_dx_peak_locs[k] + HAMMING_SIZE / 2;


    // raw value : RED(=y) and IR(=X)
    // we need to assess DC and AC value of ir and red PPG.
    for (k = 0; k < n_ir_buffer_length; k++) {
        an_x[k] = pun_ir_buffer[k];
        an_y[k] = pun_red_buffer[k];
    }

    // find precise min near an_ir_valley_locs
    n_exact_ir_valley_locs_count = 0;
    for (k = 0; k < n_npks; k++) {
        un_only_once = 1;
        m = an_ir_valley_locs[k];
        n_c_min = 16777216;//2^24;
        if (m + 5 <  BUFFER_SIZE - HAMMING_SIZE  && m - 5 > 0) {
            for (i = m - 5; i < m + 5; i++)
                if (an_x[i] < n_c_min) {
                    if (un_only_once > 0) {
                        un_only_once = 0;
                    }
                    n_c_min = an_x[i];
                    an_exact_ir_valley_locs[k] = i;
                }
            if (un_only_once == 0)
                n_exact_ir_valley_locs_count++;
        }
    }
    if (n_exact_ir_valley_locs_count < 2) {
        *pn_spo2 = -999; // do not use SPO2 since signal ratio is out of range
        *pch_spo2_valid  = 0;
        return;
    }
    // 4 pt MA
    for (k = 0; k < BUFFER_SIZE - MA4_SIZE; k++) {
        an_x[k] = (an_x[k] + an_x[k + 1] + an_x[k + 2] + an_x[k + 3]) / (int32_t)4;
        an_y[k] = (an_y[k] + an_y[k + 1] + an_y[k + 2] + an_y[k + 3]) / (int32_t)4;
    }

    //using an_exact_ir_valley_locs , find ir-red DC andir-red AC for SPO2 calibration ratio
    //finding AC/DC maximum of raw ir * red between two valley locations
    n_ratio_average = 0;
    n_i_ratio_count = 0;

    for (k = 0; k < 5; k++) an_ratio[k] = 0;
    for (k = 0; k < n_exact_ir_valley_locs_count; k++) {
        if (an_exact_ir_valley_locs[k] > BUFFER_SIZE) {
            *pn_spo2 = -999; // do not use SPO2 since valley loc is out of range
            *pch_spo2_valid  = 0;
            return;
        }
    }
    // find max between two valley locations
    // and use ratio betwen AC compoent of Ir & Red and DC compoent of Ir & Red for SPO2

    for (k = 0; k < n_exact_ir_valley_locs_count - 1; k++) {
        n_y_dc_max = -16777216;
        n_x_dc_max = -16777216;
        if (an_exact_ir_valley_locs[k + 1] - an_exact_ir_valley_locs[k] > 10) {
            for (i = an_exact_ir_valley_locs[k]; i < an_exact_ir_valley_locs[k + 1]; i++) {
                if (an_x[i] > n_x_dc_max) { n_x_dc_max = an_x[i]; n_x_dc_max_idx = i; }
                if (an_y[i] > n_y_dc_max) { n_y_dc_max = an_y[i]; n_y_dc_max_idx = i; }
            }
            n_y_ac = (an_y[an_exact_ir_valley_locs[k + 1]] - an_y[an_exact_ir_valley_locs[k] ]) * (n_y_dc_max_idx - an_exact_ir_valley_locs[k]); //red
            n_y_ac = an_y[an_exact_ir_valley_locs[k]] + n_y_ac / (an_exact_ir_valley_locs[k + 1] - an_exact_ir_valley_locs[k]);

            n_y_ac = an_y[n_y_dc_max_idx] - n_y_ac;    // subracting linear DC compoenents from raw
            n_x_ac = (an_x[an_exact_ir_valley_locs[k + 1]] - an_x[an_exact_ir_valley_locs[k]]) * (n_x_dc_max_idx - an_exact_ir_valley_locs[k]); // ir
            n_x_ac = an_x[an_exact_ir_valley_locs[k]] + n_x_ac / (an_exact_ir_valley_locs[k + 1] - an_exact_ir_valley_locs[k]);
            /* 注意：原 STM32 例程此行写的是 n_y_dc_max_idx，与 Maxim 官方参考的
             * n_x_dc_max_idx 不一致，此处按原例程 1:1 保留，以保证结果与原例程一致。 */
            n_x_ac = an_x[n_y_dc_max_idx] - n_x_ac;      // subracting linear DC compoenents from raw
            n_nume = (n_y_ac * n_x_dc_max) >> 7; //prepare X100 to preserve floating value
            n_denom = (n_x_ac * n_y_dc_max) >> 7;
            if (n_denom > 0 && n_i_ratio_count < 5 && n_nume != 0) {
                /* 原例程此处由官方的 *100 改成了 *20 */
                an_ratio[n_i_ratio_count] = (n_nume * 20) / n_denom; //formular is ( n_y_ac *n_x_dc_max) / ( n_x_ac *n_y_dc_max) ;
                n_i_ratio_count++;
            }
        }
    }

    maxim_sort_ascend(an_ratio, n_i_ratio_count);
    n_middle_idx = n_i_ratio_count / 2;

    if (n_middle_idx > 1)
        n_ratio_average = (an_ratio[n_middle_idx - 1] + an_ratio[n_middle_idx]) / 2; // use median
    else
        n_ratio_average = an_ratio[n_middle_idx];

    if (n_ratio_average > 2 && n_ratio_average < 184) {
        n_spo2_calc = uch_spo2_table[n_ratio_average];
        *pn_spo2 = n_spo2_calc;
        *pch_spo2_valid  = 1;
    } else {
        *pn_spo2 = -999; // do not use SPO2 since signal ratio is out of range
        *pch_spo2_valid  = 0;
    }
}


void maxim_find_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num)
/**
* \brief        Find peaks
* \par          Details
*               Find at most MAX_NUM peaks above MIN_HEIGHT separated by at least MIN_DISTANCE
*
* \retval       None
*/
{
    maxim_peaks_above_min_height(pn_locs, pn_npks, pn_x, n_size, n_min_height);
    maxim_remove_close_peaks(pn_locs, pn_npks, pn_x, n_min_distance);
    *pn_npks = min(*pn_npks, n_max_num);
}

void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *pn_npks, int32_t  *pn_x, int32_t n_size, int32_t n_min_height)
/**
* \brief        Find peaks above n_min_height
* \par          Details
*               Find all peaks above MIN_HEIGHT
*
* \retval       None
*/
{
    int32_t i = 1, n_width;
    *pn_npks = 0;

    while (i < n_size - 1) {
        if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i - 1]) {            // find left edge of potential peaks
            n_width = 1;
            while (i + n_width < n_size && pn_x[i] == pn_x[i + n_width])    // find flat peaks
                n_width++;
            if (pn_x[i] > pn_x[i + n_width] && (*pn_npks) < 15) {                            // find right edge of peaks
                pn_locs[(*pn_npks)++] = i;
                // for flat peaks, peak location is left edge
                i += n_width + 1;
            } else
                i += n_width;
        } else
            i++;
    }
}


void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance)
/**
* \brief        Remove peaks
* \par          Details
*               Remove peaks separated by less than MIN_DISTANCE
*
* \retval       None
*/
{

    int32_t i, j, n_old_npks, n_dist;

    /* Order peaks from large to small */
    maxim_sort_indices_descend(pn_x, pn_locs, *pn_npks);

    for (i = -1; i < *pn_npks; i++) {
        n_old_npks = *pn_npks;
        *pn_npks = i + 1;
        for (j = i + 1; j < n_old_npks; j++) {
            n_dist = pn_locs[j] - (i == -1 ? -1 : pn_locs[i]); // lag-zero peak of autocorr is at index -1
            if (n_dist > n_min_distance || n_dist < -n_min_distance)
                pn_locs[(*pn_npks)++] = pn_locs[j];
        }
    }

    // Resort indices longo ascending order
    maxim_sort_ascend(pn_locs, *pn_npks);
}

void maxim_sort_ascend(int32_t *pn_x, int32_t n_size)
/**
* \brief        Sort array
* \par          Details
*               Sort array in ascending order (insertion sort algorithm)
*
* \retval       None
*/
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++) {
        n_temp = pn_x[i];
        for (j = i; j > 0 && n_temp < pn_x[j - 1]; j--)
            pn_x[j] = pn_x[j - 1];
        pn_x[j] = n_temp;
    }
}

void maxim_sort_indices_descend(int32_t *pn_x, int32_t *pn_indx, int32_t n_size)
/**
* \brief        Sort indices
* \par          Details
*               Sort indices according to descending order (insertion sort algorithm)
*
* \retval       None
*/
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++) {
        n_temp = pn_indx[i];
        for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j - 1]]; j--)
            pn_indx[j] = pn_indx[j - 1];
        pn_indx[j] = n_temp;
    }
}
