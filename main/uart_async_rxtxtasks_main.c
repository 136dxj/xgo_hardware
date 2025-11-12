/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "icm20948.h"
static const int RX_BUF_SIZE = 1024;
#define I2C_MASTER_SCL_IO  19        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO  18        /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM     I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000    /*!< I2C master clock frequency */
static const char *TAG = "icm test";
static icm20948_handle_t icm20948 = NULL;

#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)

#define TXD_PIN1 (GPIO_NUM_14)
#define RXD_PIN1 (GPIO_NUM_13)
#define HLS_PRESENT_POSITION_L 56
#define HLS_MODE 0x21
#define HLS_KP 0x15
#define HLS_KD 0x16
icm20948_acce_value_t acce,acce1;
icm20948_gyro_value_t gyro,gyro1;
uint8_t pBuf[20];
static esp_err_t i2c_bus_init(void)
{
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

	esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
	if (ret != ESP_OK)
		return ret;

	return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t icm20948_configure(icm20948_acce_fs_t acce_fs, icm20948_gyro_fs_t gyro_fs)
{
	esp_err_t ret;

	/*
	 * One might need to change ICM20948_I2C_ADDRESS to ICM20948_I2C_ADDRESS_1
	 * if address pin pulled low (to GND)
	 */
	icm20948 = icm20948_create(I2C_MASTER_NUM, ICM20948_I2C_ADDRESS_1);
	if (icm20948 == NULL) {
		ESP_LOGE(TAG, "ICM20948 create returned NULL!");
		return ESP_FAIL;
	}
	ESP_LOGI(TAG, "ICM20948 creation successfull!");

	ret = icm20948_reset(icm20948);
	if (ret != ESP_OK)
		return ret;

	vTaskDelay(10 / portTICK_PERIOD_MS);

	ret = icm20948_wake_up(icm20948);
	if (ret != ESP_OK)
		return ret;

	ret = icm20948_set_bank(icm20948, 0);
	if (ret != ESP_OK)
		return ret;

	uint8_t device_id;
	ret = icm20948_get_deviceid(icm20948, &device_id);
	if (ret != ESP_OK)
		return ret;
	ESP_LOGI(TAG, "0x%02X", device_id);
	if (device_id != ICM20948_WHO_AM_I_VAL)
		return ESP_FAIL;

	ret = icm20948_set_gyro_fs(icm20948, gyro_fs);
	if (ret != ESP_OK)
		return ESP_FAIL;

	ret = icm20948_set_acce_fs(icm20948, acce_fs);
	if (ret != ESP_OK)
		return ESP_FAIL;

	return ret;
}
void init_pbuf()
{
    int i=0;
    uint8_t ID=0xfe;
	uint8_t checkSum = 0x00;
    pBuf[0] = 0xff;
	pBuf[1] = 0xff;
    pBuf[2] = 33;//id
	pBuf[3] = 0x02;//length
	pBuf[4] = 0x01;//order
	// pBuf[5] = 0x17;//addr
	// pBuf[6] = 0x01;//1B
    for(i=2;i<5;i++)
	    checkSum = checkSum + pBuf[i];
	pBuf[5] = ~checkSum;
}
void init(void) {

    const uart_config_t uart_config1 = {
        .baud_rate = 1000000,
        .data_bits=UART_DATA_8_BITS,                //8位数据位
        .flow_ctrl=UART_HW_FLOWCTRL_DISABLE,        //无硬件流控制
        .parity=UART_PARITY_DISABLE,                //无校验位
        .stop_bits=UART_STOP_BITS_1,                 //1位停止位
        .source_clk = UART_SCLK_APB,  // ESP32-S3支持APB时钟
    };

    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config1));
    uart_set_pin(UART_NUM_2, TXD_PIN1, RXD_PIN1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}
int sendData2(const char* logName, unsigned char* data, int length)
{
    const int txBytes = uart_write_bytes(UART_NUM_2, data, length);
    // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}
int sendData3(unsigned char* data, int length)
{
    const int txBytes = uart_write_bytes(UART_NUM_2, data, length);
    // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}
int sendData(const char* logName, const char* data)
{
    float temperature = 25.6f;
    char buffer[32];
    esp_err_t ret1, ret2;
    snprintf(buffer, sizeof(buffer), "Temperature: %.2f", temperature);
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    ret1 = icm20948_get_acce(icm20948, &acce1);
    ret2 = icm20948_get_gyro(icm20948, &gyro1);
    snprintf(buffer, sizeof(buffer), "Temperature: %.2f", gyro1.gyro_z);
    // ESP_LOGI(logName, "TAG: '%s'", buffer);

    return txBytes;
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        sendData(TX_TASK_TAG, "Hello world");
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}
int sync_read(uint8_t* id, uint8_t length, uint8_t* rev)
{
    int i;
	uint8_t checkSum = 0x00;
    int rxBytes;
    uint8_t rBuf[30];
    rBuf[0] = 0xff;
	rBuf[1] = 0xff;
    rBuf[2] = 0xfe;//id
	rBuf[3] = 4 + length;//length
	rBuf[4] = 0x82;//order
	rBuf[5] = 0x38;//addr
	rBuf[6] = 8;//nB
    for(i=0;i<length;i++)
        rBuf[7+i] = id[i];
    for(i=2;i<7+length;i++)
	    checkSum = checkSum + rBuf[i];
	rBuf[7+length] = ~checkSum;
    sendData3(rBuf, 8+length);
    rxBytes = uart_read_bytes(UART_NUM_2, rev, RX_BUF_SIZE, 5 / portTICK_PERIOD_MS);
    return rxBytes;
}
int read_nB(uint8_t id, uint8_t num, uint8_t addr, uint8_t* data)
{
    int i;
	uint8_t checkSum = 0x00;
    int rxBytes;
    uint8_t rBuf[8];
    rBuf[0] = 0xff;
	rBuf[1] = 0xff;
    rBuf[2] = id;//id
	rBuf[3] = 0x04;//length
	rBuf[4] = 0x02;//order
	rBuf[5] = addr;//addr
	rBuf[6] = num;//nB
    for(i=2;i<7;i++)
	    checkSum = checkSum + rBuf[i];
	rBuf[7] = ~checkSum;
    sendData3(rBuf, 8);
    rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 5 / portTICK_PERIOD_MS);
    return rxBytes;
}
int write_1B(uint8_t id, uint8_t addr, uint8_t data, uint8_t* rev)
{
    int i;
	uint8_t checkSum = 0x00;
    int rxBytes;
    uint8_t wBuf[8];
    wBuf[0] = 0xff;
	wBuf[1] = 0xff;
    wBuf[2] = id;//id
	wBuf[3] = 0x04;//length
	wBuf[4] = 0x03;//order
	wBuf[5] = addr;//addr
	wBuf[6] = data;//nB
    for(i=2;i<7;i++)
	    checkSum = checkSum + wBuf[i];
	wBuf[7] = ~checkSum;
    sendData3(wBuf, 8);
    rxBytes = uart_read_bytes(UART_NUM_2, rev, RX_BUF_SIZE, 5 / portTICK_PERIOD_MS);
    return rxBytes;
}
int set_motor_pos(uint8_t id, uint16_t pos, uint8_t* rev)// set current pos as zero pos
{
    // FF FF 01 02 0A F2
    int i;
	uint8_t checkSum = 0x00;
    int rxBytes;
    uint8_t wBuf[8];
    wBuf[0] = 0xff;
	wBuf[1] = 0xff;
    wBuf[2] = id;//id
	wBuf[3] = 0x04;//length
	wBuf[4] = 0x0b;//order
    wBuf[5] = pos&0x00ff;
    wBuf[6] = pos>>8;
    for(i=2;i<7;i++)
	    checkSum = checkSum + wBuf[i];
	wBuf[7] = ~checkSum;
    sendData3(wBuf, 8);
    rxBytes = uart_read_bytes(UART_NUM_2, rev, RX_BUF_SIZE, 5 / portTICK_PERIOD_MS);
    return rxBytes;
}
int set_kp(uint8_t id, uint8_t kp, uint8_t* data)
{
    return write_1B(id, HLS_KP, kp, data);
}
int set_kd(uint8_t id, uint8_t kd, uint8_t* data)
{
    return write_1B(id, HLS_KD, kd, data);
}
int read_4B(uint8_t id, uint8_t addr, uint8_t* data)
{
    return read_nB(id, 4, addr, data);
}
int read_angle_vel(uint8_t id, uint8_t* data)
{
    return read_4B(id, HLS_PRESENT_POSITION_L, data);
}
int read_mode(uint8_t id, uint8_t* data)
{
    return read_nB(id, 1, HLS_MODE, data);
}
int set_mode(uint8_t id, uint8_t mode, uint8_t* rev)
{
    return write_1B(id, HLS_MODE, mode, rev);
}
static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    // esp_err_t ret1, ret2;
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    int rxBytes;
    uint8_t id[2];
    
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    id[0]=33;
    id[1]=32;
    // rxBytes = read_mode(33, data);
    // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
    // vTaskDelay(200 / portTICK_PERIOD_MS);
    // rxBytes = set_mode(33, 1, data);
    // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
    // vTaskDelay(200 / portTICK_PERIOD_MS);
    // rxBytes = set_kd(33, 15, data);
    // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
    // vTaskDelay(200 / portTICK_PERIOD_MS);
    while (1) {
        // rxBytes = read_nB(33, 1, HLS_KD, data);
        rxBytes = sync_read(id, 2, data);
        ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        }
    free(data);
}

void app_main(void)
{
    init();
    init_pbuf();
    ESP_LOGI(TAG, "Starting ICM test");
	esp_err_t ret = i2c_bus_init();
	ESP_LOGI(TAG, "I2C bus initialization: %s", esp_err_to_name(ret));
    ret = icm20948_configure(ACCE_FS_2G, GYRO_FS_1000DPS);
    if (ret != ESP_OK) {
		ESP_LOGE(TAG, "ICM configuration failure");
		vTaskDelete(NULL);
	}
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}
