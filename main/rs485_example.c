/* Uart Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "math.h"

/**
 * This is a example which echos any data it receives on UART back to the sender using RS485 interface in half duplex mode.
*/
#define TAG "RS485_ECHO_APP"

// Note: Some pins on target chip cannot be assigned for UART communication.
// Please refer to documentation for selected board and target to configure pins using Kconfig.
// #define ECHO_TEST_TXD   (CONFIG_ECHO_UART_TXD)
// #define ECHO_TEST_RXD   (CONFIG_ECHO_UART_RXD)

#define ECHO_TEST_TXD   (45)
#define ECHO_TEST_RXD   (48)
unsigned long reading1, reading2;
uint32_t  reading3;

int var = 0;
float v1 =0,avrage_value;

// RTS for RS485 Half-Duplex Mode manages DE/~RE
#define ECHO_TEST_RTS   (CONFIG_ECHO_UART_RTS)

// CTS is not used in RS485 Half-Duplex Mode
#define ECHO_TEST_CTS   (UART_PIN_NO_CHANGE)

#define BUF_SIZE        (127)
#define BAUD_RATE       (CONFIG_ECHO_UART_BAUD_RATE)

// Read packet timeout
#define PACKET_READ_TICS        (100 / portTICK_PERIOD_MS)
#define ECHO_TASK_STACK_SIZE    (2048)
#define ECHO_TASK_PRIO          (10)
#define ECHO_UART_PORT          (2)

// Timeout threshold for UART = number of symbols (~10 tics) with unchanged state on receive pin
#define ECHO_READ_TOUT          (3) // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks



static const uint16_t wCRCTable[] = {
0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };


uint16_t crc_fun(unsigned char *nData, uint16_t wLength)
{
uint8_t nTemp;
uint16_t wCRCWord = 0xFFFF;

   while (wLength--)
   {
      nTemp = *nData++ ^ wCRCWord;
      wCRCWord >>= 8;
      wCRCWord ^= wCRCTable[nTemp];
   }
   return wCRCWord;

}

static void echo_send(const int port, uint8_t *data, uint8_t length)
{
    if (uart_write_bytes(port, (const char *)data, length) != length) {
        ESP_LOGE(TAG, "Send data critical failure.");
        // add your code to handle sending failure here
        abort();
    }
}
float convert_data(uint32_t array){
        // printf("array=%lx\n",array);
		for (int i = 0; i < 9; i++){
			if(((array>>(i+23))&1) == 1){
				var = (var + pow(2,i));
			}
		}
		for(int i = 0; i<23; i++){
			if(((array>>(23-i)&1)== 1)){
				v1 = (v1 + (1/(pow(2,i))));
			}
		}
		var = var -127;
		v1 = (1+v1);
		avrage_value = v1 *(pow(2,var));
        // printf("avrage_value=%.2f\n",avrage_value);
		v1 =0;var = 0;
    return avrage_value;
}


// An example of echo test with hardware flow control on UART
static void echo_task(void *arg)
{
    const int uart_num = ECHO_UART_PORT;
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "Start RS485 application test and configure UART.");

    // Install UART driver (we don't need an event queue here)
    // In this example we don't even use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0));

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    ESP_LOGI(TAG, "UART set pins, mode and install driver.");

    // Set UART pins as per KConfig settings
    ESP_ERROR_CHECK(uart_set_pin(uart_num, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    // Set RS485 half duplex mode
    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));

    // Set read timeout of UART TOUT feature
    ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num, ECHO_READ_TOUT));

    // Allocate buffers for UART
    // uint8_t* data = (uint8_t*) malloc(BUF_SIZE);

    ESP_LOGI(TAG, "UART start recieve loop.\r\n");
    // echo_send(uart_num, "Start RS485 UART test.\r\n", 24);

    float result;
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    int crc;
    reading1=0;
    reading2=0;
    reading3=0;
    printf("get data\n");
    uint8_t modbus_request[8];
    while(1){
    modbus_request[0]= 0x01;
    modbus_request[1]= 0x03;
    modbus_request[2]= 0x00;
    modbus_request[3]= 0x01;
    modbus_request[4]= 0x00;
    modbus_request[5]= 0x02;

    crc =crc_fun(modbus_request, 6);
    modbus_request[6]= crc;
    modbus_request[7]= crc>>8;
    printf("modbus command = 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X\n",modbus_request[0],modbus_request[1],modbus_request[2],modbus_request[3],modbus_request[4],modbus_request[5],modbus_request[6],modbus_request[7]);
    // while(1){
    echo_send(uart_num, modbus_request, sizeof(modbus_request));
    int len = uart_read_bytes(uart_num, data, BUF_SIZE, PACKET_READ_TICS);
    printf("length =%d\n",len);
    // while(1){
        if (len > 0) {
            ESP_LOGI(TAG, "Received %u bytes:\n  resopnse : 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X \n", len,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8]);
            printf("ems\n");
            reading1 = (data[4]&0x000000FF);
            reading1 |= (data[3] << 8) & 0x0000FF00; 
            reading1 = reading1 & 0x0000FFFF;
            reading2 = data[6] & 0x000000FF; //Load LSB of higher 16bit
            reading2 |= (data[5] << 8) & 0x0000FF00;
            reading2 = reading2 << 16 & 0xFFFF0000;
            reading3 = reading1 | reading2;
            // break;
            printf("reading3=%ld\n",reading3); 

            result= convert_data(reading3);
            printf("ems frequency =%.2lf\n",result);
    }
    // while(1) {
    //     //Read data from UART
    //     int len = uart_read_bytes(uart_num, data, BUF_SIZE, PACKET_READ_TICS);

    //     //Write data back to UART
    //     if (len > 0) {
    //         echo_send(uart_num, "\r\n", 2);
    //         char prefix[] = "RS485 Received: [";
    //         echo_send(uart_num, prefix, (sizeof(prefix) - 1));
    //         ESP_LOGI(TAG, "Received %u bytes:", len);
    //         printf("[ ");
    //         for (int i = 0; i < len; i++) {
    //             printf("0x%.2X ", (uint8_t)data[i]);
    //             echo_send(uart_num, (const char*)&data[i], 1);
    //             // Add a Newline character if you get a return charater from paste (Paste tests multibyte receipt/buffer)
    //             if (data[i] == '\r') {
    //                 echo_send(uart_num, "\n", 1);
    //             }
    //         }
    //         printf("] \n");
    //         echo_send(uart_num, "]\r\n", 3);
    //     } else {
    //         // Echo a "." to show we are alive while we wait for input
    //         echo_send(uart_num, ".", 1);
    //         ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, 10));
    //     }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    //A uart read/write example without event queue;
    xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, ECHO_TASK_PRIO, NULL);
}
