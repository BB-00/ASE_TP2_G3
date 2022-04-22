#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include "esp_rom_gpio.h"
#include "soc/uart_periph.h"

#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_log.h"

#include <sys/fcntl.h>
#include <sys/errno.h>
#include <sys/unistd.h>
#include <sys/select.h>
#include "esp_vfs.h"
#include "esp_vfs_dev.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
#if CONFIG_IDF_TARGET_ESP32
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
#elif CONFIG_IDF_TARGET_ESP32S2
static const adc_channel_t channel = ADC_CHANNEL_6;     // GPIO7 if ADC1, GPIO17 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

uint32_t adc_reading = 0;

static const char* TAG = "uart_echo";

static void light_sleep_task(void* arg)
{
        printf("Entering light sleep\n");
        /* To make sure the complete line is printed before entering sleep mode,
         * need to wait until UART TX FIFO is empty:
         */
        esp_sleep_enable_timer_wakeup(5000000);  //5 segundos   
        uart_wait_tx_idle_polling(CONFIG_ESP_CONSOLE_UART_NUM);

        /* Get timestamp before entering sleep */
        int64_t t_before_us = esp_timer_get_time();

        /* Enter sleep mode */
        esp_light_sleep_start();

        /* Get timestamp after waking up from sleep */
        int64_t t_after_us = esp_timer_get_time();
}

static void uart_echo(void *arg)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_0, 2*1024, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);

    //Configure ADC
    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);

    while (1) {

        int fd;

        if ((fd = open("/dev/uart/0", O_RDWR)) == -1) {
            ESP_LOGE(TAG, "Cannot open UART");
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }

        // We have a driver now installed so set up the read/write functions to use driver also.
        esp_vfs_dev_uart_use_driver(0);

        while (1) { 
            int previous_read = 0;

            adc_reading = adc1_get_raw((adc1_channel_t)channel);
            //Convert adc_reading to voltage in mV
            uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);


            int s;
            fd_set rfds;
            struct timeval tv = {
                .tv_sec = 3,
                .tv_usec = 0,
            };

            FD_ZERO(&rfds);
            FD_SET(fd, &rfds);

            s = select(fd + 1, &rfds, NULL, NULL, &tv);

            if (s < 0) {
                ESP_LOGE(TAG, "Select failed: errno %d", errno);
                break;
            } else if (s == 0) {
                xTaskCreate(light_sleep_task, "light_sleep_task", 4096, NULL, 6, NULL);
                ESP_LOGI(TAG, "Timeout has been reached and nothing has been received, Voltage is : %d", voltage);
            } else {
                if (FD_ISSET(fd, &rfds)) {
                    char buf;
                    if (read(fd, &buf, 1) > 0) {
                        ESP_LOGI(TAG, "Voltage is :%d,  Received: %c ", voltage, buf);
                        // Note: Only one character was read even the buffer contains more. The other characters will
                        // be read one-by-one by subsequent calls to select() which will then return immediately
                        // without timeout.
                    } else {
                        ESP_LOGE(TAG, "UART read error");
                        break;
                    }
                } else {
                    ESP_LOGE(TAG, "No FD has been set in select()");
                    break;
                }
            }   
            
        }

        close(fd);
    }

    vTaskDelete(NULL);
}

void app_main(void)
{
    xTaskCreate(uart_echo, "uart_echo", 4*1024, NULL, 5, NULL);
}