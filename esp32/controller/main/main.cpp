#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"


#include "WT901Parser.h"

#define RXD2 16
#define TXD2 17
#define UART_PORT_NUM UART_NUM_2
#define BUF_SIZE 1024

static const char *TAG = "IMU";

void init_uart() {
    // Configure UART parameters (9600 baud, 8N1)
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
    };
    uart_config.flags = {0};
    
    // Install the UART driver and configure the pins
    uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, TXD2, RXD2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

extern "C" void app_main() {
  init_uart();
  ESP_LOGI(TAG, "UART2 Initialized. Waiting for WT901 data...");

  WT901Parser parser;
  uint8_t rx_data[128];


  // timer to make output more readable (10 Hz)
  TickType_t lastPrintTicks = xTaskGetTickCount();
  const TickType_t printIntervalTicks = pdMS_TO_TICKS(100);

  while (true) {
    int rx_len = uart_read_bytes(UART_PORT_NUM, rx_data, sizeof(rx_data), pdMS_TO_TICKS(10));

    if (rx_len > 0) {
      for (int i = 0; i < rx_len; i++) {
        parser.update(rx_data[i]);
      }
    }
    

    TickType_t currentTicks = xTaskGetTickCount();
    if (currentTicks - lastPrintTicks >= printIntervalTicks) {
      lastPrintTicks = currentTicks;
      IMUData data = parser.getData();

      float time_s = esp_timer_get_time() / 1000000.0f;

      printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                   time_s,
                   data.acc[0], data.acc[1], data.acc[2],
                   data.angle[0], data.angle[1], data.angle[2]);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

