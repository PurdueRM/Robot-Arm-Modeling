#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"

#include <fcntl.h>


#include "WT901Parser.h"

#define RXD2 16
#define TXD2 17
#define UART_PORT_NUM UART_NUM_2
#define BUF_SIZE 1024

static const char *TAG = "IMU";
static float yaw_offset = 0.0f;
static IMUData data;

/* 
void reset_imu_z_axis() {
  ESP_LOGI(TAG, "Sending Reset Command to IMU ...");

  // unlocking (entering write mode)
  uint8_t unlock_cmd[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
  uart_write_bytes(UART_PORT_NUM, (const char *) unlock_cmd, sizeof(unlock_cmd));
  vTaskDelay(pdMS_TO_TICKS(100));

  // setting heading angle to 0 0x04 (in calibration mode 0x01)
  uint8_t reset_cmd[] = {0xFF, 0xAA, 0x01, 0x04, 0x00};
  uart_write_bytes(UART_PORT_NUM, (const char *) reset_cmd, sizeof(reset_cmd));
  vTaskDelay(pdMS_TO_TICKS(100));

  // save configuration to flash
  uint8_t save_cmd[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
  uart_write_bytes(UART_PORT_NUM, (const char *) save_cmd, sizeof(save_cmd));
  vTaskDelay(pdMS_TO_TICKS(100));
}
*/

void init_uart() {
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

  // setting up keyboard input
  setvbuf(stdin, NULL, _IONBF, 0);
  int flags = fcntl(fileno(stdin), F_GETFL);
  fcntl(fileno(stdin), F_SETFL, flags | O_NONBLOCK);

  while (true) {

    // checking keyboard input (space resets)
    
    int c = fgetc(stdin);

    if (c == ' ') {
      printf("\n--- SPACEBAR DETECTED: ZEROING HEADING ANGLE ---\n");
      // reset_imu_z_axis();
      yaw_offset = data.angle[2];

    }

    int rx_len = uart_read_bytes(UART_PORT_NUM, rx_data, sizeof(rx_data), pdMS_TO_TICKS(10));

    if (rx_len > 0) {
      for (int i = 0; i < rx_len; i++) {
        parser.update(rx_data[i]);
      }
    }
    

    TickType_t currentTicks = xTaskGetTickCount();
    if (currentTicks - lastPrintTicks >= printIntervalTicks) {
      lastPrintTicks = currentTicks;
      data = parser.getData();

      float time_s = esp_timer_get_time() / 1000000.0f;

      printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                   time_s,
                   data.acc[0], data.acc[1], data.acc[2],
                   data.angle[0], data.angle[1], data.angle[2] - yaw_offset);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

