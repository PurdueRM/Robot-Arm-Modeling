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

enum WT901_CalibMode {
    CALIB_NORMAL         = 0x00,
    CALIB_ACCEL          = 0x01,
    CALIB_HEADING_ZERO   = 0x04,
    CALIB_MAG_SPHERICAL  = 0x07,
};

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

void enter_write_mode() {
  ESP_LOGI(TAG, "Entering Write Mode ...");

  uint8_t unlock_cmd[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
  uart_write_bytes(UART_PORT_NUM, (const char *) unlock_cmd, sizeof(unlock_cmd));
  vTaskDelay(pdMS_TO_TICKS(100));

}

void exit_write_mode() {
  uint8_t save_cmd[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
  uart_write_bytes(UART_PORT_NUM, (const char *) save_cmd, sizeof(save_cmd));
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_LOGI(TAG, "Exited Write Mode");
}

void set_imu_calibration_mode(WT901_CalibMode mode) {
  uint8_t mode_cmd[] = {0xFF, 0xAA, 0x01, (uint8_t)mode, 0x00};
  uart_write_bytes(UART_PORT_NUM, (const char*)mode_cmd, sizeof(mode_cmd));
  vTaskDelay(pdMS_TO_TICKS(100));
}

extern "C" void app_main() {
  init_uart();

  // setting up keyboard input
  setvbuf(stdin, NULL, _IONBF, 0);
  int flags = fcntl(fileno(stdin), F_GETFL);
  fcntl(fileno(stdin), F_SETFL, flags | O_NONBLOCK);

  printf("\n==================================\n");
  printf("       IMU Calibration Menu       \n");
  printf("==================================\n");
  printf("[0] Zero Heading\n");
  printf("[1] Calibrate Accelerometer\n");
  printf("[2] Calibrate Magnetometer\n");
  printf("==================================\n");
  printf("Please enter your selection (1-4): ");

  while (true) {
    int c = fgetc(stdin);

    switch (c) {
      case '0':
        enter_write_mode();
        ESP_LOGI(TAG, "Zeroing Heading ...");
        set_imu_calibration_mode(CALIB_HEADING_ZERO);
        ESP_LOGI(TAG, "Finished Zeroing Heading ...");
        exit_write_mode();
        break;
      case '1':
        printf("\n==========================================\n");
        printf("   ACCELEROMETER CALIBRATION\n");
        printf("==========================================\n");
        printf("1. Place the controller perfectly flat on a table.\n");
        printf("2. Do not bump or vibrate the table.\n");
        printf("3. Press 'ENTER' to begin...\n");

        while (fgetc(stdin) != '\n') { vTaskDelay(pdMS_TO_TICKS(10)); }

        printf("\n>>> Calibrating... Keep still for 3 seconds...\n");
        enter_write_mode();
        set_imu_calibration_mode(CALIB_ACCEL);
        ESP_LOGI(TAG, "Calibrating... 3s");
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "Calibrating... 2s");
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        ESP_LOGI(TAG, "Calibrating... 1s");
        vTaskDelay(pdMS_TO_TICKS(1000));

        exit_write_mode();

        printf(">>> Accelerometer calibration saved.\n\n");
        break;
      
      case '2':
        printf("\n==========================================\n");
        printf("   MAGNETIC FIELD CALIBRATION\n");
        printf("==========================================\n");
        printf("1. Hold the controller in the air.\n");
        printf("2. Press 'ENTER' to begin the recording mode.\n");

        while (fgetc(stdin) != '\n') { vTaskDelay(pdMS_TO_TICKS(10)); }

        enter_write_mode();

        printf("\n>>> Recording... Spin the controller in a 3D Figure-8.\n");
        printf(">>> Keep spinning it to map all sides of the sphere.\n");
        printf(">>> Press 'ENTER' again when you are finished...\n");
        
        set_imu_calibration_mode(CALIB_MAG_SPHERICAL);

        while (fgetc(stdin) != '\n') { vTaskDelay(pdMS_TO_TICKS(10)); }

        exit_write_mode();

        printf("\n>>> Magnetic calibration saved.\n\n");

      case '\n':
      case '\r':
        break;
      
      default:
        printf("Invalid Option. Try again!\n");
        break;
    }
  }
}

