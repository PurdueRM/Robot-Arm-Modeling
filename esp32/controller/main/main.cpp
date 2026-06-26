#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

#include <fcntl.h>


#include "WT901Parser.h"
#include "crc.h"
#include "joystick.h"

#define RXD2 16
#define TXD2 17
#define UART_PORT_NUM_2 UART_NUM_2
#define BUF_SIZE_2 1024

#define RX1_PIN_NUM 18
#define TX1_PIN_NUM 19
#define UART_PORT_NUM_1 UART_NUM_1
#define BUF_SIZE_1 512

#define JOYSTICK_SW GPIO_NUM_25

static const char *TAG = "IMU";
static float yaw_offset = 0.0f;
static IMUData data;
static uint8_t packet_seq_num = 0;
uint8_t tx1_buf[39];

typedef struct {
  uint8_t data[9][3];
  uint8_t joystick_x;
  uint8_t joystick_y;
  uint8_t flags;
} packed_payload_t;

packed_payload_t payload[30];
joystick_t joystick;
adc_oneshot_unit_handle_t adc_handle;

void init_adc() {
  adc_oneshot_unit_init_cfg_t init_config = {
    .unit_id = ADC_UNIT_1,
  };

  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

  adc_oneshot_chan_cfg_t config = {
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_12,
  };

  ESP_ERROR_CHECK(
    adc_oneshot_config_channel(
      adc_handle,
      ADC_CHANNEL_4, // gpio32
      &config
    )
  );

  ESP_ERROR_CHECK(
    adc_oneshot_config_channel(
      adc_handle,
      ADC_CHANNEL_5, // gpio33
      &config
    )
  );

}

void init_uart_1() {
  uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0,
    .source_clk = UART_SCLK_DEFAULT,
  };

  uart_param_config(UART_PORT_NUM_1, &uart_config);
  uart_driver_install(UART_PORT_NUM_1, BUF_SIZE_1, 0, 0, NULL, 0);
  uart_set_pin(UART_PORT_NUM_1, TX1_PIN_NUM, RX1_PIN_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void init_uart_2() {
  uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0,
    .source_clk = UART_SCLK_DEFAULT,
  };

  uart_param_config(UART_PORT_NUM_2, &uart_config);
  uart_driver_install(UART_PORT_NUM_2, BUF_SIZE_2 * 2, 0, 0, NULL, 0);
  uart_set_pin(UART_PORT_NUM_2, TXD2, RXD2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void read_joystick() {
    int rawx;
    int rawy;
    adc_oneshot_read(adc_handle, ADC_CHANNEL_4, &rawx);
    adc_oneshot_read(adc_handle, ADC_CHANNEL_5, &rawy);

    joystick.x = rawx >> 4;
    joystick.y = rawy >> 4;

    joystick.sw = gpio_get_level(JOYSTICK_SW) == 0;
}

void construct_controller_packet(const uint8_t *payload, uint8_t *tx_buf) {
  tx_buf[0] = 0xA5;                 // SOF
  tx_buf[1] = 30;                   // Data len low byte
  tx_buf[2] = 0x00;                 // Data len high byte
  tx_buf[3] = packet_seq_num++;     // Sequence number (increments automatically)
  
  Append_CRC8_Check_Sum(tx_buf, 5);

  tx_buf[5] = 0x02;                 // Cmd ID Low
  tx_buf[6] = 0x03;                 // Cmd ID High

  memcpy(&tx_buf[7], payload, 30);

  Append_CRC16_Check_Sum(tx_buf, 39);
}

extern "C" void app_main() {
  init_uart_1();

  init_uart_2();

  init_adc();

  WT901Parser parser;
  uint8_t rx_data[128];

  // timer to print at 10Hz for readability
  TickType_t lastPrintTicks = xTaskGetTickCount();
  const TickType_t transmitPrintTicks = pdMS_TO_TICKS(100);

  // timer to transit at 30Hz
  TickType_t lastTransmitTicks = xTaskGetTickCount();
  const TickType_t transmitIntervalTicks = pdMS_TO_TICKS(33);

  // setting up keyboard input
  setvbuf(stdin, NULL, _IONBF, 0);
  int flags = fcntl(fileno(stdin), F_GETFL);
  fcntl(fileno(stdin), F_SETFL, flags | O_NONBLOCK);

  while (true) {

    // checking keyboard input (space resets)

    int c = fgetc(stdin);

    if (c == ' ') {
      printf("\n--- SPACEBAR DETECTED: ZEROING HEADING ANGLE ---\n");
      yaw_offset = data.angle[2];
    }

    int rx_len = uart_read_bytes(UART_PORT_NUM_2, rx_data, sizeof(rx_data), pdMS_TO_TICKS(10));

    if (rx_len > 0) {
      for (int i = 0; i < rx_len; i++) {
        parser.update(rx_data[i]);  
      }
    }

    read_joystick();

    printf("X=%4d  Y=%4d  SW=%d\n",joystick.x, joystick.y, joystick.sw);


    // TickType_t currentTicks = xTaskGetTickCount();
    // if (currentTicks - lastTransmitTicks >= transmitIntervalTicks) {
    //   lastTransmitTicks = currentTicks;
    //   data = parser.getData();
      
    //   construct_controller_packet(data, tx1_buf);

    //   uart_write_bytes(UART_PORT_NUM_1, (const char*)tx1_buf, 39);

    // }

    /*
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
    */

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

