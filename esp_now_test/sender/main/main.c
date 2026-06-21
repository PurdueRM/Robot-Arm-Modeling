#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

uint8_t receiver_mac[] = {0x48, 0x9D, 0x31, 0xE3, 0x30, 0x50};

void app_main() {
  nvs_flash_init();

  esp_event_loop_create_default();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_start();

  esp_now_init();

  int counter = 0;

  esp_now_peer_info_t peer = {0};
  memcpy(peer.peer_addr, receiver_mac, 6);
  peer.channel = 0;
  peer.encrypt = false;

  esp_now_add_peer(&peer);

  while (1) {
    counter++;

    esp_err_t err = esp_now_send(receiver_mac,
        (uint8_t *)&counter,
        sizeof(counter));

    if (err != ESP_OK) {
      printf("Send error: %d\n", err);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
     /* uint8_t mac[6];
        esp_wifi_get_mac(WIFI_IF_STA, mac);
        printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
        mac[0], mac[1], mac[2],
        mac[3], mac[4], mac[5]);
      */
}
