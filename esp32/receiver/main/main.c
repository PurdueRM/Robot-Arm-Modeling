#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_log.h"

void recv_cb(const esp_now_recv_info_t *esp_info, const uint8_t *data, int data_len) {
  printf("Received %d bytes\n", data_len);

  printf("From: %02X:%02X:%02X:%02X:%02X:%02X\n",
           esp_info->src_addr[0], esp_info->src_addr[1], esp_info->src_addr[2],
           esp_info->src_addr[3], esp_info->src_addr[4], esp_info->src_addr[5]);

  printf("Data: %d\n", *data);
}

void app_main() {
  nvs_flash_init();

  esp_event_loop_create_default();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_start();

  esp_now_init();

  esp_now_register_recv_cb(recv_cb);

  /* uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
           mac[0], mac[1], mac[2],
           mac[3], mac[4], mac[5]);
 */
}
