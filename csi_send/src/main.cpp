#include <Arduino.h>

extern "C" {
#include "esp_err.h"
#include "esp_event.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
}

#ifndef CSI_SEND_CHANNEL
#define CSI_SEND_CHANNEL 11
#endif

#ifndef CSI_SEND_FREQUENCY
#define CSI_SEND_FREQUENCY 100
#endif

#if CSI_SEND_FREQUENCY <= 0
#error "CSI_SEND_FREQUENCY must be greater than 0"
#endif

#ifndef CSI_SEND_MAC0
#define CSI_SEND_MAC0 0x1a
#endif
#ifndef CSI_SEND_MAC1
#define CSI_SEND_MAC1 0x00
#endif
#ifndef CSI_SEND_MAC2
#define CSI_SEND_MAC2 0x00
#endif
#ifndef CSI_SEND_MAC3
#define CSI_SEND_MAC3 0x00
#endif
#ifndef CSI_SEND_MAC4
#define CSI_SEND_MAC4 0x00
#endif
#ifndef CSI_SEND_MAC5
#define CSI_SEND_MAC5 0x00
#endif

static constexpr uint32_t SERIAL_BAUD = 921600;
static constexpr uint32_t SEND_INTERVAL_MS =
  (1000UL / CSI_SEND_FREQUENCY) > 0 ? (1000UL / CSI_SEND_FREQUENCY) : 1UL;
static const uint8_t CSI_SEND_MAC[] = {
  CSI_SEND_MAC0, CSI_SEND_MAC1, CSI_SEND_MAC2, CSI_SEND_MAC3, CSI_SEND_MAC4, CSI_SEND_MAC5
};
static const uint8_t ESPNOW_BROADCAST_MAC[] = {
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

static uint32_t sendCount = 0;
static bool senderReady = false;

void printMac(const uint8_t *mac) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

bool checkEsp(const char *label, esp_err_t error) {
  if (error == ESP_OK) {
    return true;
  }

  Serial.printf("# %s failed: %s (%d)\n", label, esp_err_to_name(error), error);
  return false;
}

bool checkEspAlreadyOk(const char *label, esp_err_t error) {
  if (error == ESP_OK || error == ESP_ERR_INVALID_STATE) {
    return true;
  }

  Serial.printf("# %s failed: %s (%d)\n", label, esp_err_to_name(error), error);
  return false;
}

bool initWifi() {
  if (!checkEspAlreadyOk("esp_netif_init", esp_netif_init())) {
    return false;
  }

  if (!checkEspAlreadyOk("esp_event_loop_create_default", esp_event_loop_create_default())) {
    return false;
  }

  wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
  if (!checkEsp("esp_wifi_init", esp_wifi_init(&config))) {
    return false;
  }

  if (!checkEsp("esp_wifi_set_storage", esp_wifi_set_storage(WIFI_STORAGE_RAM))) {
    return false;
  }

  if (!checkEsp("esp_wifi_set_mode", esp_wifi_set_mode(WIFI_MODE_STA))) {
    return false;
  }

  if (!checkEsp("esp_wifi_set_mac", esp_wifi_set_mac(WIFI_IF_STA, CSI_SEND_MAC))) {
    return false;
  }

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61 || \
    (CONFIG_IDF_TARGET_ESP32C6 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0))
  if (!checkEsp("esp_wifi_set_band_mode", esp_wifi_set_band_mode(WIFI_BAND_MODE_2G_ONLY))) {
    return false;
  }
#endif

#if CONFIG_IDF_TARGET_ESP32C5
  wifi_protocols_t protocols = {
    .ghz_2g = WIFI_PROTOCOL_11N,
    .ghz_5g = WIFI_PROTOCOL_11N,
  };
  if (!checkEsp("esp_wifi_set_protocols", esp_wifi_set_protocols(ESP_IF_WIFI_STA, &protocols))) {
    return false;
  }

  wifi_bandwidths_t bandwidth = {
    .ghz_2g = WIFI_BW_HT40,
    .ghz_5g = WIFI_BW_HT40,
  };
  if (!checkEsp("esp_wifi_set_bandwidths", esp_wifi_set_bandwidths(ESP_IF_WIFI_STA, &bandwidth))) {
    return false;
  }
#elif (CONFIG_IDF_TARGET_ESP32C6 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)) || CONFIG_IDF_TARGET_ESP32C61
  wifi_protocols_t protocols = {
    .ghz_2g = WIFI_PROTOCOL_11N,
  };
  if (!checkEsp("esp_wifi_set_protocols", esp_wifi_set_protocols(ESP_IF_WIFI_STA, &protocols))) {
    return false;
  }

  wifi_bandwidths_t bandwidth = {
    .ghz_2g = WIFI_BW_HT40,
  };
  if (!checkEsp("esp_wifi_set_bandwidths", esp_wifi_set_bandwidths(ESP_IF_WIFI_STA, &bandwidth))) {
    return false;
  }
#else
  if (!checkEsp("esp_wifi_set_bandwidth", esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40))) {
    return false;
  }
#endif

  if (!checkEsp("esp_wifi_start", esp_wifi_start())) {
    return false;
  }

  if (!checkEsp("esp_wifi_set_ps", esp_wifi_set_ps(WIFI_PS_NONE))) {
    return false;
  }

  if (!checkEsp("esp_wifi_set_channel",
                esp_wifi_set_channel(CSI_SEND_CHANNEL, WIFI_SECOND_CHAN_BELOW))) {
    return false;
  }

  return true;
}

bool initEspNow() {
  if (!checkEsp("esp_now_init", esp_now_init())) {
    return false;
  }

  if (!checkEsp("esp_now_set_pmk", esp_now_set_pmk(reinterpret_cast<const uint8_t *>("pmk1234567890123")))) {
    return false;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, ESPNOW_BROADCAST_MAC, sizeof(peer.peer_addr));
  peer.channel = CSI_SEND_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;

  if (!checkEsp("esp_now_add_peer", esp_now_add_peer(&peer))) {
    return false;
  }

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  esp_now_rate_config_t rateConfig = {
    .phymode = WIFI_PHY_MODE_HT40,
    .rate = WIFI_PHY_RATE_MCS0_LGI,
    .ersu = false,
    .dcm = false,
  };
  if (!checkEsp("esp_now_set_peer_rate_config",
                esp_now_set_peer_rate_config(peer.peer_addr, &rateConfig))) {
    return false;
  }
#else
  if (!checkEsp("esp_wifi_config_espnow_rate",
                esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_MCS0_LGI))) {
    return false;
  }
#endif

  return true;
}

void printStartupInfo() {
  Serial.println("# ================ CSI SEND ================");
  Serial.printf("# WiFi channel: %u\n", CSI_SEND_CHANNEL);
  Serial.printf("# Send frequency: %u Hz\n", CSI_SEND_FREQUENCY);
  Serial.print("# Sender MAC: ");
  printMac(CSI_SEND_MAC);
  Serial.println();
  Serial.print("# Destination MAC: ");
  printMac(ESPNOW_BROADCAST_MAC);
  Serial.println();
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(2000);

  printStartupInfo();

  senderReady = initWifi() && initEspNow();
  if (senderReady) {
    Serial.println("# ESP-NOW sender ready");
  } else {
    Serial.println("# ESP-NOW sender failed to start");
  }
}

void loop() {
  if (!senderReady) {
    delay(1000);
    return;
  }

  esp_err_t error = esp_now_send(ESPNOW_BROADCAST_MAC,
                                 reinterpret_cast<const uint8_t *>(&sendCount),
                                 sizeof(sendCount));
  if (error != ESP_OK) {
    Serial.printf("# free_heap: %lu <%s> ESP-NOW send error\n",
                  static_cast<unsigned long>(esp_get_free_heap_size()),
                  esp_err_to_name(error));
  }

  sendCount++;
  delay(SEND_INTERVAL_MS);
}
