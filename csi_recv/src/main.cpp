#include <Arduino.h>
#include <math.h>

extern "C" {
#include "esp_err.h"
#include "esp_event.h"
#include "esp_idf_version.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "mbedtls/base64.h"
#include "rom/ets_sys.h"
}

#ifndef CSI_RECV_CHANNEL
#define CSI_RECV_CHANNEL 11
#endif

#ifndef CSI_RECV_PRINT_STATUS
#define CSI_RECV_PRINT_STATUS 0
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

static constexpr uint32_t SERIAL_BAUD = 2000000;
static constexpr uint32_t STATUS_INTERVAL_MS = 5000;
static constexpr size_t RADAR_MAX_PAIRS = 384;
static constexpr size_t RADAR_HISTORY_LEN = 100;

static const uint8_t CSI_SEND_MAC[] = {
  CSI_SEND_MAC0, CSI_SEND_MAC1, CSI_SEND_MAC2, CSI_SEND_MAC3, CSI_SEND_MAC4, CSI_SEND_MAC5
};
static const uint8_t ESPNOW_BROADCAST_MAC[] = {
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

static volatile uint32_t csiPacketsSeen = 0;
static volatile uint32_t espNowPacketsSeen = 0;
static volatile uint32_t lastEspNowId = 0;
static volatile bool haveEspNowId = false;
static bool receiverReady = false;
static uint32_t lastStatusMs = 0;

static float radarBaseline[RADAR_MAX_PAIRS] = {};
static float radarPrevious[RADAR_MAX_PAIRS] = {};
static float radarWanderHistory[RADAR_HISTORY_LEN] = {};
static float radarJitterHistory[RADAR_HISTORY_LEN] = {};
static bool radarHasBaseline = false;
static size_t radarHistoryCount = 0;
static uint32_t radarSequence = 0;

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
                esp_wifi_set_channel(CSI_RECV_CHANNEL, WIFI_SECOND_CHAN_BELOW))) {
    return false;
  }

  return true;
}

void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int dataLen) {
  if (mac == nullptr || data == nullptr || dataLen < static_cast<int>(sizeof(uint32_t))) {
    return;
  }

  if (memcmp(mac, CSI_SEND_MAC, sizeof(CSI_SEND_MAC)) != 0) {
    return;
  }

  uint32_t id = 0;
  memcpy(&id, data, sizeof(id));
  lastEspNowId = id;
  haveEspNowId = true;
  espNowPacketsSeen++;
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
  peer.channel = CSI_RECV_CHANNEL;
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

  if (!checkEsp("esp_now_register_recv_cb", esp_now_register_recv_cb(onEspNowRecv))) {
    return false;
  }

  return true;
}

float meanHistory(const float *history, size_t count) {
  if (count == 0) {
    return 0.0f;
  }

  float sum = 0.0f;
  for (size_t i = 0; i < count; i++) {
    sum += history[i];
  }
  return sum / count;
}

float medianHistory(const float *history, size_t count) {
  if (count == 0) {
    return 0.0f;
  }

  float sorted[RADAR_HISTORY_LEN] = {};
  for (size_t i = 0; i < count; i++) {
    sorted[i] = history[i];
  }

  for (size_t i = 1; i < count; i++) {
    float value = sorted[i];
    size_t j = i;
    while (j > 0 && sorted[j - 1] > value) {
      sorted[j] = sorted[j - 1];
      j--;
    }
    sorted[j] = value;
  }

  if ((count % 2) == 0) {
    return (sorted[count / 2 - 1] + sorted[count / 2]) * 0.5f;
  }
  return sorted[count / 2];
}

void printRadarData(const wifi_csi_info_t *info) {
  const size_t pairCountFromCsi = info->len / 2;
  const size_t pairCount = pairCountFromCsi < RADAR_MAX_PAIRS ? pairCountFromCsi : RADAR_MAX_PAIRS;
  if (pairCount == 0) {
    return;
  }

  float wanderSum = 0.0f;
  float jitterSum = 0.0f;
  float baselineSum = 0.0f;
  float previousSum = 0.0f;

  for (size_t i = 0; i < pairCount; i++) {
    const float imag = static_cast<float>(info->buf[i * 2]);
    const float real = static_cast<float>(info->buf[i * 2 + 1]);
    const float amplitude = sqrtf(real * real + imag * imag);

    if (!radarHasBaseline) {
      radarBaseline[i] = amplitude;
      radarPrevious[i] = amplitude;
    }

    wanderSum += fabsf(amplitude - radarBaseline[i]);
    jitterSum += fabsf(amplitude - radarPrevious[i]);
    baselineSum += radarBaseline[i];
    previousSum += radarPrevious[i];

    radarBaseline[i] = radarBaseline[i] * 0.995f + amplitude * 0.005f;
    radarPrevious[i] = amplitude;
  }

  radarHasBaseline = true;

  const float wander = wanderSum / (baselineSum + pairCount);
  const float jitter = jitterSum / (previousSum + pairCount);
  const size_t historyIndex = radarSequence % RADAR_HISTORY_LEN;
  radarWanderHistory[historyIndex] = wander;
  radarJitterHistory[historyIndex] = jitter;
  if (radarHistoryCount < RADAR_HISTORY_LEN) {
    radarHistoryCount++;
  }

  const float wanderAverage = meanHistory(radarWanderHistory, radarHistoryCount);
  const float jitterMedian = medianHistory(radarJitterHistory, radarHistoryCount);
  const float someoneThreshold = 0.020f;
  const float moveThreshold = 0.015f;
  const int someoneStatus = wanderAverage > someoneThreshold ? 1 : 0;
  const int moveStatus = jitter > moveThreshold ? 1 : 0;

  if (radarSequence == 0) {
    ets_printf("# ================ RADAR RECV ================\n");
    ets_printf("type,sequence,timestamp,waveform_wander,wander_average,waveform_wander_threshold,someone_status,waveform_jitter,jitter_midean,waveform_jitter_threshold,move_status\n");
  }

  ets_printf("RADAR_DADA,%lu,%lu,%.6f,%.6f,%.6f,%d,%.6f,%.6f,%.6f,%d\n",
             static_cast<unsigned long>(radarSequence++),
             static_cast<unsigned long>(millis()),
             wander,
             wanderAverage,
             someoneThreshold,
             someoneStatus,
             jitter,
             jitterMedian,
             moveThreshold,
             moveStatus);
}

void onCsiData(void *, wifi_csi_info_t *info) {
  if (info == nullptr || info->buf == nullptr || info->len == 0) {
    ets_printf("# wifi_csi_cb invalid argument\n");
    return;
  }

  if (memcmp(info->mac, CSI_SEND_MAC, sizeof(CSI_SEND_MAC)) != 0) {
    return;
  }

  const wifi_pkt_rx_ctrl_t *rxCtrl = &info->rx_ctrl;
  const uint32_t packetCount = ++csiPacketsSeen;

  if (packetCount == 1) {
    ets_printf("# ================ CSI RECV ================\n");
    ets_printf("type,sequence,timestamp,taget_seq,target,mac,rssi,rate,sig_mode,mcs,bandwidth,smoothing,not_sounding,aggregation,stbc,fec_coding,sgi,noise_floor,ampdu_cnt,channel,secondary_channel,local_timestamp,ant,sig_len,rx_state,agc_gain,fft_gain,len,first_word,data\n");
  }

  char base64Data[1024] = {};
  size_t base64Len = 0;
  int encodeResult = mbedtls_base64_encode(
    reinterpret_cast<unsigned char *>(base64Data),
    sizeof(base64Data) - 1,
    &base64Len,
    reinterpret_cast<const unsigned char *>(info->buf),
    info->len);
  if (encodeResult != 0) {
    return;
  }
  base64Data[base64Len] = '\0';

  ets_printf("CSI_DATA,%u,%lu,0,unknown," MACSTR ",%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%u,%u,%u,%u,%u,%u,%u,0,0,%u,%u,%s\n",
             packetCount,
             static_cast<unsigned long>(millis()),
             MAC2STR(info->mac),
             rxCtrl->rssi,
             rxCtrl->rate,
             rxCtrl->sig_mode,
             rxCtrl->mcs,
             rxCtrl->cwb,
             rxCtrl->smoothing,
             rxCtrl->not_sounding,
             rxCtrl->aggregation,
             rxCtrl->stbc,
             rxCtrl->fec_coding,
             rxCtrl->sgi,
             rxCtrl->noise_floor,
             rxCtrl->ampdu_cnt,
             rxCtrl->channel,
             rxCtrl->secondary_channel,
             rxCtrl->timestamp,
             rxCtrl->ant,
             rxCtrl->sig_len,
             rxCtrl->rx_state,
             info->len,
             info->first_word_invalid ? 1 : 0,
             base64Data);

  printRadarData(info);
}

bool initCsi() {
  if (!checkEsp("esp_wifi_set_promiscuous", esp_wifi_set_promiscuous(true))) {
    return false;
  }

  wifi_csi_config_t csiConfig = {};
  csiConfig.lltf_en = true;
  csiConfig.htltf_en = true;
  csiConfig.stbc_htltf2_en = true;
  csiConfig.ltf_merge_en = true;
  csiConfig.channel_filter_en = true;
  csiConfig.manu_scale = false;
  csiConfig.shift = 0;

  if (!checkEsp("esp_wifi_set_csi_config", esp_wifi_set_csi_config(&csiConfig))) {
    return false;
  }

  if (!checkEsp("esp_wifi_set_csi_rx_cb", esp_wifi_set_csi_rx_cb(onCsiData, nullptr))) {
    return false;
  }

  if (!checkEsp("esp_wifi_set_csi", esp_wifi_set_csi(true))) {
    return false;
  }

  return true;
}

void printStartupInfo() {
  Serial.println("# ================ CSI RECV ================");
  Serial.printf("# WiFi channel: %u\n", CSI_RECV_CHANNEL);
  Serial.print("# Expected sender MAC: ");
  printMac(CSI_SEND_MAC);
  Serial.println();
  Serial.print("# ESP-NOW peer MAC: ");
  printMac(ESPNOW_BROADCAST_MAC);
  Serial.println();
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(2000);

  printStartupInfo();

  receiverReady = initWifi() && initEspNow() && initCsi();
  if (receiverReady) {
    Serial.println("# CSI receiver ready");
  } else {
    Serial.println("# CSI receiver failed to start");
  }
}

void loop() {
#if CSI_RECV_PRINT_STATUS
  if (millis() - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = millis();
    Serial.printf("# status ready=%d espnow_seen=%lu csi_seen=%lu last_id=%lu free_heap=%lu\n",
                  receiverReady ? 1 : 0,
                  static_cast<unsigned long>(espNowPacketsSeen),
                  static_cast<unsigned long>(csiPacketsSeen),
                  static_cast<unsigned long>(lastEspNowId),
                  static_cast<unsigned long>(esp_get_free_heap_size()));
  }
#endif

  delay(20);
}
