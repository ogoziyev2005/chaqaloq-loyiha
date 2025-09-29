#include <Arduino.h>
#include <WiFi.h>
#include <driver/i2s.h>
#include <time.h>
#include "esp_heap_caps.h"
#include <Wire.h>
#include <Adafruit_BME680.h>

// Edge Impulse
#include <chaqaloq_inferencing.h>  // EI exported header

// ===== FIREBASE =====
#include <Firebase_ESP_Client.h>
#include <string>
// Agar mavjud bo'lsa ishlating, bo'lmasa komment qiling:
// #include "addons/TokenHelper.h"
// #include "addons/RTDBHelper.h"

// ---------- USER SETTINGS ----------
#define WIFI_SSID     "U-ENTER Guest"
#define WIFI_PASSWORD "Innovation"

// Firebase creds
  #define API_KEY        "AIzaSyC6KKZDOSxky7Ruq9eGZU2PcXuBI6bBCVA"
  #define DATABASE_URL   "https://noona-c0c30-default-rtdb.europe-west1.firebasedatabase.app"
  #define USER_EMAIL     "esp32@iot.com"
  #define USER_PASSWORD  "esp32test123"

// I2S pins (SPH0645 → ESP32)
#define I2S_WS   15   // LRCLK/WS
#define I2S_SCK  33   // BCLK
#define I2S_SD   13   // DOUT

#define SAMPLE_RATE 16000
#define TARGET_LABEL "cry"

// Trigger siyosati
#define P_THRESH_UP    0.80f
#define P_THRESH_DOWN  0.60f
#define N_CONSEC       2
#define COOLDOWN_MS    12000

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

// I2C (BME680)
#define I2C_SDA 27
#define I2C_SCL 14

// NTP (timestamp uchun)
#define NTP_SERVER "pool.ntp.org"
#define TZ_OFFSET_SEC  (5*3600)    // Asia/Tashkent (UTC+5)
#define DST_OFFSET_SEC 0

// ---------- GLOBALS ----------

// Firebase
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig configFB;
static bool fb_ready = false;
static uint32_t last_fb_ping_ms = 0;

// App state
static uint32_t last_alert_ms = 0;
static bool armed = true;
static int consec_hits = 0;

// I2S align autodetect
static int g_use_right = -1;  // -1 unknown, 0 left, 1 right
static int g_use_shift = -1;  // -1 unknown, 8 yoki 14

// HPF (DC/rumble) — simple 1st-order with coeff ~0.995
static int32_t hpf_y = 0, hpf_x1 = 0;
static inline int32_t hpf24(int32_t x) {
  int32_t yn = (x - hpf_x1) + (int32_t)(0.995f * (float)hpf_y);
  hpf_x1 = x; hpf_y = yn;
  return yn;
}

// EI buffer (dynamic)
static int16_t* audio_buf = nullptr;
static size_t   audio_fill = 0;

// BME680 (I2C)
Adafruit_BME680 bme;      // I2C
static bool bme_ok = false;
static uint32_t last_bme_ms = 0;
static uint32_t last_bme_push_ms = 0;

struct BmeRead {
  bool ok;
  float tempC, hum, pres_hPa;
  float gas_ohm;
};
static BmeRead lastBmeCached{false, 0, 0, 0, 0};

// ---------- Wi-Fi ----------
static void wifiConnectBlocking(uint32_t timeout_ms = 30000) {
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname("baby-cry");
  Serial.printf("[WiFi] Connecting to \"%s\"", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < timeout_ms) {
    Serial.print('.');
    delay(250);
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] Connected. IP=%s RSSI=%d\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.println("[WiFi] FAILED (SSID/parol yoki 2.4GHz sozlamasini tekshiring).");
  }
}

static void wifiEnsureConnected() {
  static uint32_t last = 0;
  if (millis() - last < 5000) return;
  last = millis();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Reconnecting...");
    WiFi.disconnect();
    WiFi.reconnect();
  }
}

// ---------- Firebase helpers ----------
static void firebaseInit() {
  configFB.api_key = API_KEY;
  configFB.database_url = DATABASE_URL;

  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  // Agar addons fayllari bor bo‘lsa, yoqing:
  // configFB.token_status_callback = tokenStatusCallback;

  Firebase.reconnectWiFi(true);
  Firebase.begin(&configFB, &auth);   // void
  fb_ready = true;
  Serial.println("[FB] begin called, waiting for Firebase.ready() ...");
}

static bool fbEnsureReady() {
  if (!fb_ready) return false;
  if (WiFi.status() != WL_CONNECTED) return false;
  return Firebase.ready();
}

static String isoTimestampNow() {
  time_t now = time(nullptr);
  struct tm t;
  gmtime_r(&now, &t); // UTC ISO8601
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &t);
  return String(buf);
}

const char* envStatusAddress = "status/otabek";
const char* cryStatusAddress = "cry/otabek";

static bool reportCryingStatus(const char* label, float prob, const BmeRead* bmeOpt) {
  if (!fbEnsureReady()) return false;

  FirebaseJson json;
  json.set("ts", isoTimestampNow());
  json.set("ts_ms", (int64_t)millis());
  json.set("label", String(label));
  json.set("prob", prob);

  if (!Firebase.RTDB.setJSON(&fbdo, cryStatusAddress, &json)) {
    Serial.printf("[FB] set /env/latest FAIL: %s\n", fbdo.errorReason().c_str());
  }
}

static void fbSetEnvLatest(const BmeRead& r) {
  if (!fbEnsureReady() || !r.ok) return;

  FirebaseJson json;
  json.set("ts", isoTimestampNow());
  json.set("ts_ms", (int64_t)millis());
  json.set("tempC", r.tempC);
  json.set("hum", r.hum);
  json.set("pres_hPa", r.pres_hPa);
  json.set("gas_ohm", r.gas_ohm);

  if (!Firebase.RTDB.setJSON(&fbdo, envStatusAddress, &json)) {
    Serial.printf("[FB] set /env/latest FAIL: %s\n", fbdo.errorReason().c_str());
  }
}

static void fbPushEnvHistory(const BmeRead& r) {
  if (!fbEnsureReady() || !r.ok) return;

  FirebaseJson json;
  json.set("ts", isoTimestampNow());
  json.set("ts_ms", (int64_t)millis());
  json.set("tempC", r.tempC);
  json.set("hum", r.hum);
  json.set("pres_hPa", r.pres_hPa);
  json.set("gas_ohm", r.gas_ohm);

  if (!Firebase.RTDB.setJSON(&fbdo, envStatusAddress, &json)) {
    Serial.printf("[FB] push /env/history FAIL: %s\n", fbdo.errorReason().c_str());
  }
}

// ---------- I2S init ----------
static void i2sMicInit() {
  i2s_config_t cfg; memset(&cfg, 0, sizeof(cfg));
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;   // SPH0645: 24-bit in 32-bit slot
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;   // interleaved R,L
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = false;
  cfg.fixed_mclk = 0;

  i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);

  i2s_pin_config_t pins; memset(&pins, 0, sizeof(pins));
  pins.bck_io_num   = I2S_SCK;
  pins.ws_io_num    = I2S_WS;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num  = I2S_SD;

  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO);

  Serial.println("[I2S] Initialized.");
}

// ---------- I2S → int16 PCM ----------
static void fillAudioForEI() {
  if (audio_fill >= EI_CLASSIFIER_RAW_SAMPLE_COUNT) return;

  int32_t buf[1024];
  size_t br = 0;
  if (i2s_read(I2S_NUM_0, buf, sizeof(buf), &br, 50 / portTICK_PERIOD_MS) != ESP_OK) return;
  int n = br / sizeof(int32_t);

  for (int i = 0; i + 1 < n && audio_fill < EI_CLASSIFIER_RAW_SAMPLE_COUNT; i += 2) {
    int32_t rawR = buf[i + 0];
    int32_t rawL = buf[i + 1];

    int32_t r24a = (rawR >> 8),  l24a = (rawL >> 8);
    int32_t r24b = (rawR >> 14), l24b = (rawL >> 14);

    if (g_use_right < 0 || g_use_shift < 0) {
      const bool nz_r8  = (r24a != 0);
      const bool nz_l8  = (l24a != 0);
      const bool nz_r14 = (r24b != 0);
      const bool nz_l14 = (l24b != 0);
      if (nz_l8 || nz_r8 || nz_l14 || nz_r14) {
        if (nz_l8)      { g_use_right = 0; g_use_shift = 8;  }
        else if (nz_r8) { g_use_right = 1; g_use_shift = 8;  }
        else if (nz_l14){ g_use_right = 0; g_use_shift = 14; }
        else            { g_use_right = 1; g_use_shift = 14; }
        Serial.printf("[I2S] Detected channel=%s shift=%d\n",
                      g_use_right ? "RIGHT" : "LEFT", g_use_shift);
      }
    }

    const int32_t raw = (g_use_right > 0) ? rawR : rawL;
    const int32_t s24 = (g_use_shift == 14) ? (raw >> 14) : (raw >> 8);

    // HPF
    const int32_t s = hpf24(s24);

    // 24-bit -> 16-bit
    const int16_t v = (int16_t)(s >> 8);
    audio_buf[audio_fill++] = v;
  }
}

// ---------- EI callback ----------
static int raw_get_data(size_t offset, size_t length, float *out_ptr) {
  if ((offset + length) > EI_CLASSIFIER_RAW_SAMPLE_COUNT) return EIDSP_OUT_OF_BOUNDS;
  for (size_t i = 0; i < length; i++) {
    out_ptr[i] = (float)audio_buf[offset + i] / 32768.0f;
  }
  return EIDSP_OK;
}

// ---------- BME680 helpers ----------
static BmeRead readBME680_blocking() {
  BmeRead r{false, 0, 0, 0, 0};
  if (!bme.beginReading()) return r;  // schedules conversion
  if (!bme.endReading())   return r;  // ~150ms incl. gas heater
  r.ok       = true;
  r.tempC    = bme.temperature;
  r.hum      = bme.humidity;
  r.pres_hPa = bme.pressure / 100.0f;     // Pa → hPa
  r.gas_ohm  = bme.gas_resistance;        // Ω
  return r;
}

// ---------- Inference + decision ----------
static void runEIAndMaybeAlert() {
  if (audio_fill < EI_CLASSIFIER_RAW_SAMPLE_COUNT) return;

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
  signal.get_data = &raw_get_data;

  ei_impulse_result_t result = { 0 };
  const bool debug = false;

  EI_IMPULSE_ERROR ei_err = run_classifier(&signal, &result, debug);
  if (ei_err != EI_IMPULSE_OK) {
    Serial.printf("[EI] run_classifier error: %d\n", ei_err);
    audio_fill = 0;
    return;
  }

  float top_p = 0.0f;
  const char* top_label = nullptr;

  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    const float p = result.classification[ix].value;
    const char* lab = result.classification[ix].label;
    if (p > top_p) { top_p = p; top_label = lab; }
  }
  Serial.printf("[EI] top=%s p=%.2f armed=%d consec=%d\n",
                top_label ? top_label : "(null)", top_p, (int)armed, consec_hits);

  const bool is_target = (top_label && (String(top_label) == String(TARGET_LABEL)));
  const uint32_t now = millis();

  if (armed) {
    if (is_target && top_p >= P_THRESH_UP) {
      consec_hits++;
      if (consec_hits >= N_CONSEC && (now - last_alert_ms) > COOLDOWN_MS) {
        digitalWrite(LED_BUILTIN, HIGH); delay(60); digitalWrite(LED_BUILTIN, LOW);

        // Firebase'ga alert push
        if (reportCryingStatus(top_label, top_p, lastBmeCached.ok ? &lastBmeCached : nullptr)) {
          last_alert_ms = now;
        }
        armed = false;
        consec_hits = 0;
      }
    } else {
      consec_hits = 0;
    }
  } else {
    if (!is_target || top_p <= P_THRESH_DOWN) {
      armed = true;
    }
  }

  audio_fill = 0; // keyingi oynaga o'tamiz
}

// ---------- Misc ----------
static void printMem() {
  size_t free8 = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  size_t free32 = heap_caps_get_free_size(MALLOC_CAP_32BIT);
  #ifdef MALLOC_CAP_SPIRAM
    size_t freeSP = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  #else
    size_t freeSP = 0;
  #endif
  Serial.printf("[MEM] free8=%u  free32=%u  freeSPIRAM=%u\n",
                (unsigned)free8, (unsigned)free32, (unsigned)freeSP);
}

// ---------- SETUP / LOOP ----------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[BOOT] EI TinyML Baby Cry (ESP32 + SPH0645 + BME680 + Firebase)");

  if ((int)EI_CLASSIFIER_FREQUENCY != SAMPLE_RATE) {
    Serial.printf("[WARN] Model freq=%d Hz, I2S freq=%d Hz — mos emas!\n",
                  (int)EI_CLASSIFIER_FREQUENCY, SAMPLE_RATE);
  }

  // EI buffer malloc
  size_t need_bytes = (size_t)EI_CLASSIFIER_RAW_SAMPLE_COUNT * sizeof(int16_t);
  #ifdef MALLOC_CAP_SPIRAM
    audio_buf = (int16_t*) heap_caps_malloc(need_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  #else
    audio_buf = nullptr;
  #endif
  if (!audio_buf) audio_buf = (int16_t*) malloc(need_bytes);
  if (!audio_buf) {
    Serial.printf("[MEM] audio_buf malloc(%u) FAILED\n", (unsigned)need_bytes);
    while (1) { delay(1000); }
  }

  printMem();

  // Wi-Fi
  wifiConnectBlocking(30000);

  // NTP (timestamplar uchun)
  configTime(TZ_OFFSET_SEC, DST_OFFSET_SEC, NTP_SERVER);

  // Firebase init
  firebaseInit();

  // I2S mic init
  i2sMicInit();

  // I2C start (SDA=27, SCL=14)
  Wire.begin(I2C_SDA, I2C_SCL);

  // BME680 init
  Serial.print("[BME680] Init...");
  bme_ok = bme.begin(0x76) || bme.begin(0x77);
  if (!bme_ok) {
    Serial.println(" FAIL (topilmadi). Ulash va I2C pinlarini tekshiring.");
  } else {
    Serial.println(" OK");
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320°C, 150 ms
  }

  // Online ping (Firebasega status) — tayyor bo'lsa
  if (fbEnsureReady()) {
    //Firebase.RTDB.setString(&fbdo, "/devices/boot_ts", isoTimestampNow());
    //Firebase.RTDB.setString(&fbdo, "/devices/ip", WiFi.localIP().toString());
  }
}

void loop() {
  // Wi-Fi sog'ligini kuzat
  wifiEnsureConnected();

  // Firebase token holati (tayyor bo‘lsa true)
  if (Firebase.ready() && (millis() - last_fb_ping_ms > 15000)) {
    last_fb_ping_ms = millis();
    // jonli ekanimizni bildiruvchi kichik status
    //Firebase.RTDB.setInt(&fbdo, "/devices/uptime_ms", (int)millis());
  }

  // Audio + inference
  fillAudioForEI();
  if (audio_fill >= EI_CLASSIFIER_RAW_SAMPLE_COUNT) {
    runEIAndMaybeAlert();
  }

  // BME680 — har ~5 s latest'ga yozamiz; har ~60 s history’ga push qilamiz
  if (millis() - last_bme_ms > 5000) {
    last_bme_ms = millis();
    if (bme_ok) {
      BmeRead r = readBME680_blocking();
      if (r.ok) {
        lastBmeCached = r;
        Serial.printf("[BME680] T=%.2fC  RH=%.1f%%  P=%.1fhPa  Gas=%.0fΩ\n",
                      r.tempC, r.hum, r.pres_hPa, r.gas_ohm);
        fbSetEnvLatest(r);
      } else {
        Serial.println("[BME680] read FAIL");
      }
    }
  }

  if (millis() - last_bme_push_ms > 60000) {
    last_bme_push_ms = millis();
    if (lastBmeCached.ok) {
      fbPushEnvHistory(lastBmeCached);
    }
  }
}
