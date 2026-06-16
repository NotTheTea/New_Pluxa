#include <Arduino.h>
#include <GTimer.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <Wire.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <secrets.h>

// ─── Pin definitions ────────────────────────────────────────────────────────
#define PIN_WARMER        23
#define PIN_LED            2
#define PIN_ONE_WIRE      21

// ─── Thermostat constants ────────────────────────────────────────────────────
#define TEMP_SETPOINT_MIN   17.0f   // lowest allowed setpoint (°C)
#define TEMP_SETPOINT_MAX   21.0f   // highest allowed setpoint (°C)
#define TEMP_HYSTERESIS      1.0f   // heater on/off band (°C)
#define TEMP_INVALID       -100.0f  // threshold below which a reading is bad
#define TEMP_INVALID_POWERUP 85.0f   // threshold for invalid reading after power-up or bad connection (°C) – DS18B20 returns 85°C on power-up

// ─── Timing constants (ms) ──────────────────────────────────────────────────
#define SENSOR_REQUEST_INTERVAL_MS    1000
#define WIFI_TASK_INTERVAL_MS        40000
#define WIFI_RETRY_INTERVAL_MS       30000

// ─── Heater-check interval default (minutes) ────────────────────────────────
#define HEATER_CHECK_INTERVAL_DEFAULT_MIN  2.0f

// ─── EMA filter coefficient ─────────────────────────────────────────────────
static const float ALPHA = 0.3f;

// ─── One-Wire / sensor setup ─────────────────────────────────────────────────
OneWire oneWire(PIN_ONE_WIRE);
DallasTemperature sensors(&oneWire);

DeviceAddress term_1 = { 0x28, 0x0C, 0x6D, 0x6A, 0x00, 0x00, 0x00, 0x15 };
DeviceAddress term_2 = { 0x28, 0x82, 0x75, 0x6B, 0x00, 0x00, 0x00, 0x21 };
DeviceAddress term_3 = { 0x28, 0x39, 0x53, 0x6B, 0x00, 0x00, 0x00, 0xBA };

// ─── Shared temperature state ────────────────────────────────────────────────
// Protected by dataMutex; only update/read inside a taken mutex.
static float dataT1 = 0.0f;
static float dataT2 = 0.0f;
static float dataT3 = 0.0f;
static bool  criticalError = false;

static bool  sensorReady = false;

SemaphoreHandle_t dataMutex;   // guards dataT1/T2/T3, criticalError, sensorReady
SemaphoreHandle_t prefsMutex;  // guards all Preferences (NVS) access

// ─── Timer ──────────────────────────────────────────────────────────────────
GTimer<millis> tmrHeater;

// ─── Task handles ───────────────────────────────────────────────────────────
TaskHandle_t hSensorTask;
TaskHandle_t hWifiTask;

// ─── Forward declarations ────────────────────────────────────────────────────
void sensorTask(void *parameter);
void wifiTask(void *parameter);
void runHeater();
void sendPing();
void sendTemps(float tempMain, float tempInside, float tempOutside);
void fetchConfig();

// ────────────────────────────────────────────────────────────────────────────
void setup()
{
  Serial.begin(115200);

  dataMutex  = xSemaphoreCreateMutex();
  prefsMutex = xSemaphoreCreateMutex();

  float boarderTimeMinutes = HEATER_CHECK_INTERVAL_DEFAULT_MIN;


  // ── Warm up sensors ──────────────────────────────────────────────────────
  sensors.begin();
  sensors.setResolution(12);
  sensors.requestTemperatures();
  delay(SENSOR_REQUEST_INTERVAL_MS);

  float t1 = sensors.getTempC(term_1);
  float t2 = sensors.getTempC(term_2);
  float t3 = sensors.getTempC(term_3);

  //readings with validation 
  dataT1 = (t1 > TEMP_INVALID && t1 < TEMP_INVALID_POWERUP) ? t1 : 20.0f;
  dataT2 = (t2 > TEMP_INVALID && t2 < TEMP_INVALID_POWERUP) ? t2 : 20.0f;
  dataT3 = (t3 > TEMP_INVALID && t3 < TEMP_INVALID_POWERUP) ? t3 : 20.0f;

  // ── Pins ─────────────────────────────────────────────────────────────────
  pinMode(PIN_WARMER, OUTPUT);
  pinMode(PIN_LED,    OUTPUT);

  // ── WiFi ─────────────────────────────────────────────────────────────────
  WiFi.begin(ssid, password);
  WiFi.setAutoReconnect(true);
  delay(5000);

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(PIN_LED, HIGH);
    Serial.println("WiFi connected");
    delay(4000);
    digitalWrite(PIN_LED, LOW);
  } else {
    Serial.println("WiFi connection failed – will retry automatically");
  }

  // ── Heater-check timer ───────────────────────────────────────────────────
  tmrHeater.setMode(GTMode::Interval);
  tmrHeater.setTime((unsigned long)(boarderTimeMinutes * 60000.0f));
  tmrHeater.start();

  // ── Spawn tasks ──────────────────────────────────────────────────────────
  // sensorTask: core 0, stack 4 KB  – only does DS18B20 I/O and math
  // wifiTask  : core 0, stack 20 KB – HTTPClient + JSON need the headroom
  // loop()    : core 1 (Arduino default) – lightweight timer-driven heater logic
  xTaskCreatePinnedToCore(sensorTask, "sensorTask", 4096,  NULL, 5, &hSensorTask, 0);
  xTaskCreatePinnedToCore(wifiTask,   "wifiTask",   20480, NULL, 4, &hWifiTask,   0);

  Serial.print("setup() running on core ");
  Serial.println(xPortGetCoreID());
}

// ────────────────────────────────────────────────────────────────────────────
void loop()
{
  if (tmrHeater) {
    bool error;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      error = criticalError;
      xSemaphoreGive(dataMutex);
    } else {
      // Could not acquire mutex – skip this cycle rather than risk bad data
      return;
    }

    if (!error) {
      runHeater();
    } else {
      Serial.println("Critical error: sensor 1 is not responding – heater disabled");
      digitalWrite(PIN_WARMER, LOW);
      digitalWrite(PIN_LED,    LOW);
    }
  }
}

// ────────────────────────────────────────────────────────────────────────────
void runHeater()
{
  // Read the setpoint from NVS (outside any critical section)
  float setpoint = 20.0f;
  if (xSemaphoreTake(prefsMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    Preferences prefs;
    prefs.begin("storage", /*readOnly=*/true);
    setpoint = prefs.getFloat("setpoint", 20.0f);
    prefs.end();
    xSemaphoreGive(prefsMutex);
  }

  // Read the current temperature
  float t1;
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    t1 = dataT1;
    xSemaphoreGive(dataMutex);
  } else {
    return; // skip cycle on mutex failure
  }

  // Hysteresis control: turn ON below (setpoint - hysteresis), OFF above setpoint
  if (t1 < setpoint - TEMP_HYSTERESIS) {
    digitalWrite(PIN_WARMER, HIGH);
    digitalWrite(PIN_LED,    HIGH);
  } else if (t1 > setpoint) {
    digitalWrite(PIN_WARMER, LOW);
    digitalWrite(PIN_LED,    LOW);
  }
  // Between the two thresholds: keep current state (hysteresis dead-band)
}

// ────────────────────────────────────────────────────────────────────────────
void sendPing()
{
  HTTPClient http;
  http.setTimeout(10000);
  http.begin(String(SERVER) + "/ping");
  http.addHeader("X-API-Key",     ESP_KEY);
  http.addHeader("Content-Type",  "application/x-www-form-urlencoded");
  int code = http.POST("");
  if (code != 200) {
    Serial.println("sendPing failed, HTTP code: " + String(code));
  }
  http.end();
}

// ────────────────────────────────────────────────────────────────────────────
void fetchConfig()
{
  HTTPClient http;
  http.setTimeout(10000);
  http.begin(String(SERVER) + "/config");
  http.addHeader("X-API-Key", ESP_KEY);
  int code = http.GET();

  if (code == 200) {
    String payload = http.getString();

    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, payload);
    if (err) {
      Serial.println("fetchConfig: JSON parse error: " + String(err.c_str()));
      http.end();
      return;
    }

    float occupied = doc["occupied"] | -1.0f;  // default -1 flags a missing key
    if (occupied >= TEMP_SETPOINT_MIN && occupied <= TEMP_SETPOINT_MAX) {
      if (xSemaphoreTake(prefsMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        Preferences prefs;
        prefs.begin("storage", /*readOnly=*/false);
        if (prefs.getFloat("setpoint", 20.0f) != occupied) {
          prefs.putFloat("setpoint", occupied);
        }
        prefs.end();
        xSemaphoreGive(prefsMutex);
      }
    }
  } else {
    Serial.println("fetchConfig failed, HTTP code: " + String(code));
  }
  http.end();
}

// ────────────────────────────────────────────────────────────────────────────
void sendTemps(float tempMain, float tempInside, float tempOutside)
{
  HTTPClient http;
  http.setTimeout(10000);
  http.begin(String(SERVER) + "/temps");
  http.addHeader("X-API-Key",    ESP_KEY);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  String body = "temp_main="    + String(tempMain,    2)
              + "&temp_inside="  + String(tempInside,  2)
              + "&temp_outside=" + String(tempOutside, 2);

  int code = http.POST(body);
  if (code != 200) {
    Serial.println("sendTemps failed, HTTP code: " + String(code));
  }
  http.end();
}

// ────────────────────────────────────────────────────────────────────────────
// sensorTask – runs on core 0
// Requests temperatures every second and updates the shared EMA-filtered values.
// ────────────────────────────────────────────────────────────────────────────
void sensorTask(void *parameter)
{
  for (;;) {
    sensors.requestTemperatures();
    vTaskDelay(pdMS_TO_TICKS(SENSOR_REQUEST_INTERVAL_MS));

    float t1 = sensors.getTempC(term_1);
    float t2 = sensors.getTempC(term_2);
    float t3 = sensors.getTempC(term_3);

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (t1 >= TEMP_INVALID_POWERUP || t1 <= TEMP_INVALID) {
        criticalError = true;
      } else {
        criticalError = false;
        dataT1 = dataT1 * (1.0f - ALPHA) + t1 * ALPHA;
      }

      if (t2 > TEMP_INVALID) dataT2 = dataT2 * (1.0f - ALPHA) + t2 * ALPHA;
      if (t3 > TEMP_INVALID) dataT3 = dataT3 * (1.0f - ALPHA) + t3 * ALPHA;

      sensorReady = true;

      xSemaphoreGive(dataMutex);
    }
  }
}

// ────────────────────────────────────────────────────────────────────────────
// wifiTask – runs on core 0
// Sends telemetry and fetches config every 40 s when WiFi is up.
// ────────────────────────────────────────────────────────────────────────────
void wifiTask(void *parameter)
{
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      float t1, t2, t3;
      bool ready;
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        t1    = dataT1;
        t2    = dataT2;
        t3    = dataT3;
        ready = sensorReady;  // FIX #5: read the ready flag under the same lock
        xSemaphoreGive(dataMutex);
      } else {
        vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_INTERVAL_MS));
        continue;
      }

      if (!ready) {
        vTaskDelay(pdMS_TO_TICKS(SENSOR_REQUEST_INTERVAL_MS * 2));
        continue;
      }

      sendTemps(t1, t2, t3);
      sendPing();
      fetchConfig();
      vTaskDelay(pdMS_TO_TICKS(WIFI_TASK_INTERVAL_MS));
    } else {
      vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_INTERVAL_MS));
    }
  }
}