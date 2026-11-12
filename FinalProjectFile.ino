#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>

// Preprocessor check for ESP32 Core v2.x vs v3.x WPA2 Enterprise headers
#if __has_include("esp_eap_client.h")
  #include "esp_eap_client.h"
#else
  #include "esp_wpa2.h"
#endif

// ============================================================================
// STRUCT DEFINITIONS & FORWARD PROTOTYPES
// ============================================================================
struct BMP280Data {
  float tempC;
  float tempF;
  float pressurePa;
  float altitudeM;
};

struct CapData {
  float td_us;
  float total_C_pF;
  bool  pressed;
};

struct UltrasonicData {
  long  duration_us;
  float distance_cm;
};

struct GoBeyondResult {
  float meqi_score;
  float sub_light;
  float sub_thermal;
  float sub_weather;
  float user_multiplier;
};

BMP280Data readBMP280();
CapData readCapacitive();
UltrasonicData readUltrasonic();
GoBeyondResult calculateMEQI(float light_pct, float therm_F, float bmp_F, float pressure_Pa, float dist_cm, bool cap_pressed);

// ============================================================================
// NETWORK & MQTT CONFIGURATION
// ============================================================================
const char* ssid     = "PAWS-Secure";
const char* username = "ajs03011";
const char* password = "81-C0rvette&85";

const char* mqtt_server = "eduoracle.ugavel.com";
const int   mqtt_port   = 1883;
const char* mqtt_user   = "giiuser";
const char* mqtt_pass   = "giipassword";
const char* MQTT_TOPIC  = "elee4230sp26/ajs03011/system_telemetry";

// ============================================================================
// HARDWARE PIN DEFINITIONS
// ============================================================================
const int ADC_PIN          = 34; // Shared MUX ADC input
const int MUX_CTRL_PIN     = 25; // MUX gate control line
const int I2C_SDA          = 21; // BMP280 SDA
const int I2C_SCL          = 22; // BMP280 SCL
const int TRIG_PIN         = 19; // Ultrasonic Trigger
const int ECHO_PIN         = 18; // Ultrasonic Echo
const int DRIVE_PIN        = 16; // Capacitive Drive Pin
const int COMP_PIN         = 17; // Capacitive Comparator Input
const int LED_OPEN_PIN     = 26; // Status LED: Green (Open Indicator)
const int LED_CLOSE_PIN    = 27; // Status LED: Red (Closed Indicator)

// ============================================================================
// HARDWARE PARAMETERS & ADJUSTED CALIBRATION THRESHOLDS
// ============================================================================
const float V_ADC_MAX          = 3.30;
const float VCC                = 3.30;
const float THERM_MUX_OFFSET_V = 0.12; 

// LDR Calibration & Adjusted Day/Night Thresholds
const float LDR_INTERCEPT      = 1.8000;
const float LDR_SLOPE          = 0.0130;
const float DAY_THRESHOLD      = 60.0; // Ambient room light (~70%) exceeds 60%
const float NIGHT_THRESHOLD    = 45.0; // Covered LDR (~40%) falls below 45%

// Thermistor Circuit Constants
const float VREF               = 1.65;
const float GAIN               = 3.50;
const float R1                 = 393.5;
const float R2                 = 392.7;
const float R3                 = 393.3;
const float RP                 = 680.4;
const float TC_K               = 26.9 + 273.15;
const float R_TC               = 965.0;
const float B_PARAM            = 3636.0;
const float CALIBRATION_OFFSET_F = 0.0;

// Capacitive Sensor Circuit Constants
const float R_OHMS             = 3300000.0;
const float LN2                = 0.69314718;
const float SCALE_FACTOR       = (R_OHMS * LN2) / 1000000.0; 
const unsigned long HALF_PERIOD_US = 5000;
float td_baseline              = 0.0;
float T_THRESHOLD              = 0.0;
const float THRESHOLD_OFFSET_US = 30.0;

// Ultrasonic Motion Threshold
const float HAND_MOTION_THRESHOLD_CM = 15.0; 

// ============================================================================
// KALMAN FILTER CLASS (1-D Dynamic Response)
// ============================================================================
class SimpleKalmanFilter {
private:
  float _err_measure;
  float _err_estimate;
  float _q;
  float _current_estimate;
  float _last_estimate;
  float _kalman_gain;
  bool  _initialized;

public:
  SimpleKalmanFilter(float mea_e, float est_e, float q) {
    _err_measure  = mea_e;
    _err_estimate = est_e;
    _q            = q;
    _initialized  = false;
  }

  float updateEstimate(float mea) {
    if (!_initialized) {
      _last_estimate = mea;
      _current_estimate = mea;
      _initialized = true;
      return mea;
    }
    _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
    _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
    _err_estimate = (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
    _last_estimate = _current_estimate;
    return _current_estimate;
  }
};

SimpleKalmanFilter thermKalman(1.0, 1.0, 0.20);

// ============================================================================
// GLOBAL OBJECTS & STATE VARIABLES
// ============================================================================
Adafruit_BMP280 bmp;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

bool curtain_open           = false; 
bool manual_override_active = false;
unsigned long tap_timestamps[10];
int tap_count               = 0;
bool last_cap_state         = false;
unsigned long last_tap_time = 0;
unsigned long last_sensor_time = 0;

GoBeyondResult current_meqi;

// ============================================================================
// HELPER & SENSOR SUB-FUNCTIONS
// ============================================================================
void selectMuxChannel(bool select_ldr) {
  pinMode(MUX_CTRL_PIN, OUTPUT);
  digitalWrite(MUX_CTRL_PIN, select_ldr ? LOW : HIGH);
  delay(25); 
}

float readAveragedADC(int samples = 32) {
  analogRead(ADC_PIN);
  delay(2);
  analogRead(ADC_PIN);
  delay(2);

  long raw_sum = 0;
  for (int i = 0; i < samples; i++) {
    raw_sum += analogRead(ADC_PIN);
    delay(2);
  }
  return (float)raw_sum / (float)samples;
}

float readLDR() {
  selectMuxChannel(true);
  float ldr_raw_avg   = readAveragedADC(32);
  float ldr_v_adc     = (ldr_raw_avg / 4095.0) * V_ADC_MAX;
  float ldr_light_pct = (ldr_v_adc - LDR_INTERCEPT) / LDR_SLOPE;
  if (ldr_light_pct < 0.0)   ldr_light_pct = 0.0;
  if (ldr_light_pct > 100.0) ldr_light_pct = 100.0;
  return ldr_light_pct;
}

float readThermistor() {
  selectMuxChannel(false);
  float therm_raw_avg    = readAveragedADC(32);
  float therm_v_measured = (therm_raw_avg / 4095.0) * V_ADC_MAX;
  float therm_v_adc      = therm_v_measured + THERM_MUX_OFFSET_V;

  float V_diff  = (therm_v_adc - VREF) / GAIN;
  float V_plus  = VCC * (R3 / (R1 + R3));
  float V_minus = V_plus - V_diff;
  float R_eq    = (V_minus * R2) / (VCC - V_minus);
  float R_T     = (RP * R_eq) / (RP - R_eq);

  float inv_T                = (1.0 / TC_K) + (1.0 / B_PARAM) * log(R_T / R_TC);
  float thermistor_tempK     = 1.0 / inv_T;
  float thermistor_tempC     = thermistor_tempK - 273.15;
  float thermistor_raw_tempF = (thermistor_tempC * 9.0 / 5.0) + 32.0;
  return thermistor_raw_tempF + CALIBRATION_OFFSET_F;
}

BMP280Data readBMP280() {
  BMP280Data data;
  data.tempC      = bmp.readTemperature();
  data.tempF      = (data.tempC * 9.0 / 5.0) + 32.0;
  data.pressurePa = bmp.readPressure();
  data.altitudeM  = bmp.readAltitude(1015.3);
  return data;
}

UltrasonicData readUltrasonic() {
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  UltrasonicData uData;
  uData.duration_us = pulseIn(ECHO_PIN, HIGH, 30000); 
  if (uData.duration_us == 0) {
    uData.distance_cm = 400.0; 
  } else {
    uData.distance_cm = (float)uData.duration_us * 0.034 / 2.0;
  }
  return uData;
}

CapData readCapacitive() {
  digitalWrite(DRIVE_PIN, LOW);
  delayMicroseconds(HALF_PERIOD_US);

  unsigned long start_time = micros();
  digitalWrite(DRIVE_PIN, HIGH);

  while (digitalRead(COMP_PIN) == LOW) {
    if ((micros() - start_time) > 5000) break;
  }
  unsigned long end_time = micros();

  CapData data;
  data.td_us      = (float)(end_time - start_time);
  data.total_C_pF = data.td_us / SCALE_FACTOR;
  data.pressed    = (data.td_us > T_THRESHOLD);
  return data;
}

void calibrateCapacitive() {
  digitalWrite(DRIVE_PIN, LOW);
  delay(200);
  float baseline_sum = 0.0;
  int samples = 10;

  for (int i = 0; i < samples; i++) {
    digitalWrite(DRIVE_PIN, LOW);
    delayMicroseconds(HALF_PERIOD_US);

    unsigned long start_time = micros();
    digitalWrite(DRIVE_PIN, HIGH);

    while (digitalRead(COMP_PIN) == LOW) {
      if ((micros() - start_time) > 5000) break;
    }
    baseline_sum += (float)(micros() - start_time);
    delay(20);
  }
  td_baseline = baseline_sum / (float)samples;
  T_THRESHOLD = td_baseline + THRESHOLD_OFFSET_US;
}

bool checkMultiTapOverride(bool cap_pressed) {
  unsigned long now = millis();
  
  if (cap_pressed && !last_cap_state && (now - last_tap_time > 200)) {
    last_tap_time = now;
    if (tap_count < 10) {
      tap_timestamps[tap_count++] = now;
      Serial.print("  [GESTURE TAP ");
      Serial.print(tap_count);
      Serial.println("/4 REGISTERED]");
    }
  }
  last_cap_state = cap_pressed;

  int valid_index = 0;
  for (int i = 0; i < tap_count; i++) {
    if (now - tap_timestamps[i] <= 5000) {
      tap_timestamps[valid_index++] = tap_timestamps[i];
    }
  }
  tap_count = valid_index;

  if (tap_count >= 4) {
    tap_count = 0; 
    return true;   
  }
  return false;
}

// ============================================================================
// "GO BEYOND" ALGORITHM: MEQI COMPUTATION
// ============================================================================
GoBeyondResult calculateMEQI(float light_pct, float therm_F, float bmp_F, float pressure_Pa, float dist_cm, bool cap_pressed) {
  GoBeyondResult res;

  res.sub_light = light_pct; 

  float temp_avg = (therm_F + bmp_F) / 2.0;
  res.sub_thermal = 100.0 - (10.0 * fabs(temp_avg - 72.0));
  if (res.sub_thermal < 0.0)   res.sub_thermal = 0.0;
  if (res.sub_thermal > 100.0) res.sub_thermal = 100.0;

  res.sub_weather = ((pressure_Pa - 98000.0) / 4000.0) * 100.0;
  if (res.sub_weather < 0.0)   res.sub_weather = 0.0;
  if (res.sub_weather > 100.0) res.sub_weather = 100.0;

  bool user_active = (dist_cm < HAND_MOTION_THRESHOLD_CM) || cap_pressed;
  res.user_multiplier = user_active ? 1.25 : 0.80;

  float raw_score = (0.50 * res.sub_light) + (0.30 * res.sub_thermal) + (0.20 * res.sub_weather);
  res.meqi_score  = raw_score * res.user_multiplier;
  if (res.meqi_score > 100.0) res.meqi_score = 100.0;

  return res;
}

// ============================================================================
// LED DISPLAY ENGINE
// ============================================================================
void updateLEDDisplay(unsigned long now_ms) {
  int pulse_brightness = (int)(137.5 + 117.5 * sin((double)now_ms * 0.0041888));

  if (curtain_open) {
    analogWrite(LED_CLOSE_PIN, 0); 
    if (!manual_override_active && current_meqi.meqi_score > 70.0) {
      analogWrite(LED_OPEN_PIN, pulse_brightness); 
    } else {
      analogWrite(LED_OPEN_PIN, 255);              
    }
  } else {
    analogWrite(LED_OPEN_PIN, 0); 
    if (!manual_override_active && current_meqi.meqi_score < 45.0) {
      analogWrite(LED_CLOSE_PIN, pulse_brightness); 
    } else {
      analogWrite(LED_CLOSE_PIN, 255);              
    }
  }
}

// ============================================================================
// NETWORK CONNECTION MANAGEMENT
// ============================================================================
void connectToWifi() {
  delay(10);
  Serial.print("Connecting to WPA2 Enterprise Wi-Fi: ");
  Serial.println(ssid);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  
#if __has_include("esp_eap_client.h")
  esp_eap_client_set_identity((uint8_t *)username, strlen(username));
  esp_eap_client_set_username((uint8_t *)username, strlen(username));
  esp_eap_client_set_password((uint8_t *)password, strlen(password));
  esp_wifi_sta_enterprise_enable();
#else
  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)username, strlen(username));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)username, strlen(username));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)password, strlen(password));
  esp_wifi_sta_wpa2_ent_enable();
#endif

  WiFi.begin(ssid);
  int timeoutCounter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    timeoutCounter++;
    if (timeoutCounter > 40) {
      Serial.println("\nConnection timed out. Retrying...");
      WiFi.begin(ssid);
      timeoutCounter = 0;
    }
  }
  Serial.println("\nWi-Fi Connected!");
}

void connectToMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT broker...");
    String clientId = "ESP32-CurtainControl-" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println(" Connected to MQTT!");
    } else {
      Serial.print(" Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Retrying in 2 seconds...");
      delay(2000);
    }
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(100);

  pinMode(MUX_CTRL_PIN, OUTPUT);
  digitalWrite(MUX_CTRL_PIN, LOW);
  analogSetPinAttenuation(ADC_PIN, ADC_11db);
  pinMode(ADC_PIN, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  pinMode(DRIVE_PIN, OUTPUT);
  pinMode(COMP_PIN, INPUT);

  pinMode(LED_OPEN_PIN, OUTPUT);
  pinMode(LED_CLOSE_PIN, OUTPUT);

  Wire.begin(I2C_SDA, I2C_SCL);
  if (!bmp.begin(0x77)) {
    Serial.println("BMP280 init failed at 0x77! Checking 0x76...");
    if (!bmp.begin(0x76)) {
      Serial.println("BMP280 not found! Re-check I2C wiring.");
    }
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  calibrateCapacitive();

  connectToWifi();
  mqttClient.setServer(mqtt_server, mqtt_port);
  
  Serial.println("==========================================================");
  Serial.println(" Multi-Sensor Curtain Control System Initialized");
  Serial.println("==========================================================");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  if (WiFi.status() != WL_CONNECTED) connectToWifi();
  if (!mqttClient.connected())       connectToMQTT();
  mqttClient.loop();

  unsigned long now_ms = millis();

  // 1. FAST POLL: Capacitive touch & 4-tap gesture (runs every ~20ms)
  CapData capData = readCapacitive();
  bool override_triggered = checkMultiTapOverride(capData.pressed);

  if (override_triggered) {
    curtain_open = !curtain_open;          
    manual_override_active = true;         
    Serial.print("\n>>>> MANUAL 4-TAP GESTURE OVERRIDE TRIGGERED: CURTAIN ");
    Serial.print(curtain_open ? "OPEN" : "CLOSED");
    Serial.println(" <<<<\n");
  }

  // 2. TIMED POLL: Read all sensors, compute MEQI & publish IoT telemetry (every 500ms)
  if (now_ms - last_sensor_time >= 500) {
    last_sensor_time = now_ms;

    float light_pct              = readLDR();
    float therm_raw_F            = readThermistor();
    float therm_kalman_F         = thermKalman.updateEstimate(therm_raw_F);
    BMP280Data bmpData           = readBMP280();
    UltrasonicData ultraData     = readUltrasonic();

    float temp_avg               = (therm_kalman_F + bmpData.tempF) / 2.0;
    float temp_diff_pct          = (fabs(therm_kalman_F - bmpData.tempF) / temp_avg) * 100.0;

    current_meqi = calculateMEQI(light_pct, therm_kalman_F, bmpData.tempF, bmpData.pressurePa, ultraData.distance_cm, capData.pressed);

    // 3. AUTOMATION LOGIC STATE MACHINE
    bool motion_pickup_detected = (ultraData.distance_cm < HAND_MOTION_THRESHOLD_CM) || capData.pressed;

    if (light_pct <= NIGHT_THRESHOLD) {
      curtain_open = false;
      manual_override_active = false;
    } else if (light_pct >= DAY_THRESHOLD && motion_pickup_detected) {
      if (!manual_override_active) {
        curtain_open = true;
      }
    }

    // 4. SERIAL TELEMETRY DISPLAY (Every Sensor Output Visible)
    Serial.print("LDR: ");          Serial.print(light_pct, 1);       Serial.print("% | ");
    Serial.print("Therm(Raw): ");   Serial.print(therm_raw_F, 2);     Serial.print("°F | ");
    Serial.print("Therm(Kalman): ");Serial.print(therm_kalman_F, 2);  Serial.print("°F | ");
    Serial.print("BMP: ");          Serial.print(bmpData.tempF, 2);   Serial.print("°F | ");
    Serial.print("ΔT: ");           Serial.print(temp_diff_pct, 2);   Serial.print("% || ");
    Serial.print("Dist: ");         Serial.print(ultraData.distance_cm, 1); Serial.print("cm | ");
    Serial.print("Cap: ");          Serial.print(capData.total_C_pF, 1);    Serial.print("pF (");
    Serial.print(capData.pressed ? "PRESS DETECTED" : "NO PRESS");          Serial.print(") | ");
    Serial.print("MEQI: ");         Serial.print(current_meqi.meqi_score, 1); Serial.print("% | ");
    Serial.print("Curtain: ");      Serial.println(curtain_open ? "OPEN" : "CLOSED");

    // 5. COMPLETE CSV MQTT PAYLOAD (Includes numerical pF AND press status)
    // Format: millis, light_pct, therm_raw_F, therm_kalman_F, bmp_temp_F, temp_diff_pct, bmp_press_Pa, ultra_duration_us, ultra_dist_cm, cap_pF, cap_status, meqi_score, curtain_state
    String csvPayload = String(now_ms) + "," +
                        String(light_pct, 1) + "," +
                        String(therm_raw_F, 2) + "," +
                        String(therm_kalman_F, 2) + "," +
                        String(bmpData.tempF, 2) + "," +
                        String(temp_diff_pct, 2) + "," +
                        String(bmpData.pressurePa, 1) + "," +
                        String(ultraData.duration_us) + "," +
                        String(ultraData.distance_cm, 1) + "," +
                        String(capData.total_C_pF, 1) + "," +
                        String(capData.pressed ? "PRESS DETECTED" : "NO PRESS") + "," +
                        String(current_meqi.meqi_score, 1) + "," +
                        String(curtain_open ? "OPEN" : "CLOSED");

    mqttClient.publish(MQTT_TOPIC, csvPayload.c_str());
  }

  updateLEDDisplay(now_ms);

  delay(20); 
}