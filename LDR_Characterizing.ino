#include <Arduino.h>

// --- Pin Definitions ---
const int LED_PIN = 25;  // PWM Output Pin driving the LED
const int ADC_PIN = 34;  // ADC Input measuring V_out
const float VCC   = 3.30; // Circuit Supply Voltage

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Configure Pins
  pinMode(LED_PIN, OUTPUT);
  analogSetPinAttenuation(ADC_PIN, ADC_11db);
  pinMode(ADC_PIN, INPUT);

  Serial.println("\n--- START TEST SWEEP ---");
  Serial.println("Relative_Intensity_Pct,Voltage");
  delay(1000);
}

void loop() {
  // Sweep PWM from 0 to 255 in steps of 13 (~5% brightness increments)
  for (int pwm_val = 0; pwm_val <= 255; pwm_val += 13) {
    // 1. Set LED Brightness
    analogWrite(LED_PIN, pwm_val);
    
    // 2. Allow LDR and LED thermal settling
    delay(150);

    // 3. Average 32 ADC readings to reduce noise
    long raw_sum = 0;
    const int SAMPLES = 32;
    for (int i = 0; i < SAMPLES; i++) {
      raw_sum += analogRead(ADC_PIN);
      delay(1);
    }
    float raw_avg = (float)raw_sum / (float)SAMPLES;

    // 4. Calculate actual hardware voltage at Pin 34
    float v_out = (raw_avg / 4095.0) * VCC;

    // 5. Convert PWM count to relative percentage
    float intensity_pct = (pwm_val / 255.0) * 100.0;

    // --- OUTPUT FORMAT (SERIAL PLOTTER & CSV COMPATIBLE) ---
    Serial.print("Intensity_Pct:");
    Serial.print(intensity_pct, 1);
    Serial.print(",");
    Serial.print("Voltage:");
    Serial.println(v_out, 3);
  }

  // Turn LED off and wait before next sweep
  analogWrite(LED_PIN, 0);
  delay(3000);
}