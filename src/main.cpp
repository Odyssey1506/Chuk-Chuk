
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>

// ---------- Level sensor ----------
constexpr uint8_t LEVEL_PIN = PA1;
constexpr float SHUNT_OHMS = 97.5f;
constexpr float ADC_REFERENCE_V = 3.3f;
constexpr float ADC_MAX_COUNT = 4095.0f;
constexpr uint8_t SAMPLE_COUNT = 32;

constexpr float SENSOR_MIN_MA = 4.0f;
constexpr float SENSOR_MAX_MA = 20.0f;
constexpr float SENSOR_RANGE_MM = 2000.0f;
constexpr float TANK_HEIGHT_MM = 590.0f;

// ---------- BMP390 ----------
Adafruit_BMP3XX bmp;
bool bmpReady = false;

// ---------- SIM7600G ----------
// TXD -> PA10 (STM32 RX)
// RXD -> PB6  (STM32 TX)
Uart ModemSerial(USART1);

uint32_t lastModemTest = 0;
constexpr uint32_t MODEM_TEST_INTERVAL_MS = 30000;

// ---------- Read level sensor ----------
void readLevelSensor() {
  uint32_t sum = 0;

  for (uint8_t i = 0; i < SAMPLE_COUNT; i++) {
    sum += analogRead(LEVEL_PIN);
    delay(2);
  }

  float raw = (float)sum / SAMPLE_COUNT;
  float voltage = raw * ADC_REFERENCE_V / ADC_MAX_COUNT;
  float currentMA = (voltage / SHUNT_OHMS) * 1000.0f;

  // Convert 4–20 mA to sensor depth in mm.
  float sensorDepthMM =
      ((currentMA - SENSOR_MIN_MA) /
       (SENSOR_MAX_MA - SENSOR_MIN_MA)) * SENSOR_RANGE_MM;

  bool currentValid = currentMA >= 3.6f && currentMA <= 21.0f;

  Serial.println("\n--- LEVEL SENSOR ---");
  Serial.print("ADC raw: ");
  Serial.println(raw, 1);

  Serial.print("ADC voltage: ");
  Serial.print(voltage, 3);
  Serial.println(" V");

  Serial.print("Loop current: ");
  Serial.print(currentMA, 2);
  Serial.println(" mA");

  if (!currentValid) {
    Serial.println("WARNING: Current out of expected range.");
    Serial.println("Check sensor power, wiring and shunt.");
    return;
  }

  // Assumes the sensor measures water depth from the tank bottom.
  float tankLevelMM = constrain(sensorDepthMM, 0.0f, TANK_HEIGHT_MM);
  float fillPercent = (tankLevelMM / TANK_HEIGHT_MM) * 100.0f;

  Serial.print("Calculated sensor depth: ");
  Serial.print(sensorDepthMM, 0);
  Serial.println(" mm");

  Serial.print("Tank level (clamped): ");
  Serial.print(tankLevelMM, 0);
  Serial.println(" mm");

  Serial.print("Tank fill: ");
  Serial.print(fillPercent, 1);
  Serial.println(" %");
}

// ---------- Initialize BMP390 ----------
void initBMP() {
  Wire.setSCL(PB8);
  Wire.setSDA(PB9);
  Wire.begin();
  Wire.setClock(100000);

  if (bmp.begin_I2C(0x77, &Wire) ||
      bmp.begin_I2C(0x76, &Wire)) {
    bmpReady = true;

    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

    Serial.println("[BMP390] Found");
  } else {
    Serial.println("[BMP390] Not found - check wiring.");
  }
}

// ---------- Test SIM7600G ----------
void testModem() {
  while (ModemSerial.available()) {
    ModemSerial.read();
  }

  Serial.println("\n[MODEM] Sending AT...");
  ModemSerial.println("AT");

  uint32_t start = millis();

  while (millis() - start < 2000) {
    while (ModemSerial.available()) {
      String response = ModemSerial.readString();
      response.trim();

      if (response.length() > 0) {
        Serial.print("[MODEM] Response: ");
        Serial.println(response);
        return;
      }
    }
    delay(10);
  }

  Serial.println("[MODEM] No response.");
  Serial.println("Check modem power, UART wiring and baud rate.");
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("STM32 BOOT OK");

  Serial.println("\n==============================");
  Serial.println("Industrial Tank Monitor");
  Serial.println("STM32 Nucleo-L476RG");
  Serial.println("==============================");

  analogReadResolution(12);
  pinMode(LEVEL_PIN, INPUT_ANALOG);

  initBMP();

  // SIM7600G UART
  ModemSerial.setRx(PA10);
  ModemSerial.setTx(PB6);
  ModemSerial.begin(115200);
  ModemSerial.setTimeout(200);

  delay(500);
  testModem();
  lastModemTest = millis();
}

// ---------- Main loop ----------
void loop() {
  readLevelSensor();

  Serial.println("\n--- BMP390 ---");

  if (bmpReady && bmp.performReading()) {
    Serial.print("Temperature: ");
    Serial.print(bmp.temperature, 2);
    Serial.println(" °C");

    Serial.print("Pressure: ");
    Serial.print(bmp.pressure / 100.0f, 2);
    Serial.println(" hPa");
  } else {
    Serial.println("BMP390 reading unavailable.");
  }

  if (millis() - lastModemTest >= MODEM_TEST_INTERVAL_MS) {
    testModem();
    lastModemTest = millis();
  }

  delay(2000);
}
