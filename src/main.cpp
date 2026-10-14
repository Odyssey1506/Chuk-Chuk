#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <math.h>
#include <string.h>

// Chuk-Chuk POC: STM32 Nucleo-L476RG + SIM7600X + BMP390
// UART: modem TXD -> PA10, modem RXD -> PB6
// I2C: BMP390 SDA -> PB9, SCL -> PB8
// Analog: 4-20 mA front-end output -> PA1
// Use a suitable external modem supply and common GND.
//
// This is fault-tolerant POC firmware, not safety-certified firmware.
// It cannot guarantee delivery during outages or power faults.

Uart ModemSerial(USART1);
Adafruit_BMP3XX bmp;

static const uint32_t USB_BAUD = 115200;
static const uint32_t MODEM_BAUD = 115200;
static const uint8_t LEVEL_PIN = PA1;
static const float SHUNT_OHMS = 97.5f;
static const float ADC_VREF = 3.3f;
static const float ADC_MAX = 4095.0f;
static const float SENSOR_RANGE_MM = 2000.0f;
static const float TANK_HEIGHT_MM = 200.0f;

static const char TRAIN_ID[] = "TANK-001";
static const char APN[] = "JioNet";
static const char API_URL[] =
    "https://water-level-sensor.onrender.com/api/v1/telemetry";

static const uint32_t TELEMETRY_INTERVAL_MS = 30000UL;
static const uint8_t MODEM_RESET_AFTER_FAILURES = 3;

uint32_t lastTelemetryTime = 0;
uint8_t consecutivePostFailures = 0;
uint8_t consecutiveAtFailures = 0;
bool bmpReady = false;

char inputLine[240];
size_t inputLength = 0;

// ---------------- MODEM HELPERS ----------------

String readModemFor(uint32_t timeoutMs) {
  String response;
  uint32_t start = millis();

  while ((uint32_t)(millis() - start) < timeoutMs) {
    while (ModemSerial.available()) {
      char c = (char)ModemSerial.read();
      Serial.write(c);
      response += c;
    }
    delay(1);
  }
  return response;
}

String sendAT(const char *cmd, uint32_t timeoutMs = 2500) {
  while (ModemSerial.available()) ModemSerial.read();

  Serial.print("\r\n>> ");
  Serial.println(cmd);
  ModemSerial.print(cmd);
  ModemSerial.print("\r\n");

  String response = readModemFor(timeoutMs);
  Serial.println();
  return response;
}

bool waitForModemText(const char *needle, uint32_t timeoutMs) {
  String got;
  uint32_t start = millis();

  while ((uint32_t)(millis() - start) < timeoutMs) {
    while (ModemSerial.available()) {
      char c = (char)ModemSerial.read();
      Serial.write(c);
      got += c;
      if (got.indexOf(needle) >= 0) return true;
    }
    delay(1);
  }
  return false;
}

bool modemResponds() {
  for (uint8_t i = 0; i < 2; ++i) {
    String r = sendAT("AT", 1800);
    if (r.indexOf("OK") >= 0) return true;
    delay(300);
  }
  return false;
}

// ---------------- SENSORS ----------------

float readLoopCurrentmA() {
  uint32_t sum = 0;
  for (int i = 0; i < 64; ++i) {
    sum += analogRead(LEVEL_PIN);
    delay(2);
  }

  float adc = sum / 64.0f;
  float voltage = adc * ADC_VREF / ADC_MAX;
  return (voltage / SHUNT_OHMS) * 1000.0f;
}

float calculateLevelMM(float currentmA) {
  float level = (currentmA - 4.0f) * SENSOR_RANGE_MM / 16.0f;
  if (level < 0.0f) level = 0.0f;
  if (level > TANK_HEIGHT_MM) level = TANK_HEIGHT_MM;
  return level;
}

// ---------------- TIME / NETWORK ----------------

String getTimestampISO() {
  String r = sendAT("AT+CCLK?", 2500);
  int p = r.indexOf("+CCLK: \"");
  if (p < 0) return "";
  p += 8;

  int quote = r.indexOf('"', p);
  if (quote < 0) return "";

  String value = r.substring(p, quote);
  if (value.length() < 17) return "";

  String yy = value.substring(0, 2);
  String mm = value.substring(3, 5);
  String dd = value.substring(6, 8);
  String hh = value.substring(9, 11);
  String mi = value.substring(12, 14);
  String ss = value.substring(15, 17);

  String offset = "";
  if (value.length() >= 20) {
    char sign = value.charAt(17);
    int quarters = value.substring(18, 20).toInt();
    int totalMinutes = quarters * 15;
    char buf[8];
    snprintf(buf, sizeof(buf), "%c%02d:%02d",
             sign, totalMinutes / 60, totalMinutes % 60);
    offset = String(buf);
  }

  return "20" + yy + "-" + mm + "-" + dd + "T" +
         hh + ":" + mi + ":" + ss + offset;
}

bool simReady() {
  String r = sendAT("AT+CPIN?", 2500);
  return r.indexOf("READY") >= 0;
}

bool networkRegistered() {
  String r = sendAT("AT+CEREG?", 2500);
  int p = r.indexOf("+CEREG:");
  if (p < 0) return false;

  int colon = r.indexOf(':', p);
  if (colon < 0) return false;

  String fields = r.substring(colon + 1);
  fields.trim();

  int comma = fields.indexOf(',');
  String status = (comma >= 0) ? fields.substring(comma + 1) : fields;
  status.trim();

  int end = status.indexOf('\r');
  if (end >= 0) status = status.substring(0, end);
  end = status.indexOf('\n');
  if (end >= 0) status = status.substring(0, end);
  status.trim();

  return status == "1" || status == "5";
}

bool packetAttached() {
  String r = sendAT("AT+CGATT?", 2500);
  return r.indexOf("+CGATT: 1") >= 0;
}

bool configureAPNAndAttach() {
  char cmd[120];
  snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", APN);

  String r = sendAT(cmd, 3000);
  if (r.indexOf("OK") < 0) return false;

  // Request packet attachment, then verify actual state.
  sendAT("AT+CGATT=1", 10000);
  delay(500);
  return packetAttached();
}

bool recoverModemConnection() {
  Serial.println("\r\n[RECOVERY] Checking modem...");

  if (!modemResponds()) {
    ++consecutiveAtFailures;
    Serial.println("[RECOVERY] No AT response.");

    if (consecutiveAtFailures >= MODEM_RESET_AFTER_FAILURES) {
      Serial.println("[RECOVERY] Restarting modem: AT+CFUN=1,1");
      sendAT("AT+CFUN=1,1", 2000);
      delay(30000);
      consecutiveAtFailures = 0;
    }
    return false;
  }

  consecutiveAtFailures = 0;

  if (!simReady()) {
    Serial.println("[RECOVERY] SIM not ready; retry next cycle.");
    return false;
  }

  if (!networkRegistered()) {
    Serial.println("[RECOVERY] Not registered; retry next cycle.");
    return false;
  }

  if (!configureAPNAndAttach()) {
    Serial.println("[RECOVERY] Packet data not attached; retry next cycle.");
    return false;
  }

  Serial.println("[RECOVERY] Modem/SIM/network/data attach OK.");
  return true;
}

// ---------------- HTTP POST ----------------

bool postTelemetry() {
  float currentmA = readLoopCurrentmA();

  Serial.println("\r\n--- SENSOR READINGS ---");
  Serial.print("Loop current: ");
  Serial.print(currentmA, 3);
  Serial.println(" mA");

  if (!isfinite(currentmA) || currentmA < 3.5f || currentmA > 21.0f) {
    Serial.println("[SENSOR] Current outside expected range; skip POST.");
    return false;
  }

  float levelMM = calculateLevelMM(currentmA);
  float levelPercent = levelMM / TANK_HEIGHT_MM * 100.0f;
  float pressureHpa = NAN;
  float temperatureC = NAN;

  if (bmpReady && bmp.performReading()) {
    pressureHpa = bmp.pressure / 100.0f;
    temperatureC = bmp.temperature;
  } else {
    Serial.println("[SENSOR] BMP390 unavailable; sending null values.");
  }

  String timestamp = getTimestampISO();
  if (timestamp.length() == 0) {
    Serial.println("[TIME] Modem clock unavailable; skip POST.");
    return false;
  }

  char pressureJson[24];
  char temperatureJson[24];
  if (isfinite(pressureHpa))
    snprintf(pressureJson, sizeof(pressureJson), "%.2f", pressureHpa);
  else
    strcpy(pressureJson, "null");

  if (isfinite(temperatureC))
    snprintf(temperatureJson, sizeof(temperatureJson), "%.2f", temperatureC);
  else
    strcpy(temperatureJson, "null");

  char json[500];
  snprintf(
      json, sizeof(json),
      "{\"train_id\":\"%s\",\"timestamp\":\"%s\","
      "\"latitude\":null,\"longitude\":null,"
      "\"level_mm\":%.1f,\"level_percent\":%.1f,"
      "\"loop_current_ma\":%.3f,"
      "\"pressure_hpa\":%s,\"temperature_c\":%s}",
      TRAIN_ID, timestamp.c_str(), levelMM, levelPercent, currentmA,
      pressureJson, temperatureJson);

  Serial.println("\r\n--- JSON TO SEND ---");
  Serial.println(json);

  sendAT("AT+HTTPTERM", 1200); // ERROR is normal if no session exists.

  String r = sendAT("AT+HTTPINIT", 5000);
  if (r.indexOf("OK") < 0) {
    Serial.println("[HTTP] HTTPINIT failed.");
    return false;
  }

  bool setupOK = true;
  r = sendAT("AT+HTTPPARA=\"CID\",1", 2500);
  if (r.indexOf("OK") < 0) setupOK = false;

  // Keep the SNI setting used in the existing firmware.
  // Some modem firmware may reject this command.
  sendAT("AT+CSSLCFG=\"enableSNI\",0,1", 2500);

  char cmd[300];
  snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"", API_URL);
  r = sendAT(cmd, 4000);
  if (r.indexOf("OK") < 0) setupOK = false;

  r = sendAT("AT+HTTPPARA=\"CONTENT\",\"application/json\"", 2500);
  if (r.indexOf("OK") < 0) setupOK = false;

  if (!setupOK) {
    Serial.println("[HTTP] HTTP parameter setup failed.");
    sendAT("AT+HTTPTERM", 1200);
    return false;
  }

  snprintf(cmd, sizeof(cmd), "AT+HTTPDATA=%u,10000",
           (unsigned int)strlen(json));
  r = sendAT(cmd, 5000);

  if (r.indexOf("DOWNLOAD") < 0) {
    Serial.println("[HTTP] No DOWNLOAD prompt.");
    sendAT("AT+HTTPTERM", 1200);
    return false;
  }

  Serial.println("\r\n>> [JSON request body]");
  ModemSerial.print(json);

  if (!waitForModemText("OK", 12000)) {
    Serial.println("[HTTP] Modem did not confirm HTTPDATA.");
    sendAT("AT+HTTPTERM", 1200);
    return false;
  }

  while (ModemSerial.available()) ModemSerial.read();

  Serial.println("\r\n>> AT+HTTPACTION=1");
  ModemSerial.print("AT+HTTPACTION=1\r\n");

  String actionResponse;
  uint32_t start = millis();
  bool gotAction = false;

  while ((uint32_t)(millis() - start) < 90000UL) {
    while (ModemSerial.available()) {
      char c = (char)ModemSerial.read();
      Serial.write(c);
      actionResponse += c;

      int pos = actionResponse.indexOf("+HTTPACTION:");
      if (pos >= 0 && actionResponse.indexOf('\n', pos) >= 0) {
        gotAction = true;
        break;
      }
    }
    if (gotAction) break;
    delay(1);
  }

  Serial.println();

  int p = actionResponse.indexOf("+HTTPACTION:");
  int comma1 = (p >= 0) ? actionResponse.indexOf(',', p) : -1;
  int comma2 = (comma1 >= 0)
                  ? actionResponse.indexOf(',', comma1 + 1)
                  : -1;

  int method = -1;
  int status = -1;
  if (gotAction && comma1 >= 0 && comma2 >= 0) {
    method = actionResponse.substring(p + 12, comma1).toInt();
    status = actionResponse.substring(comma1 + 1, comma2).toInt();
  }

  Serial.print("HTTP method: ");
  Serial.println(method);
  Serial.print("HTTP status/error: ");
  Serial.println(status);

  sendAT("AT+HTTPTERM", 1200);

  if (method == 1 && status >= 200 && status < 300) {
    Serial.println("[HTTP] POST accepted (2xx).");
    return true;
  }

  Serial.println("[HTTP] POST failed or timed out.");
  return false;
}

// ---------------- Commands ----------------

void printHelp() {
  Serial.println("\r\nCommands:");
  Serial.println("  POST    : send telemetry immediately");
  Serial.println("  SENSORS : print sensor readings");
  Serial.println("  HELP    : show commands");
  Serial.println("  AT...   : pass an AT command to modem");
  Serial.println("Automatic telemetry interval: 30 seconds.");
}

void printSensors() {
  float currentmA = readLoopCurrentmA();
  float levelMM = calculateLevelMM(currentmA);

  Serial.println("\r\n--- SENSOR READINGS ---");
  Serial.print("Loop current: ");
  Serial.print(currentmA, 3);
  Serial.println(" mA");

  Serial.print("Level: ");
  Serial.print(levelMM, 1);
  Serial.println(" mm");

  Serial.print("Fill: ");
  Serial.print(levelMM / TANK_HEIGHT_MM * 100.0f, 1);
  Serial.println(" %");

  if (bmpReady && bmp.performReading()) {
    Serial.print("Pressure: ");
    Serial.print(bmp.pressure / 100.0f, 2);
    Serial.println(" hPa");
    Serial.print("Temperature: ");
    Serial.print(bmp.temperature, 2);
    Serial.println(" C");
  } else {
    Serial.println("BMP390: unavailable");
  }
}

void handleCommand(char *line) {
  while (*line == ' ' || *line == '\t') ++line;

  size_t len = strlen(line);
  while (len && (line[len - 1] == ' ' || line[len - 1] == '\t')) {
    line[--len] = '\0';
  }
  if (len == 0) return;

  Serial.print("\r\n[COMMAND] ");
  Serial.println(line);

  if (!strcmp(line, "HELP") || !strcmp(line, "help")) {
    printHelp();
  } else if (!strcmp(line, "POST") || !strcmp(line, "post")) {
    bool ok = postTelemetry();
    Serial.println(ok ? "[MANUAL] POST succeeded." : "[MANUAL] POST failed.");
    lastTelemetryTime = millis();
  } else if (!strcmp(line, "SENSORS") || !strcmp(line, "sensors")) {
    printSensors();
  } else {
    sendAT(line, 5000);
  }
}

// ---------------- Setup / loop ----------------

void setup() {
  Serial.begin(USB_BAUD);
  uint32_t waitStart = millis();
  while (!Serial && (uint32_t)(millis() - waitStart) < 4000UL) delay(10);
  delay(300);

  Serial.println("\r\n=== CHUK-CHUK STM32 / SIM7600X ===");
  Serial.println("Automatic telemetry starts after 30 seconds.");
  Serial.println("Type HELP for commands.");

  analogReadResolution(12);
  pinMode(LEVEL_PIN, INPUT);

  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.begin();

  bmpReady = bmp.begin_I2C(0x76);
  if (!bmpReady) bmpReady = bmp.begin_I2C(0x77);

  if (bmpReady) {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    Serial.println("[BOOT] BMP390 detected.");
  } else {
    Serial.println("[BOOT] WARNING: BMP390 not detected.");
  }

  ModemSerial.setRx(PA10);
  ModemSerial.setTx(PB6);
  ModemSerial.begin(MODEM_BAUD);
  delay(500);

  if (modemResponds()) {
    Serial.println("[BOOT] Modem responds to AT.");
  } else {
    Serial.println("[BOOT] Modem not responding yet; recovery will retry.");
  }

  printHelp();
  lastTelemetryTime = millis();
}

void loop() {
  // Keep manual serial commands available while connected to a laptop.
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      if (inputLength > 0) {
        inputLine[inputLength] = '\0';
        handleCommand(inputLine);
        inputLength = 0;
      }
    } else if (c >= 32 && c <= 126) {
      if (inputLength < sizeof(inputLine) - 1) {
        inputLine[inputLength++] = c;
      } else {
        inputLength = 0;
        Serial.println("\r\nInput too long; cleared command line.");
      }
    }
  }

  while (ModemSerial.available()) {
    Serial.write(ModemSerial.read());
  }

  uint32_t now = millis();
  if ((uint32_t)(now - lastTelemetryTime) >= TELEMETRY_INTERVAL_MS) {
    Serial.println("\r\n[AUTO] Starting telemetry cycle...");

    bool success = false;

    if (recoverModemConnection()) {
      success = postTelemetry();
    } else {
      Serial.println("[AUTO] Modem/network not ready; will retry.");
    }

    if (success) {
      consecutivePostFailures = 0;
      Serial.println("[AUTO] Telemetry delivered successfully.");
    } else {
      if (consecutivePostFailures < 255) ++consecutivePostFailures;
      Serial.print("[AUTO] Consecutive failed cycles: ");
      Serial.println(consecutivePostFailures);
    }

    // The interval starts after the attempt, so long network waits do
    // not cause a burst of back-to-back requests.
    lastTelemetryTime = millis();
  }
}
