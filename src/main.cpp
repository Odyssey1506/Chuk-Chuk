#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <math.h>
#include <string.h>

// STM32 Nucleo-L476RG + SIM7600X
// Modem TXD -> PA10 (STM32 RX)
// Modem RXD -> PB6  (STM32 TX)
// Common GND required. Power SIM7600X from a suitable external supply.

Uart ModemSerial(USART1);
Adafruit_BMP3XX bmp;

static const uint32_t USB_BAUD = 115200;
static const uint32_t MODEM_BAUD = 115200;

static const uint8_t LEVEL_PIN = PA1;
static const float SHUNT_OHMS = 97.5f;
static const float ADC_VREF = 3.3f;
static const float ADC_MAX = 4095.0f;
static const float SENSOR_RANGE_MM = 2000.0f;
static const float TANK_HEIGHT_MM = 590.0f;

static const char TRAIN_ID[] = "TANK-001";
static const char APN[] = "JioNet";
static const char API_URL[] =
    "https://water-level-sensor.onrender.com/api/v1/telemetry";

bool bmpReady = false;
char inputLine[240];
size_t inputLength = 0;

// Read and print modem output for a fixed interval. Any received bytes are
// printed to USB Serial and returned, allowing AT commands typed by the user.
String readModemFor(uint32_t timeoutMs) {
  String response;
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    while (ModemSerial.available()) {
      char c = (char)ModemSerial.read();
      Serial.write(c);
      response += c;
    }
    delay(1);
  }
  return response;
}

String sendAT(const char *cmd, uint32_t timeoutMs = 3000) {
  while (ModemSerial.available()) ModemSerial.read();

  Serial.print("\r\n>> ");
  Serial.println(cmd);
  ModemSerial.print(cmd);
  ModemSerial.print("\r\n");

  String response = readModemFor(timeoutMs);
  Serial.println();
  return response;
}

float readLoopCurrentmA() {
  uint32_t sum = 0;
  for (int i = 0; i < 64; ++i) {
    sum += analogRead(LEVEL_PIN);
    delay(2);
  }
  const float adc = sum / 64.0f;
  const float voltage = adc * ADC_VREF / ADC_MAX;
  return voltage / SHUNT_OHMS * 1000.0f;
}

float calculateLevelMM(float currentmA) {
  float level = (currentmA - 4.0f) * SENSOR_RANGE_MM / 16.0f;
  if (level < 0.0f) level = 0.0f;
  if (level > TANK_HEIGHT_MM) level = TANK_HEIGHT_MM;
  return level;
}

// SIM7600 CCLK format is typically: +CCLK: "yy/MM/dd,hh:mm:ss+zz"
// zz is in quarter-hour units. Preserve its offset in the ISO timestamp.
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
    String quarters = value.substring(18, 20);
    int q = quarters.toInt();
    int totalMinutes = q * 15;
    int oh = totalMinutes / 60;
    int om = totalMinutes % 60;
    char buf[8];
    snprintf(buf, sizeof(buf), "%c%02d:%02d", sign, oh, om);
    offset = String(buf);
  } else {
    // Unknown timezone: do not falsely label local time as UTC.
    offset = "";
  }

  String stamp = "20" + yy + "-" + mm + "-" + dd + "T" +
                 hh + ":" + mi + ":" + ss + offset;
  return stamp;
}

bool waitForModemText(const char *needle, uint32_t timeoutMs, String *capture = nullptr) {
  String got;
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    while (ModemSerial.available()) {
      char c = (char)ModemSerial.read();
      Serial.write(c);
      got += c;
      if (got.indexOf(needle) >= 0) {
        if (capture) *capture += got;
        return true;
      }
    }
    delay(1);
  }
  if (capture) *capture += got;
  return false;
}

bool postTelemetry() {
  float currentmA = readLoopCurrentmA();
  Serial.println("\r\n--- SENSOR READINGS ---");
  Serial.print("Loop current: ");
  Serial.print(currentmA, 3);
  Serial.println(" mA");

  if (!isfinite(currentmA) || currentmA < 3.5f || currentmA > 21.0f) {
    Serial.println("ERROR: Current is outside the expected 4-20 mA sensor range.");
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
    Serial.println("WARNING: BMP390 reading unavailable; values will be null.");
  }

  String timestamp = getTimestampISO();
  if (timestamp.length() == 0) {
    Serial.println("ERROR: Modem clock unavailable. Set time or check AT+CCLK?.");
    return false;
  }

  char pressureJson[24];
  char temperatureJson[24];
  if (isfinite(pressureHpa)) snprintf(pressureJson, sizeof(pressureJson), "%.2f", pressureHpa);
  else strcpy(pressureJson, "null");
  if (isfinite(temperatureC)) snprintf(temperatureJson, sizeof(temperatureJson), "%.2f", temperatureC);
  else strcpy(temperatureJson, "null");

  char json[500];
  snprintf(json, sizeof(json),
    "{\"train_id\":\"%s\",\"timestamp\":\"%s\","
    "\"latitude\":null,\"longitude\":null,"
    "\"level_mm\":%.1f,\"level_percent\":%.1f,"
    "\"loop_current_ma\":%.3f,\"pressure_hpa\":%s,\"temperature_c\":%s}",
    TRAIN_ID, timestamp.c_str(), levelMM, levelPercent, currentmA,
    pressureJson, temperatureJson);

  Serial.println("\r\n--- JSON TO SEND ---");
  Serial.println(json);

  sendAT("AT+HTTPTERM", 1000); // ERROR is normal when no HTTP session exists.
  String r = sendAT("AT+HTTPINIT", 5000);
  if (r.indexOf("OK") < 0) {
    Serial.println("ERROR: HTTPINIT failed.");
    return false;
  }

  sendAT("AT+HTTPPARA=\"CID\",1", 2500);
  sendAT("AT+CSSLCFG=\"enableSNI\",0,1", 2500);

  char cmd[260];
  snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"", API_URL);
  r = sendAT(cmd, 4000);
  if (r.indexOf("OK") < 0) {
    Serial.println("ERROR: Could not set HTTP URL.");
    sendAT("AT+HTTPTERM", 1000);
    return false;
  }

  r = sendAT("AT+HTTPPARA=\"CONTENT\",\"application/json\"", 2500);
  if (r.indexOf("OK") < 0) {
    Serial.println("ERROR: Could not set JSON content type.");
    sendAT("AT+HTTPTERM", 1000);
    return false;
  }

  snprintf(cmd, sizeof(cmd), "AT+HTTPDATA=%u,10000", (unsigned int)strlen(json));
  r = sendAT(cmd, 4000);
  if (r.indexOf("DOWNLOAD") < 0) {
    Serial.println("ERROR: Modem did not provide DOWNLOAD prompt.");
    sendAT("AT+HTTPTERM", 1000);
    return false;
  }

  Serial.println("\r\n>> [JSON request body]");
  ModemSerial.print(json);
  if (!waitForModemText("OK", 12000)) {
    Serial.println("\r\nERROR: Modem did not confirm HTTPDATA.");
    sendAT("AT+HTTPTERM", 1000);
    return false;
  }

  while (ModemSerial.available()) ModemSerial.read();
  Serial.println("\r\n>> AT+HTTPACTION=1");
  ModemSerial.print("AT+HTTPACTION=1\r\n");

  String actionResponse;
  uint32_t start = millis();
  bool gotAction = false;
  while (millis() - start < 90000) {
    while (ModemSerial.available()) {
      char c = (char)ModemSerial.read();
      Serial.write(c);
      actionResponse += c;
      int p = actionResponse.indexOf("+HTTPACTION:");
      if (p >= 0 && actionResponse.indexOf('\n', p) >= 0) {
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
  int comma2 = (comma1 >= 0) ? actionResponse.indexOf(',', comma1 + 1) : -1;
  int method = -1;
  int status = -1;
  int bodyLength = 0;
  if (gotAction && comma1 >= 0 && comma2 >= 0) {
    method = actionResponse.substring(p + 12, comma1).toInt();
    status = actionResponse.substring(comma1 + 1, comma2).toInt();
    int lineEnd = actionResponse.indexOf('\n', comma2 + 1);
    String lenStr = (lineEnd >= 0)
      ? actionResponse.substring(comma2 + 1, lineEnd)
      : actionResponse.substring(comma2 + 1);
    bodyLength = lenStr.toInt();
  }

  Serial.print("HTTP method: "); Serial.println(method);
  Serial.print("HTTP status/error: "); Serial.println(status);
  Serial.print("Response body length: "); Serial.println(bodyLength);
  sendAT("AT+HTTPTERM", 1000);

  if (method == 1 && status >= 200 && status < 300) {
    Serial.println("SUCCESS: Telemetry POST accepted by HTTP endpoint.");
    return true;
  }
  Serial.println("POST not successful. If status is 713, investigate modem DNS/network.");
  return false;
}

void printHelp() {
  Serial.println("\r\nCommands:");
  Serial.println("  AT commands: type any modem command, e.g. AT or AT+CPIN?");
  Serial.println("  POST        : read sensors and POST telemetry");
  Serial.println("  SENSORS     : print current sensor readings");
  Serial.println("  HELP        : show this help");
  Serial.println("Use Newline or Both NL & CR as line ending.");
}

void printSensors() {
  float currentmA = readLoopCurrentmA();
  float levelMM = calculateLevelMM(currentmA);
  Serial.println("\r\n--- SENSOR READINGS ---");
  Serial.print("Loop current: "); Serial.print(currentmA, 3); Serial.println(" mA");
  Serial.print("Level: "); Serial.print(levelMM, 1); Serial.println(" mm");
  Serial.print("Fill: "); Serial.print(levelMM / TANK_HEIGHT_MM * 100.0f, 1); Serial.println(" %");
  if (bmpReady && bmp.performReading()) {
    Serial.print("Pressure: "); Serial.print(bmp.pressure / 100.0f, 2); Serial.println(" hPa");
    Serial.print("Temperature: "); Serial.print(bmp.temperature, 2); Serial.println(" C");
  } else {
    Serial.println("BMP390: unavailable");
  }
}

void handleCommand(char *line) {
  while (*line == ' ' || *line == '\t') ++line;
  size_t len = strlen(line);
  while (len && (line[len - 1] == ' ' || line[len - 1] == '\t')) line[--len] = '\0';
  if (len == 0) return;

  Serial.print("\r\n[COMMAND] ");
  Serial.println(line);

  if (!strcmp(line, "HELP") || !strcmp(line, "help")) {
    printHelp();
  } else if (!strcmp(line, "POST") || !strcmp(line, "post")) {
    postTelemetry();
  } else if (!strcmp(line, "SENSORS") || !strcmp(line, "sensors")) {
    printSensors();
  } else {
    // Pass through any typed AT command and display the modem's response.
    sendAT(line, 5000);
  }
}

void setup() {
  Serial.begin(USB_BAUD);
  uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 4000) delay(10);
  delay(300);
  Serial.println("\r\n=== STM32 NUCLEO-L476RG / SIM7600X ===");
  Serial.println("USB serial ready at 115200 baud.");
  Serial.println("Type HELP and press Enter.");
  Serial.println("If the modem is wired/powered, an initial AT test follows.");

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
    Serial.println("BMP390 detected.");
  } else {
    Serial.println("WARNING: BMP390 not detected at 0x76 or 0x77.");
  }

  ModemSerial.setRx(PA10);
  ModemSerial.setTx(PB6);
  ModemSerial.begin(MODEM_BAUD);
  delay(500);
  sendAT("AT", 2500);
  printHelp();
}

void loop() {
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

  // Print any unsolicited modem notifications (e.g. network registration).
  while (ModemSerial.available()) {
    Serial.write(ModemSerial.read());
  }
}
