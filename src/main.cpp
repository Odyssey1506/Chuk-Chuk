
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <math.h>
#include <stdlib.h>

// =====================================================
// STM32 NUCLEO-L476RG + SIM7600 + BMP390 + 4-20mA SENSOR
// Sends one telemetry record to the backend on startup.
// =====================================================

// ---------------- MODEM ----------------
Uart ModemSerial(USART1);

const char API_URL[] =
    "https://water-level-sensor.onrender.com/api/v1/telemetry";

const char TRAIN_ID[] = "TANK-001";

// ---------------- WATER LEVEL SENSOR ----------------
#define LEVEL_PIN PA1

const float SHUNT_OHMS = 97.5f;
const float ADC_VREF = 3.3f;
const float ADC_MAX = 4095.0f;

const float SENSOR_RANGE_MM = 2000.0f;
const float TANK_HEIGHT_MM = 590.0f;

// ---------------- BMP390 ----------------
Adafruit_BMP3XX bmp;
bool bmpReady = false;

// ---------------- MODEM RESPONSE HELPERS ----------------

// Send an AT command and collect its response.
String sendAT(const char *cmd, uint32_t timeoutMs = 3000)
{
    while (ModemSerial.available()) {
        ModemSerial.read();
    }

    Serial.print("\n>> ");
    Serial.println(cmd);

    ModemSerial.print(cmd);
    ModemSerial.print("\r\n");

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

    Serial.println();
    return response;
}

// Wait for a particular modem response, such as DOWNLOAD or OK.
bool waitForText(const char *expected, uint32_t timeoutMs,
                 String *captured = nullptr)
{
    String response;
    uint32_t start = millis();

    while (millis() - start < timeoutMs) {
        while (ModemSerial.available()) {
            char c = (char)ModemSerial.read();
            Serial.write(c);
            response += c;

            if (response.indexOf(expected) >= 0) {
                if (captured != nullptr) {
                    *captured += response;
                }
                return true;
            }
        }
        delay(1);
    }

    if (captured != nullptr) {
        *captured += response;
    }

    return false;
}

// ---------------- SENSOR READING ----------------

// Average 64 ADC samples to reduce noise.
float readLoopCurrentmA()
{
    uint32_t sum = 0;
    const int samples = 64;

    for (int i = 0; i < samples; i++) {
        sum += analogRead(LEVEL_PIN);
        delay(2);
    }

    float adc = sum / (float)samples;
    float voltage = adc * ADC_VREF / ADC_MAX;

    // Current in mA = voltage / resistance * 1000.
    return (voltage / SHUNT_OHMS) * 1000.0f;
}

// Convert 4-20mA into a 0-2000mm sensor measurement,
// then clamp to the physical tank height.
float calculateLevelMM(float currentmA)
{
    float level = (currentmA - 4.0f)
                  * SENSOR_RANGE_MM / 16.0f;

    if (level < 0.0f) level = 0.0f;
    if (level > TANK_HEIGHT_MM) level = TANK_HEIGHT_MM;

    return level;
}

// Read modem clock. This assumes the modem clock is UTC,
// as suggested by your earlier CCLK output.
String getTimestampUTC()
{
    String response = sendAT("AT+CCLK?", 1500);

    int p = response.indexOf("+CCLK: \"");
    if (p < 0) return "";

    p += 8;

    // Expected: YY/MM/DD,HH:MM:SS
    if (response.length() < (unsigned int)(p + 17)) {
        return "";
    }

    String yy = response.substring(p, p + 2);
    String mm = response.substring(p + 3, p + 5);
    String dd = response.substring(p + 6, p + 8);
    String hh = response.substring(p + 9, p + 11);
    String mi = response.substring(p + 12, p + 14);
    String ss = response.substring(p + 15, p + 17);

    // Do not invent a timestamp if the clock is invalid.
    if (yy.toInt() < 26 || mm.toInt() < 1 || mm.toInt() > 12 ||
        dd.toInt() < 1 || dd.toInt() > 31 ||
        hh.toInt() > 23 || mi.toInt() > 59 || ss.toInt() > 60) {
        return "";
    }

    return "20" + yy + "-" + mm + "-" + dd +
           "T" + hh + ":" + mi + ":" + ss + "Z";
}

// ---------------- HTTP POST ----------------

bool postTelemetry(const char *json)
{
    // End any leftover HTTP session. ERROR is harmless if
    // no HTTP session was active.
    sendAT("AT+HTTPTERM", 1000);

    String response = sendAT("AT+HTTPINIT", 5000);

    if (response.indexOf("OK") < 0) {
        Serial.println("ERROR: HTTPINIT failed.");
        return false;
    }

    sendAT("AT+HTTPPARA=\"CID\",1");

    // Enable SNI for the HTTPS hostname.
    sendAT("AT+CSSLCFG=\"enableSNI\",0,1");

    char urlCommand[220];
    snprintf(urlCommand, sizeof(urlCommand),
             "AT+HTTPPARA=\"URL\",\"%s\"", API_URL);

    response = sendAT(urlCommand);

    if (response.indexOf("OK") < 0) {
        Serial.println("ERROR: URL configuration failed.");
        sendAT("AT+HTTPTERM", 1000);
        return false;
    }

    response = sendAT(
        "AT+HTTPPARA=\"CONTENT\",\"application/json\""
    );

    if (response.indexOf("OK") < 0) {
        Serial.println("ERROR: Content-Type configuration failed.");
        sendAT("AT+HTTPTERM", 1000);
        return false;
    }

    // Tell the modem the exact number of JSON bytes.
    char dataCommand[64];
    snprintf(dataCommand, sizeof(dataCommand),
             "AT+HTTPDATA=%u,10000",
             (unsigned int)strlen(json));

    response = sendAT(dataCommand, 3000);

    if (response.indexOf("DOWNLOAD") < 0) {
        Serial.println("ERROR: Modem did not request JSON data.");
        sendAT("AT+HTTPTERM", 1000);
        return false;
    }

    Serial.println("\nSending JSON body...");
    ModemSerial.print(json);

    // Wait for modem to confirm it accepted the body.
    response = "";
    if (!waitForText("OK", 12000, &response)) {
        Serial.println("\nERROR: No HTTPDATA confirmation.");
        sendAT("AT+HTTPTERM", 1000);
        return false;
    }

    // Start POST. HTTPACTION returns an immediate OK,
    // then an asynchronous +HTTPACTION result.
    response = sendAT("AT+HTTPACTION=1", 3000);

    if (response.indexOf("+HTTPACTION:") < 0) {
        if (!waitForText("+HTTPACTION:", 90000, &response)) {
            Serial.println("\nERROR: Timed out waiting for HTTP result.");
            sendAT("AT+HTTPTERM", 1000);
            return false;
        }
    }

    Serial.println("\n--- HTTP RESULT ---");

    int p = response.indexOf("+HTTPACTION:");
    if (p < 0) {
        Serial.println("ERROR: HTTP result not found.");
        sendAT("AT+HTTPTERM", 1000);
        return false;
    }

    int firstComma = response.indexOf(',', p);
    int secondComma = response.indexOf(',', firstComma + 1);

    if (firstComma < 0 || secondComma < 0) {
        Serial.println("ERROR: Could not parse HTTP status.");
        sendAT("AT+HTTPTERM", 1000);
        return false;
    }

    int httpStatus =
        response.substring(firstComma + 1, secondComma).toInt();

    Serial.print("HTTP status: ");
    Serial.println(httpStatus);

    // Read the backend's JSON response.
    String body = sendAT("AT+HTTPREAD", 10000);

    bool received = body.indexOf("\"status\":\"received\"") >= 0;
    bool hasRecordId = body.indexOf("\"record_id\"") >= 0;

    sendAT("AT+HTTPTERM", 1000);

    if (httpStatus == 200 && received && hasRecordId) {
        Serial.println("\nSUCCESS: Backend confirmed telemetry.");
        return true;
    }

    Serial.println(
        "\nPOST completed, but backend confirmation was not verified."
    );
    Serial.println("Check HTTP status and response body above.");
    return false;
}

// ---------------- SETUP ----------------

void setup()
{
    Serial.begin(115200);
    delay(1500);

    Serial.println("\n=== TANK TELEMETRY START ===");

    // Configure ADC.
    analogReadResolution(12);
    pinMode(LEVEL_PIN, INPUT);

    // Configure I2C for BMP390.
    Wire.setSDA(PB9);
    Wire.setSCL(PB8);
    Wire.begin();

    bmpReady = bmp.begin_I2C(0x76);
    if (!bmpReady) {
        bmpReady = bmp.begin_I2C(0x77);
    }

    if (bmpReady) {
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        Serial.println("BMP390 detected.");
    } else {
        Serial.println("WARNING: BMP390 not detected.");
    }

    // Configure modem UART.
    ModemSerial.setRx(PA10);
    ModemSerial.setTx(PB6);
    ModemSerial.begin(115200);
    delay(1000);

    // Verify SIM and network.
    sendAT("AT");
    sendAT("AT+CPIN?");
    sendAT("AT+CEREG?");
    sendAT("AT+CGATT?");

    // Configure Jio APN and activate packet data.
    sendAT("AT+CGDCONT=1,\"IP\",\"JioNet\"");
    sendAT("AT+CGACT=1,1", 15000);

    // Read live sensor values.
    float currentmA = readLoopCurrentmA();

    Serial.println("\n--- SENSOR READINGS ---");
    Serial.print("Loop current: ");
    Serial.print(currentmA, 3);
    Serial.println(" mA");

    // Reject a disconnected or out-of-range sensor.
    if (!isfinite(currentmA) ||
        currentmA < 3.5f || currentmA > 21.0f) {
        Serial.println("ERROR: Invalid loop current; POST cancelled.");
        return;
    }

    float levelMM = calculateLevelMM(currentmA);
    float levelPercent = levelMM / TANK_HEIGHT_MM * 100.0f;

    float pressureHpa = NAN;
    float temperatureC = NAN;

    if (bmpReady && bmp.performReading()) {
        pressureHpa = bmp.pressure / 100.0f;
        temperatureC = bmp.temperature;
    }

    String timestamp = getTimestampUTC();

    if (timestamp.length() == 0) {
        Serial.println(
            "ERROR: Could not get a valid modem timestamp; POST cancelled."
        );
        return;
    }

    Serial.print("Level: ");
    Serial.print(levelMM, 1);
    Serial.println(" mm");

    Serial.print("Fill: ");
    Serial.print(levelPercent, 1);
    Serial.println(" %");

    Serial.print("Pressure: ");
    Serial.println(pressureHpa);

    Serial.print("Temperature: ");
    Serial.println(temperatureC);

    // Use JSON null when GPS is unavailable. Likewise, avoid
    // fabricating BMP390 readings if the sensor is unavailable.
    char pressureJson[24];
    char temperatureJson[24];

    if (isfinite(pressureHpa)) {
        snprintf(pressureJson, sizeof(pressureJson),
                 "%.2f", pressureHpa);
    } else {
        strcpy(pressureJson, "null");
    }

    if (isfinite(temperatureC)) {
        snprintf(temperatureJson, sizeof(temperatureJson),
                 "%.2f", temperatureC);
    } else {
        strcpy(temperatureJson, "null");
    }

    char json[500];

    snprintf(
        json, sizeof(json),
        "{\"train_id\":\"%s\","
        "\"timestamp\":\"%s\","
        "\"latitude\":null,"
        "\"longitude\":null,"
        "\"level_mm\":%.1f,"
        "\"level_percent\":%.1f,"
        "\"loop_current_ma\":%.3f,"
        "\"pressure_hpa\":%s,"
        "\"temperature_c\":%s}",
        TRAIN_ID,
        timestamp.c_str(),
        levelMM,
        levelPercent,
        currentmA,
        pressureJson,
        temperatureJson
    );

    Serial.println("\n--- JSON TO SEND ---");
    Serial.println(json);

    bool success = postTelemetry(json);

    Serial.println();
    if (success) {
        Serial.println("=== TELEMETRY SUCCESS ===");
    } else {
        Serial.println("=== TELEMETRY NOT CONFIRMED ===");
    }
}

void loop()
{
    // One POST per reset. Continuous reporting can be added
    // after the first successful telemetry record is confirmed.
}
