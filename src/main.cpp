
#include <Arduino.h>

Uart ModemSerial(USART1);

const char* commands[] = {
  "AT+CPIN?",
  "AT+CSQ",
  "AT+CEREG?",
  "AT+CGATT?"
};

const int COMMAND_COUNT = sizeof(commands) / sizeof(commands[0]);

void sendATCommand(const char* command) {
  // Clear old modem data
  while (ModemSerial.available()) {
    ModemSerial.read();
  }

  Serial.println();
  Serial.print(">> ");
  Serial.println(command);

  ModemSerial.print(command);
  ModemSerial.print("\r\n");

  unsigned long start = millis();
  bool received = false;

  while (millis() - start < 3000) {
    while (ModemSerial.available()) {
      char c = (char)ModemSerial.read();
      Serial.write(c);
      received = true;
    }
    delay(1);
  }

  if (!received) {
    Serial.println("[No response]");
  }

  Serial.println();
  Serial.println("------------------------------");
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  ModemSerial.setRx(PA10);
  ModemSerial.setTx(PB6);
  ModemSerial.begin(115200);

  Serial.println();
  Serial.println("SIM7600X NETWORK DIAGNOSTICS");
  Serial.println("============================");

  delay(1000);

  // Check basic communication first
  sendATCommand("AT");

  for (int i = 0; i < COMMAND_COUNT; i++) {
    sendATCommand(commands[i]);
  }

  Serial.println();
  Serial.println("Diagnostics complete.");
}

void loop() {
  // One-time diagnostic test.
  // Reset the board to run it again.
}
