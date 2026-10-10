#include <Arduino.h>

const char* net_setup_cmds[] = {
  "AT+CGDCONT=1,\"IP\",\"jionet\"", // Configure APN for Jio
  "AT+CGACT=1,1",                   // Activate PDP context #1
  "AT+NETOPEN",                     // Open SIM7600 onboard IP stack
  "AT+IPADDR"                       // Retrieve assigned public/carrier IP
};

const int NUM_CMDS = sizeof(net_setup_cmds) / sizeof(net_setup_cmds[0]);
int step_idx = 0;
unsigned long last_step = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  Serial1.setRx(PA10);
  Serial1.setTx(PB6);
  Serial1.begin(115200);

  delay(1500);
  Serial.println("\r\n================================================");
  Serial.println("     JIO 4G DATA BEARER ACTIVATION              ");
  Serial.println("================================================");
}

void loop() {
  while (Serial1.available()) {
    char c = Serial1.read();
    Serial.write(c);
  }

  if (millis() - last_step >= 3500 && step_idx < NUM_CMDS) {
    last_step = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    Serial.print("\r\n[STEP] ---> ");
    Serial.println(net_setup_cmds[step_idx]);

    Serial1.print(net_setup_cmds[step_idx]);
    Serial1.print("\r\n");

    step_idx++;
  }
}