
#include <SPI.h>
#include <RH_RF69.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>

#define DEBUG

/* RF69 Device Id */
#define SELF_ID 0x10 // 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80

/* RF69 US Frequency */
#define RF69_FREQUENCY 915.0

/* RF69 Encryption key, 16 bytes */
#define RF69_ENCRYPTION_KEY { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* Event Keep-alive */
#define EVENT_KEEPALIVE (30 * 1000)

/* Vibration Duration */
#define VIBRATION_DURATION (5 * 1000)

/**
 * Message Packet Protocol
 */
#define MP_VERSION 0

#define MP_ACTION_ALERT 'A'
#define MP_ACTION_RESPOND 'R'
#define MP_ACTION_ACK 'K'

typedef struct {
  uint8_t version = MP_VERSION;
  uint8_t id;
  char action;
} MessagePacket;


/**
 * RF69 Radio
 */

/* RF69 Encryption Key */
uint8_t encryption_key[16] = RF69_ENCRYPTION_KEY;

/* RF69 Pin Configuration */
#define RFM69_CS 8
#define RFM69_INT 3
#define RFM69_RST 4

/* hardware configuration */
#define VBAT_PIN A7
#define BUTTON_PIN A0
#define MOTOR_PIN A1

/* rf69 Radio */
RH_RF69 radio(RFM69_CS, RFM69_INT);

/* LED Matrix */
Adafruit_8x8matrix ledMatrix = Adafruit_8x8matrix();

/* Local State */
volatile bool _buttonPressed = false;
volatile unsigned long _vibrationAt = 0;
volatile unsigned long _eventAt = 0;

volatile uint8_t _alerts = 0;
volatile uint8_t _responses = 0;
volatile uint8_t _acks = 0;

void setup() {
  Serial.begin(115200);

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // manual reset rf69
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!radio.init()) {
    // TODO: Error state!
    while (1);
  }

  if (!radio.setFrequency(RF69_FREQUENCY)) {
    // TODO: Error state!
    while (1);
  }

  radio.setTxPower(20, true);

  radio.setEncryptionKey(encryption_key);
  radio.setPromiscuous(true);

  ledMatrix.begin(0x70);
  ledMatrix.clear();
  ledMatrix.writeDisplay();

  pinMode(VBAT_PIN, INPUT);

  pinMode(BUTTON_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button_ISR, RISING);

  pinMode(MOTOR_PIN, OUTPUT);
  stopVibration();
}

void loop() {
#ifdef DEBUG
  batteryVoltage();
#endif

  if (buttonPressed()) {
    processButtonPress();
  }

  processMessages();

  if (endEvent()) {
    clearEvent();
  }

  if (shouldStopVibration()) {
    stopVibration();
  }

  delay(1000);
}

void button_ISR() {
#ifdef DEBUG
  Serial.println("button_ISR");
#endif

  setEventAt();
  markButtonPress();
}

#ifdef DEBUG
void batteryVoltage() {
  float batteryVoltage = analogRead(VBAT_PIN) * 2 * 3.3 / 1024;

  Serial.print("Battery Voltage: ");
  Serial.println(batteryVoltage);
}
#endif

void setEventAt() {
  _eventAt = millis();
}

void clearEventAt() {
  _eventAt = 0;
}

bool endEvent() {
#ifdef DEBUG
//  Serial.print("_eventAt: ");
//  Serial.print(_eventAt);
//  Serial.print(" timeout: ");
//  Serial.print(_eventAt + EVENT_KEEPALIVE);
//  Serial.print(" millis: ");
//  Serial.println(millis());
#endif
  return _eventAt && (_eventAt + EVENT_KEEPALIVE) < millis();
}

bool buttonPressed() {
#ifdef DEBUG
//  Serial.print("buttonPressed: ");
//  Serial.println(_buttonPressed ? "true" : "false");
#endif

  return _buttonPressed;
}

void processButtonPress() {
#ifdef DEBUG
  Serial.println("processButtonPress");
#endif

  if (_alerts & SELF_ID) {
#ifdef DEBUG
    Serial.println("processButtonPress: already alerted");
#endif
  } else if (_responses & SELF_ID) {
#ifdef DEBUG
    Serial.println("processButtonPress: already responsed");
#endif
  } else if (_alerts) {
#ifdef DEBUG
    Serial.println("processButtonPress: respond");
#endif
    respond();
  } else {
#ifdef DEBUG
    Serial.println("processButtonPress: alert");
#endif
    alert();
  }

  clearButtonPress();
}

void markButtonPress() {
#ifdef DEBUG
  Serial.println("buttonPressed: mark");
#endif
  stopVibration();
  _buttonPressed = true;
}

void clearButtonPress() {
#ifdef DEBUG
  Serial.println("buttonPressed: clear");
#endif
  _buttonPressed = false;
}

void alert() {
  alertLightOn(SELF_ID);
  broadcastAlert();
}

void respond() {
  responseLightOn(SELF_ID);
  broadcastRespond();
}

void alertLightOn(int id) {
  _alerts |= id;
  writeLeds();
}

void responseLightOn(int id) {
  _responses |= id;
  writeLeds();
}

void ackLightOn(int id) {
  _acks |= id;
  writeLeds();
}

void writeLeds() {
#ifdef DEBUG
  Serial.print("LEDS: ");
  Serial.print(" _alerts: ");
  Serial.print(_alerts, BIN);
  Serial.print(" _responses: ");
  Serial.print(_responses, BIN);
  Serial.print(" _acks: ");
  Serial.print(_acks, BIN);
#endif

  for (int i = 0; i < 8; i++) {
    Serial.print("i: ");
    Serial.print(i);
    Serial.print(" _alerts: ");
    Serial.print(_alerts, BIN);
    Serial.print(" _responses: ");
    Serial.print(_responses, BIN);
    Serial.print(" _acks: ");
    Serial.print(_acks, BIN);
    Serial.print(" offset: " );
    Serial.println((0x01 << i));

    ledMatrix.drawPixel(7, i, _alerts & (0x01 << i) ? LED_ON : LED_OFF);
    ledMatrix.drawPixel(6, i, _responses & (0x01 << i) ? LED_ON : LED_OFF);
    ledMatrix.drawPixel(5, i, _acks & (0x01 << i) ? LED_ON : LED_OFF);
  }

  ledMatrix.writeDisplay();
}

void clearLeds() {
  ledMatrix.clear();
  ledMatrix.writeDisplay();
}

void broadcastAlert() {
  MessagePacket message;
  message.id = SELF_ID;
  message.action = MP_ACTION_ALERT;

  send(&message);
}

void broadcastRespond() {
  MessagePacket message;
  message.id = SELF_ID;
  message.action = MP_ACTION_RESPOND;

  send(&message);
}

void broadcastAck() {
  Serial.println("broadcast: ACK");
  MessagePacket message;
  message.id = SELF_ID;
  message.action = MP_ACTION_ACK;

  send(&message);
}

void send(MessagePacket *message) {
  char buffer[radio.maxMessageLength()];

  snprintf(buffer, sizeof(buffer),
    "%02d%02d%c",
    message->version, message->id, message->action);

#ifdef DEBUG
  Serial.print("SEND: ");
  Serial.println(buffer);
#endif

  radio.send((uint8_t*) buffer, strlen(buffer));
}

void processMessages() {
  while (hasMessages()) {
    processMessage();
  }
}

bool hasMessages() {
  return radio.available();
}

void processMessage() {
  uint8_t buffer[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t length = sizeof(buffer);

  if (radio.recv(buffer, &length)) {
    if (!length) {
      return;
    }
    buffer[length] = 0;

    Serial.print("Received [");
    Serial.print(length);
    Serial.print("]: ");
    Serial.println((char*) buffer);
    Serial.print("RSSI: ");
    Serial.println(radio.lastRssi(), DEC);

    receive(buffer);
  }
}

void receive(uint8_t *buffer) {
  char tmp[8];

  MessagePacket message;

  strncpy(tmp, (char*) &buffer[0], 2);
  message.version = atoi(tmp);

  if (message.version != MP_VERSION) {
    // Unsupported version
    return;
  }

  strncpy(tmp, (char*) &buffer[2], 2);
  message.id = atoi(tmp);

  message.action = buffer[4];

  if (message.action == MP_ACTION_ALERT) {
    Serial.println("receive: ALERT");
    setEventAt();
    broadcastAck();
    alertLightOn(message.id);
    startVibration();
  } else if (message.action == MP_ACTION_RESPOND) {
    Serial.println("receive: RESPOND");
    setEventAt();
    responseLightOn(message.id);
    startVibration();
  } else if (message.action == MP_ACTION_ACK) {
    Serial.println("receive: ACK");
    setEventAt();
    ackLightOn(message.id);
  }
}

void clearEvent() {
#ifdef DEBUG
  Serial.println("clearEvent");
#endif

  clearEventAt();
  clearAlerts();
  clearResponses();
  clearAcks();
  clearLeds();
}

void clearAlerts() {
  _alerts = 0;
}

void clearResponses() {
  _responses = 0;
}

void clearAcks() {
  _acks = 0;
}

#pragma mark -- Vibration

void startVibration() {
  setVibrationAt();
  digitalWrite(MOTOR_PIN, HIGH);
}

void stopVibration() {
  clearVibrationAt();
  digitalWrite(MOTOR_PIN, LOW);
}

bool shouldStopVibration() {
  return _vibrationAt && (_vibrationAt + VIBRATION_DURATION) < millis();
}

void setVibrationAt() {
  _vibrationAt = millis();
}

void clearVibrationAt() {
  _vibrationAt = 0;
}
