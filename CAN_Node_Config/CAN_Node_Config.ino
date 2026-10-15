// SPDX-License-Identifier: MIT
// Copyright (c) 2026 NNITShimaLab contributors
#include <Arduino_CAN.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>
#include "NodeConfig.h"
using namespace nodecfg;

Node node;
Button button;
Adafruit_NeoPixel pixel(1, RGB_BUILTIN, NEO_GRB + NEO_KHZ800);
constexpr unsigned BOOT_PIN = 26;
bool canStarted = false;
uint32_t lastHello = 0, lastData = 0, lastLog = 0, busOffAt = 0;
uint32_t sequence = 0, rxCount = 0, txFailed = 0;
unsigned helloPart = 0;
char uidText[33];

bool saveId(uint16_t id) {
  Record old; EEPROM.get(0, old);
  Record next = {RECORD_MAGIC, id, uint16_t(~id)};
  if (memcmp(&old, &next, sizeof(next)) != 0) EEPROM.put(0, next);
  Record check; EEPROM.get(0, check);
  return validRecord(check) && check.id == id;
}

void showLed(uint32_t now) {
  uint32_t color = 0;
  if (node.state == Error) {
    unsigned phase = now % (unsigned(node.error) * 400 + 1400);
    if (phase < unsigned(node.error) * 400 && phase % 400 < 160) color = pixel.Color(24, 0, 0);
  } else if (node.state == Unset) {
    if ((now / 300) % 2 == 0) color = pixel.Color(24, 0, 0);
  } else if (node.state == Scanning || node.state == Claiming) {
    if ((now / 200) % 2 == 0) color = pixel.Color(16, 12, 0);
  } else {
    IdColor indication = idLed(node.id - FIRST_ID, now);
    if (indication == LedBlue) color = pixel.Color(0, 0, 24);
    else if (indication == LedGreen) color = pixel.Color(0, 24, 0);
  }
  static uint32_t previous = 0xFFFFFFFFu;
  if (color != previous) { pixel.setPixelColor(0, color); pixel.show(); previous = color; }
}

void setup() {
  Serial.begin(115200);
  pinMode(BOOT_PIN, INPUT_PULLUP);
  pinMode(PIN_RGB_EN, OUTPUT); digitalWrite(PIN_RGB_EN, HIGH);
  pixel.setBrightness(26); // Scale all existing LED colors to approximately 10%.
  pixel.begin(); pixel.clear(); pixel.show();
  const uint32_t *words = R_BSP_UniqueIdGet()->unique_id_words;
  for (unsigned i = 0; i < 4; ++i) for (unsigned j = 0; j < 4; ++j)
    node.uid[i * 4 + j] = uint8_t(words[i] >> (24 - 8 * j));
  for (unsigned i = 0; i < 16; ++i) snprintf(uidText + 2 * i, 3, "%02X", node.uid[i]);
  Record saved; EEPROM.get(0, saved);
  uint8_t identity[16]; memcpy(identity, node.uid, 16);
  node.begin(identity, validRecord(saved) ? saved.id : UNSET, millis());
  digitalWrite(8, LOW); pinMode(8, OUTPUT); // TCAN330 S must be LOW for transmission.
  canStarted = CAN.begin(CanBitRate::BR_250k);
  if (!canStarted) node.fail(CanFault);
}

void loop() {
  uint32_t now = millis();
  unsigned event = button.update(digitalRead(BOOT_PIN) == LOW, now);
  if (event == 2) {
    node.reset(now);
    if (!canStarted) node.fail(CanFault);
  } else if (event == 1 && canStarted) node.request(now);

  if (canStarted) {
    while (CAN.available()) {
      CanMsg msg = CAN.read();
      if (msg.isStandardId()) node.observeData(msg.getStandardId(), now);
      if (msg.data_length != 8) continue;
      if (msg.isExtendedId()) {
        Frame f; f.id = msg.getExtendedId(); memcpy(f.data, msg.data, 8); node.receive(f, now);
      } else if (validId(msg.getStandardId()) && msg.data[0] == 0xCA && msg.data[1] == 0xFE &&
                 msg.data[2] == msg.getStandardId() - FIRST_ID) {
        ++rxCount;
      }
    }
    node.tick(now);
    if (R_CAN0->STR_b.BOST) {
      if (!busOffAt) busOffAt = now;
      if (uint32_t(now - busOffAt) >= 2000) node.fail(CanFault);
    } else busOffAt = 0;
  }
  if (node.savePending) {
    node.savePending = false;
    if (!saveId(node.id)) node.fail(StorageFault);
  }
  if (canStarted && uint32_t(now - lastHello) >= 75 + node.tag % 31) {
    Frame f = node.announcement(helloPart);
    int rc = CAN.write(CanMsg(CanExtendedId(f.id), 8, f.data));
    if (rc >= 0) helloPart = (helloPart + 1) % 3;
    lastHello = now;
  }
  if (canStarted && node.dataReady(now) && uint32_t(now - lastData) >= 1000) {
    uint8_t data[8] = {0xCA, 0xFE, uint8_t(node.id - FIRST_ID), 1, 0, 0, 0, 0};
    memcpy(data + 4, &sequence, 4);
    int rc = CAN.write(CanMsg(CanStandardId(node.id), 8, data));
    if (rc < 0) ++txFailed;
    ++sequence; lastData = now;
  }
  showLed(now);
  if (Serial && uint32_t(now - lastLog) >= 1000) {
    Serial.print("UID="); Serial.print(uidText);
    Serial.print(" state="); Serial.print(unsigned(node.state));
    Serial.print(" ID=0x"); Serial.print(node.id, HEX);
    Serial.print(" error="); Serial.print(unsigned(node.error));
    Serial.print(" rx="); Serial.print(rxCount);
    Serial.print(" tx_failed="); Serial.print(txFailed);
    Serial.print(" TEC="); Serial.print(canStarted ? R_CAN0->TECR : 0);
    Serial.print(" REC="); Serial.println(canStarted ? R_CAN0->RECR : 0);
    lastLog = now;
  }
}




