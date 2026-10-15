// SPDX-License-Identifier: MIT
// Copyright (c) 2026 NNITShimaLab contributors
#include "../CAN_Node_Config/NodeConfig.h"
#include <assert.h>
#include <stdio.h>
using namespace nodecfg;

void identity(uint8_t *uid, unsigned n) {
  memset(uid, 0, 16); uid[12] = uint8_t(n >> 24); uid[13] = uint8_t(n >> 16);
  uid[14] = uint8_t(n >> 8); uid[15] = uint8_t(n);
}
void announce(Node &sender, Node &receiver, uint32_t now) {
  for (unsigned part = 0; part < 3; ++part) receiver.receive(sender.announcement(part), now);
}

int main() {
  for (unsigned count = 0; count < COUNT; ++count) {
    unsigned greenStarts = 0;
    IdColor previous = LedOff;
    for (uint32_t now = 0; now < 10000; ++now) {
      IdColor color = idLed(count, now);
      if (color == LedGreen && previous != LedGreen) ++greenStarts;
      if (now < 300) assert(color == LedBlue);
      if (now >= 300 && now < 600) assert(color == LedOff);
      previous = color;
    }
    assert(greenStarts == count);
    assert(idLed(count, 10000) == LedBlue);
    assert(idLed(count, 9999) == LedOff);
  }
  Record r = {RECORD_MAGIC, FIRST_ID, uint16_t(~FIRST_ID)};
  assert(validRecord(r)); r.id ^= 1; assert(!validRecord(r));
  r = {RECORD_MAGIC, UNSET, 0}; assert(validRecord(r));
  r = {RECORD_MAGIC, 0x200, uint16_t(~0x200)}; assert(!validRecord(r));

  Button button;
  assert(button.update(true, 0) == 0);
  assert(button.update(false, 10) == 0); // Contact bounce must not be a press.
  assert(button.update(true, 20) == 0);
  assert(button.update(true, 45) == 0);
  assert(button.update(false, 60) == 0);
  assert(button.update(false, 85) == 0);
  assert(button.update(false, 546) == 1);
  assert(button.update(true, 600) == 0);
  assert(button.update(true, 625) == 0);
  assert(button.update(false, 650) == 0);
  assert(button.update(false, 675) == 0);
  assert(button.update(true, 800) == 0);
  assert(button.update(true, 825) == 2);
  assert(button.update(true, 2000) == 0); // Holding must not repeat.

  Node nodes[10]; uint8_t uid[16];
  for (unsigned i = 0; i < 10; ++i) {
    identity(uid, i + 1); nodes[i].begin(uid, UNSET, 0); nodes[i].request(0);
  }
  // All ten request an ID simultaneously. Exercise the real arbitration state machine.
  for (uint32_t now = 0; now <= 40000; now += 100) {
    for (auto &n : nodes) { n.tick(now); n.savePending = false; }
    for (unsigned i = 0; i < 10; ++i) for (unsigned j = 0; j < 10; ++j)
      if (i != j) announce(nodes[i], nodes[j], now);
  }
  uint32_t used = 0;
  for (auto &n : nodes) {
    assert(n.state == Assigned && validId(n.id));
    uint32_t bit = 1u << (n.id - FIRST_ID); assert(!(used & bit)); used |= bit;
  }
  assert(used == 0x3FF); // Exactly the ten smallest IDs, no gaps.

  Node rebooted;
  identity(uid, 1); rebooted.begin(uid, nodes[0].id, 50000);
  assert(rebooted.state == Assigned && rebooted.id == nodes[0].id);
  assert(!rebooted.dataReady(50000)); assert(rebooted.dataReady(53000));
  rebooted.reset(54000); assert(rebooted.state == Unset && rebooted.savePending && rebooted.id == UNSET);

  Node a, b;
  identity(uid, 100); a.begin(uid, FIRST_ID, 0);
  identity(uid, 200); b.begin(uid, FIRST_ID, 0);
  announce(a, b, 100); announce(b, a, 100);
  assert(a.error == DuplicateId && b.error == DuplicateId);
  assert(!a.dataReady(5000) && !b.dataReady(5000));

  Node established, joining;
  identity(uid, 7); established.begin(uid, FIRST_ID, 0);
  identity(uid, 8); joining.begin(uid, UNSET, 0); joining.request(0); joining.tick(1500);
  assert(joining.state == Claiming && joining.id == FIRST_ID);
  announce(established, joining, 1600);
  assert(joining.state == Scanning && established.state == Assigned);
  joining.tick(3100); assert(joining.id == FIRST_ID + 1);
  joining.tick(4600); assert(joining.state == Assigned && joining.savePending);

  // Incomplete/corrupt discovery must not reserve IDs or fabricate a collision.
  Node listener;
  identity(uid, 9); listener.begin(uid, UNSET, 0);
  listener.receive(established.announcement(0), 10);
  listener.request(10); listener.tick(1510);
  assert(listener.id == FIRST_ID);
  Frame bad = established.announcement(1); bad.data[2] ^= 1;
  listener.receive(bad, 1520); listener.receive(established.announcement(2), 1520);
  assert(listener.state == Claiming);

  Node existingTraffic;
  identity(uid, 10); existingTraffic.begin(uid, UNSET, 0);
  existingTraffic.observeData(FIRST_ID, 100);
  existingTraffic.request(100); existingTraffic.tick(1600);
  assert(existingTraffic.id == FIRST_ID + 1);
  existingTraffic.tick(3100); existingTraffic.savePending = false;
  existingTraffic.observeData(FIRST_ID + 1, 3200);
  assert(existingTraffic.error == DuplicateId);

  Node occupied[COUNT], newcomer;
  identity(uid, 1000); newcomer.begin(uid, UNSET, 0);
  for (unsigned i = 0; i < COUNT; ++i) {
    identity(uid, i + 500); occupied[i].begin(uid, FIRST_ID + i, 0);
    announce(occupied[i], newcomer, 100);
  }
  newcomer.request(100); newcomer.tick(1600);
  assert(newcomer.error == NoFreeId);
  newcomer.reset(4000); newcomer.savePending = false; newcomer.request(4000); newcomer.tick(5500);
  assert(newcomer.state == Claiming && newcomer.id == FIRST_ID); // Offline peers expire.

  // Time arithmetic also works across millis() wraparound.
  Node wrap; identity(uid, 4000); wrap.begin(uid, UNSET, 0xFFFFFF00u);
  wrap.request(0xFFFFFF00u); wrap.tick(uint32_t(0xFFFFFF00u + DISCOVER_MS));
  assert(wrap.state == Claiming);
  puts("PASS: button debounce/double press, persistence validation, ten-node simultaneous allocation,");
  puts("      minimum free ID, reboot/reset, duplicate-ID stop, full network, peer expiry, millis wrap.");
}

