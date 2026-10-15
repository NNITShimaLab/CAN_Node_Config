// SPDX-License-Identifier: MIT
// Copyright (c) 2026 NNITShimaLab contributors
#pragma once
#include <stdint.h>
#include <string.h>

namespace nodecfg {
constexpr uint16_t FIRST_ID = 0x100, COUNT = 16, UNSET = 0xFFFF;
constexpr uint32_t PEER_TIMEOUT = 3000, DISCOVER_MS = 1500, CLAIM_MS = 1500;
enum State : uint8_t { Unset, Scanning, Claiming, Assigned, Error };
enum ErrorCode : uint8_t { NoError, DuplicateId, CanFault, StorageFault, NoFreeId, DiscoveryCollision, TooManyPeers };
struct Frame { uint32_t id; uint8_t data[8]; };
struct Record { uint32_t magic; uint16_t id, inverse; };
constexpr uint32_t RECORD_MAGIC = 0x58434E01; // XCN, format version 1.
enum IdColor : uint8_t { LedOff, LedBlue, LedGreen };
constexpr IdColor idLed(unsigned count, uint32_t now) {
  unsigned phase = now % 10000;
  if (phase < 300) return LedBlue;
  if (phase < 600) return LedOff;
  phase -= 600;
  return phase < count * 500 && phase % 500 < 200 ? LedGreen : LedOff;
}
inline bool validId(uint16_t id) { return id >= FIRST_ID && id < FIRST_ID + COUNT; }
inline bool validRecord(const Record &r) {
  return r.magic == RECORD_MAGIC && uint16_t(~r.id) == r.inverse && (r.id == UNSET || validId(r.id));
}
inline uint32_t uidTag(const uint8_t *uid) {
  // shortcut: 25-bit discovery tags may collide; use stronger discovery addressing if guaranteed uniqueness is needed.
  uint32_t hash = 2166136261u;
  for (unsigned i = 0; i < 16; ++i) hash = (hash ^ uid[i]) * 16777619u;
  return hash & 0x01FFFFFFu;
}
inline bool owner(State state) { return state == Assigned || state == Error; }

class Button {
  bool raw = false, stable = false, waiting = false;
  uint32_t changedAt = 0, firstAt = 0;
public:
  // 0=no event, 1=single press, 2=double press. Input true means pressed.
  unsigned update(bool pressed, uint32_t now) {
    if (pressed != raw) { raw = pressed; changedAt = now; }
    if (stable != raw && uint32_t(now - changedAt) >= 25) {
      stable = raw;
      if (stable) {
        if (waiting && uint32_t(now - firstAt) <= 500) { waiting = false; return 2; }
        waiting = true; firstAt = now;
      }
    }
    if (waiting && uint32_t(now - firstAt) > 500) { waiting = false; return 1; }
    return 0;
  }
};

class Node {
  struct Peer {
    bool used = false, complete = false;
    uint32_t tag = 0, last = 0, assemblyAt = 0;
    uint16_t id = UNSET, assemblingId = UNSET;
    State state = Unset, assemblingState = Unset;
    uint8_t uid[16] = {}, mask = 0;
  } peers[COUNT];
  uint32_t dataLast[COUNT] = {}, dataSeen = 0;
  uint32_t phaseAt = 0, bornAt = 0;
  uint16_t firstFree(uint32_t now) const {
    for (uint16_t candidate = FIRST_ID; candidate < FIRST_ID + COUNT; ++candidate) {
      unsigned offset = candidate - FIRST_ID;
      bool busy = (dataSeen & (1u << offset)) && uint32_t(now - dataLast[offset]) < PEER_TIMEOUT;
      for (const auto &p : peers)
        if (p.complete && uint32_t(now - p.last) < PEER_TIMEOUT && p.id == candidate &&
            (owner(p.state) || p.state == Claiming)) busy = true;
      if (!busy) return candidate;
    }
    return UNSET;
  }
public:
  uint8_t uid[16] = {};
  uint32_t tag = 0;
  State state = Unset;
  uint16_t id = UNSET;
  ErrorCode error = NoError;
  bool savePending = false;
  void begin(const uint8_t *identity, uint16_t saved, uint32_t now) {
    memcpy(uid, identity, 16); tag = uidTag(uid); bornAt = phaseAt = now;
    id = validId(saved) ? saved : UNSET; state = validId(saved) ? Assigned : Unset;
  }
  void fail(ErrorCode code) { if (state != Error) { error = code; state = Error; } }
  void reset(uint32_t now) {
    id = UNSET; error = NoError; state = Unset; phaseAt = now; savePending = true;
  }
  void request(uint32_t now) {
    if (state == Unset) { state = Scanning; phaseAt = now; }
  }
  void observeData(uint16_t canId, uint32_t now) {
    if (!validId(canId)) return;
    unsigned offset = canId - FIRST_ID;
    dataSeen |= 1u << offset; dataLast[offset] = now;
    // Normal CAN mode does not echo our own transmitted frames into the receive queue.
    if (id == canId && state == Assigned) fail(DuplicateId);
    else if (id == canId && state == Claiming) { id = UNSET; state = Scanning; phaseAt = now; }
  }
  void tick(uint32_t now) {
    if (state == Scanning && uint32_t(now - phaseAt) >= DISCOVER_MS) {
      id = firstFree(now);
      if (id == UNSET) fail(NoFreeId);
      else { state = Claiming; phaseAt = now; }
    } else if (state == Claiming && uint32_t(now - phaseAt) >= CLAIM_MS) {
      state = Assigned; savePending = true;
    }
  }
  bool dataReady(uint32_t now) const {
    return state == Assigned && !savePending && uint32_t(now - bornAt) >= PEER_TIMEOUT;
  }
  Frame announcement(unsigned part) const {
    Frame f = {0x10000000u | (tag << 2) | part, {}};
    f.data[0] = 0x10 | state;
    f.data[1] = validId(id) ? uint8_t(id - FIRST_ID) : 0xFF;
    for (unsigned j = 0; j < 6 && part * 6 + j < 16; ++j) f.data[2 + j] = uid[part * 6 + j];
    return f;
  }
  void receive(const Frame &f, uint32_t now) {
    if ((f.id & 0x18000000u) != 0x10000000u || (f.id & 3) > 2 ||
        (f.data[0] & 0xF0) != 0x10 || (f.data[0] & 0x0F) > Error ||
        (f.data[1] >= COUNT && f.data[1] != 0xFF)) return;
    State remoteState = State(f.data[0] & 0x0F);
    uint16_t remoteId = f.data[1] == 0xFF ? UNSET : FIRST_ID + f.data[1];
    if ((remoteState == Assigned || remoteState == Claiming) && remoteId == UNSET) return;
    uint32_t remoteTag = (f.id >> 2) & 0x01FFFFFFu;
    Peer *p = nullptr;
    for (auto &entry : peers) if (entry.used && entry.tag == remoteTag) { p = &entry; break; }
    if (!p) for (auto &entry : peers)
      if (!entry.used || uint32_t(now - entry.last) >= PEER_TIMEOUT) { p = &entry; *p = Peer{}; p->used = true; p->tag = remoteTag; break; }
    if (!p) { fail(TooManyPeers); return; }
    if (p->assemblingId != remoteId || p->assemblingState != remoteState || uint32_t(now - p->assemblyAt) > 1000) {
      p->mask = 0; p->assemblingId = remoteId; p->assemblingState = remoteState;
    }
    if (!p->mask) p->assemblyAt = now;
    unsigned part = f.id & 3;
    for (unsigned j = 0; j < 6 && part * 6 + j < 16; ++j) p->uid[part * 6 + j] = f.data[2 + j];
    p->mask |= 1u << part; p->last = now;
    if (p->mask != 7) return;
    p->mask = 0;
    if (uidTag(p->uid) != remoteTag) return;
    if (memcmp(uid, p->uid, 16) == 0) { p->used = p->complete = false; return; }
    if (remoteTag == tag) { fail(DiscoveryCollision); return; }
    p->id = remoteId; p->state = remoteState; p->complete = true;
    if (id == UNSET || id != remoteId) return;
    if (state == Assigned && owner(remoteState)) { fail(DuplicateId); return; }
    if (state == Claiming && (owner(remoteState) ||
        (remoteState == Claiming && memcmp(p->uid, uid, 16) < 0))) {
      id = UNSET; state = Scanning; phaseAt = now;
    }
  }
};
}

