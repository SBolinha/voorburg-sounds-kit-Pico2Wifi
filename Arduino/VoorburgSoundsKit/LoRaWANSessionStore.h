#ifndef VOORBURG_LORAWAN_SESSION_STORE_H
#define VOORBURG_LORAWAN_SESSION_STORE_H

#include <EEPROM.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "LoRaWANMacState.h"

// Shares the EEPROM buffer with FrameCounterStore. Call EEPROM.begin once,
// with space for both stores, before using this journal.
class LoRaWANSessionStore {
public:
  struct State {
    LoRaWANMacState::Snapshot mac;
    uint32_t lastDownlinkCounter;
    uint32_t ackGeneration;
    uint8_t downlinkCounterSeen;
    uint8_t ackPending;
  };

  static size_t storageBytes() { return sizeof(Record) * SLOT_COUNT; }

  void begin(size_t offset, uint32_t sessionIdentity) {
    offset_ = offset;
    identity_ = sessionIdentity;
    sequence_ = 0;
    nextSlot_ = 0;
  }

  bool load(State &out) {
    bool found = false;
    Record best = {};
    for (uint8_t slot = 0; slot < SLOT_COUNT; ++slot) {
      Record record;
      EEPROM.get(offset_ + slot * sizeof(Record), record);
      if (valid(record) && (!found ||
          static_cast<int32_t>(record.sequence - best.sequence) > 0)) {
        best = record;
        nextSlot_ = static_cast<uint8_t>((slot + 1) % SLOT_COUNT);
        found = true;
      }
    }
    if (!found) return false;
    sequence_ = best.sequence;
    out = best.state;
    return true;
  }

  bool save(const State &state) {
    if (offset_ + storageBytes() > 4096 || !validState(state)) return false;
    Record record = {};
    record.magic = MAGIC;
    record.version = VERSION;
    record.size = sizeof(Record);
    record.identity = identity_;
    record.sequence = sequence_ + 1;
    record.state = state;
    record.checksum = crc(&record, offsetof(Record, checksum));
    const size_t address = offset_ + nextSlot_ * sizeof(Record);
    Record previous;
    EEPROM.get(address, previous);
    EEPROM.put(address, record);
    if (!EEPROM.commit()) {
      // Do not leave an uncommitted record for another EEPROM user to commit.
      EEPROM.put(address, previous);
      return false;
    }
    sequence_ = record.sequence;
    nextSlot_ = static_cast<uint8_t>((nextSlot_ + 1) % SLOT_COUNT);
    return true;
  }

  static bool validState(const State &state) {
    LoRaWANMacState check;
    return state.downlinkCounterSeen <= 1 && state.ackPending <= 1 &&
      check.restore(state.mac);
  }

  static uint32_t crc(const void *data, size_t length) {
    const uint8_t *bytes = static_cast<const uint8_t *>(data);
    uint32_t value = 0xFFFFFFFFUL;
    for (size_t i = 0; i < length; ++i) {
      value ^= bytes[i];
      for (uint8_t bit = 0; bit < 8; ++bit)
        value = (value & 1U) ? (value >> 1) ^ 0xEDB88320UL : value >> 1;
    }
    return ~value;
  }

private:
  struct Record {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t identity;
    uint32_t sequence;
    State state;
    uint32_t checksum;
  };
  static const uint32_t MAGIC = 0x56534D32UL; // "VSM2"
  static const uint16_t VERSION = 1;
  static const uint8_t SLOT_COUNT = 2;
  size_t offset_ = 0;
  uint32_t identity_ = 0;
  uint32_t sequence_ = 0;
  uint8_t nextSlot_ = 0;

  bool valid(const Record &record) const {
    return record.magic == MAGIC && record.version == VERSION &&
      record.size == sizeof(Record) && record.identity == identity_ &&
      record.checksum == crc(&record, offsetof(Record, checksum)) &&
      validState(record.state);
  }
};

#endif
