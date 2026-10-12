#ifndef VOORBURG_FRAME_COUNTER_STORE_H
#define VOORBURG_FRAME_COUNTER_STORE_H

#include <Arduino.h>
#include <EEPROM.h>
#include "LoraSettings.h"

// Persist FCntUp in blocks to limit flash erases. A power loss can skip the
// unused tail of a block, but never reuses a counter already used on air.
class FrameCounterStore {
public:
  // Additional journal storage follows the counter slots in the same EEPROM
  // emulation buffer, so either commit preserves both sets of records.
  bool begin(size_t additionalBytes = 0) {
    EEPROM.begin(reservedBytes() + additionalBytes);
    uint32_t best = LORAWAN_INITIAL_FCNT_UP;
    bool found = false;
    uint8_t bestSlot = SLOT_COUNT - 1;
    for (uint8_t slot = 0; slot < SLOT_COUNT; ++slot) {
      Record record;
      EEPROM.get(slot * sizeof(Record), record);
      if (valid(record) && (!found || static_cast<int32_t>(record.nextCounter - best) > 0)) {
        best = record.nextCounter;
        bestSlot = slot;
        found = true;
      }
    }
    nextCounter = best;
    blockEnd = best;
    writeSlot = found ? (bestSlot + 1) % SLOT_COUNT : 0;
    if (!found && !reserveBlock()) return false;
    return true;
  }

  uint32_t reserve() {
    if (nextCounter >= blockEnd && !reserveBlock()) return UINT32_MAX;
    return nextCounter++;
  }

  static size_t reservedBytes() { return sizeof(Record) * SLOT_COUNT; }

private:
  struct Record { uint32_t magic; uint32_t nextCounter; uint16_t check; uint16_t pad; };
  static const uint32_t MAGIC = 0x56534247UL; // "VSBG"
  static const uint8_t SLOT_COUNT = 64;
  // At the default five-minute interval this commits at most once per ~21 h.
  static const uint32_t COUNTERS_PER_BLOCK = 256;
  uint32_t nextCounter = LORAWAN_INITIAL_FCNT_UP;
  uint32_t blockEnd = LORAWAN_INITIAL_FCNT_UP;
  uint8_t writeSlot = 0;

  bool reserveBlock() {
    if (blockEnd > UINT32_MAX - COUNTERS_PER_BLOCK) return false;
    Record record;
    record.magic = MAGIC;
    record.nextCounter = blockEnd + COUNTERS_PER_BLOCK;
    record.check = checksum(record.magic, record.nextCounter);
    record.pad = 0xFFFF;
    EEPROM.put(writeSlot * sizeof(Record), record);
    if (!EEPROM.commit()) return false;
    nextCounter = blockEnd;
    blockEnd = record.nextCounter;
    writeSlot = (writeSlot + 1) % SLOT_COUNT;
    return true;
  }

  static uint16_t checksum(uint32_t magic, uint32_t counter) {
    uint16_t crc = 0xFFFF;
    const uint32_t values[2] = {magic, counter};
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(values);
    for (uint8_t i = 0; i < sizeof(values); ++i) {
      crc ^= static_cast<uint16_t>(bytes[i]) << 8;
      for (uint8_t bit = 0; bit < 8; ++bit)
        crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
    }
    return crc;
  }

  static bool valid(const Record &record) {
    return record.magic == MAGIC && record.check == checksum(record.magic, record.nextCounter);
  }
};

#endif
