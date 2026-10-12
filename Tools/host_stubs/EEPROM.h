#ifndef VOORBURG_TEST_EEPROM_H
#define VOORBURG_TEST_EEPROM_H
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Host-only EEPROM model. It is not part of the Arduino sketch include path.
struct HostEEPROM {
  uint8_t working[4096];
  uint8_t flash[4096];
  size_t size;
  unsigned commits;
  bool failCommit;
  HostEEPROM() : size(0), commits(0), failCommit(false) {
    memset(working, 0xFF, sizeof(working));
    memset(flash, 0xFF, sizeof(flash));
  }
  void begin(size_t bytes) {
    assert(bytes <= sizeof(working));
    size = bytes;
    memcpy(working, flash, sizeof(working));
  }
  template<class T> void get(size_t offset, T &out) const {
    assert(offset + sizeof(T) <= size);
    memcpy(&out, working + offset, sizeof(T));
  }
  template<class T> void put(size_t offset, const T &value) {
    assert(offset + sizeof(T) <= size);
    memcpy(working + offset, &value, sizeof(T));
  }
  bool commit() {
    if (failCommit) return false;
    memcpy(flash, working, sizeof(flash));
    ++commits;
    return true;
  }
};
extern HostEEPROM EEPROM;
#endif
