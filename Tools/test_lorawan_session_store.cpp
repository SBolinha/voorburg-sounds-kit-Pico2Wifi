// c++ -std=c++11 -Wall -Wextra -Werror -pedantic -ITools/host_stubs 
//   Tools/test_lorawan_session_store.cpp -o /tmp/voorburg-session-test
#include <assert.h>
#include <stdio.h>
#include "../Arduino/VoorburgSoundsKit/LoRaWANSessionStore.h"

HostEEPROM EEPROM;

static LoRaWANSessionStore::State protocolState() {
  LoRaWANMacState mac;
  const uint8_t commands[] = {
    0x05, 0x03, 0xD2, 0xAD, 0x84,
    0x07, 0x03, 0x18, 0x4F, 0x84, 0x50,
    0x08, 0x05, 0x06
  };
  assert(mac.processCommands(commands, sizeof(commands), 0, 29));
  LoRaWANSessionStore::State out = {};
  out.mac = mac.snapshot();
  out.lastDownlinkCounter = 7;
  out.downlinkCounterSeen = 1;
  out.ackGeneration = 3;
  out.ackPending = 1;
  return out;
}

int main() {
  const size_t counterBytes = 768;
  const size_t size = counterBytes + LoRaWANSessionStore::storageBytes();
  assert(size <= 4096);
  EEPROM.begin(size);
  const uint32_t counterMarker = 0xAABBCCDD;
  EEPROM.put(0, counterMarker);
  assert(EEPROM.commit());
  LoRaWANSessionStore journal;
  journal.begin(counterBytes, 1234);
  LoRaWANSessionStore::State loaded = {};
  assert(!journal.load(loaded));
  const LoRaWANSessionStore::State first = protocolState();
  assert(journal.save(first));

  EEPROM.begin(size); // Cold restart; retained RAM is unavailable.
  LoRaWANSessionStore restarted;
  restarted.begin(counterBytes, 1234);
  assert(restarted.load(loaded));
  assert(loaded.lastDownlinkCounter == 7 && loaded.ackPending == 1);
  LoRaWANMacState mac;
  assert(mac.restore(loaded.mac));
  assert(mac.rxDelaySeconds() == 5 && mac.rx2DataRate() == 3);
  assert(mac.channel(3).frequencyHz == 867100000UL);
  uint8_t bytes[15];
  LoRaWANMacState::AnswerBatch batch = {};
  assert(mac.prepareAnswers(bytes, batch) == 8);
  mac.answersSubmitted(batch);
  LoRaWANSessionStore::State second = loaded;
  second.mac = mac.snapshot();
  second.lastDownlinkCounter = 8;
  second.ackPending = 0;
  assert(restarted.save(second));

  EEPROM.begin(size);
  restarted.begin(counterBytes, 1234);
  assert(restarted.load(loaded) && loaded.lastDownlinkCounter == 8);
  assert(mac.restore(loaded.mac));
  assert(mac.prepareAnswers(bytes, batch) == 3); // Required sticky repetitions.
  assert(bytes[0] == 0x05 && bytes[1] == 0x07 && bytes[2] == 0x08);
  uint32_t marker = 0;
  EEPROM.get(0, marker);
  assert(marker == counterMarker); // Journal commits preserve FCntUp slots.

  // A failed save is rolled back in the shared buffer, including if another
  // EEPROM user subsequently commits successfully.
  second.lastDownlinkCounter = 9;
  EEPROM.failCommit = true;
  assert(!restarted.save(second));
  EEPROM.failCommit = false;
  assert(EEPROM.commit());
  EEPROM.begin(size);
  restarted.begin(counterBytes, 1234);
  assert(restarted.load(loaded) && loaded.lastDownlinkCounter == 8);

  // A changed ABP session must not reuse the previous session's counters.
  restarted.begin(counterBytes, 5678);
  assert(!restarted.load(loaded));

  // Corrupt the latest record and recover the previous valid slot.
  const size_t slotSize = LoRaWANSessionStore::storageBytes() / 2;
  EEPROM.flash[counterBytes + slotSize + 24] ^= 1;
  EEPROM.begin(size);
  restarted.begin(counterBytes, 1234);
  assert(restarted.load(loaded) && loaded.lastDownlinkCounter == 7);
  loaded.ackPending = 2;
  assert(!restarted.save(loaded));
  printf("LoRaWAN session journal: cold recovery, answers, counters, corruption and commit failure passed (%zu EEPROM bytes)\n", size);
}
