#!/usr/bin/env python3
"""Exercise the actual core0 flash coordinator; no hardware or TTN traffic."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
sketch = (root / "Arduino/VoorburgSoundsKit/VoorburgSoundsKit.ino").read_text()
def function_between(start, end):
    begin = sketch.index(start)
    return sketch[begin:sketch.index(end, begin)]
functions = function_between("static void saveProtocolStateWhenDue() {", "static void printBootDiagnostics()")
functions += function_between("static void serviceFlashWork() {", "void setup() {")
stub = r'''
#include <cassert>
#include <cstdint>
#include <string>
#include "MeasurementHandoff.h"
#define TTN_UPLINK_ENABLED 1
#define USB_PACKET_DUMP 1
#define F(x) x
#define LOG_PRINTLN(x) do { (void)(x); } while (0)
uint32_t now = 100;
uint32_t millis() { return now; }
uint64_t time_us_64() { return static_cast<uint64_t>(now) * 1000; }
bool protocolStorageReady = true, protocolFlashDirty = true;
uint32_t nextProtocolSaveAt = 0, nextUplinkAttemptAt = 0;
bool havePendingMeasurement = false, havePendingFrame = false;
MeasurementHandoff::Measurement pendingMeasurement;
MeasurementHandoff::FlashPause flashPause;
enum FlashWork { FLASH_NONE, FLASH_FRAME, FLASH_PROTOCOL };
FlashWork flashWork = FLASH_NONE;
uint32_t flashGeneration = 0, activeFlashGeneration = 0, flashDeadlineAt = 0;
bool audioActive = true, storeSucceeds = true;
std::string calls;
uint32_t builtSequence = 0;
struct TTNSemtechUDPClient {
  static void markProtocolStorageOperation() {}
  static void clearOperationMarker() {}
};
struct Store {
  bool save(int) {
    assert(!audioActive && flashPause.acknowledged(activeFlashGeneration));
    calls += "save ";
    return storeSucceeds;
  }
} sessionStore;
int currentProtocolState() { return 0; }
bool prepareUplink() {
  assert(!audioActive && flashPause.acknowledged(activeFlashGeneration));
  calls += "frame ";
  builtSequence = pendingMeasurement.sequence;
  havePendingFrame = true;
  return true;
}
'''
checks = r'''
void reset() {
  if (flashPause.requestedGeneration()) flashPause.release(flashPause.requestedGeneration());
  now = 100; protocolStorageReady = protocolFlashDirty = true;
  nextProtocolSaveAt = nextUplinkAttemptAt = 0;
  havePendingFrame = havePendingMeasurement = false;
  flashWork = FLASH_NONE; audioActive = storeSucceeds = true;
  calls.clear(); builtSequence = 0;
}
void audioAck(bool success = true) {
  if (success) audioActive = false;
  flashPause.acknowledgeStopped(activeFlashGeneration, success);
}
int main() {
  reset(); serviceFlashWork();
  assert(flashWork == FLASH_PROTOCOL && calls.empty() && audioActive);
  serviceFlashWork(); assert(calls.empty()); // No flash until core1 ACK.
  audioAck(); serviceFlashWork();
  assert(calls == "save " && !protocolFlashDirty && flashWork == FLASH_NONE);
  assert(!flashPause.requestedGeneration());

  reset(); serviceFlashWork(); audioAck(false); serviceFlashWork();
  assert(calls.empty() && protocolFlashDirty && !flashPause.requestedGeneration());
  assert(nextProtocolSaveAt == 1100 && nextUplinkAttemptAt == 1100);

  reset(); serviceFlashWork(); now = 1200; serviceFlashWork();
  assert(calls.empty() && !flashPause.requestedGeneration());

  reset(); storeSucceeds = false; serviceFlashWork(); audioAck(); serviceFlashWork();
  assert(calls == "save " && protocolFlashDirty && nextProtocolSaveAt == 30100);
  assert(!flashPause.requestedGeneration());

  reset(); havePendingMeasurement = true;
  pendingMeasurement = {}; pendingMeasurement.sequence = 1;
  pendingMeasurement.completedUs = 1; pendingMeasurement.expiresUs = 1000000;
  pendingMeasurement.setWindowEndUtc(1791532800);
  serviceFlashWork(); assert(flashWork == FLASH_FRAME && calls.empty());
  // A newer payload can replace it while the coordinator waits for ACK.
  pendingMeasurement.sequence = 2;
  audioAck(); serviceFlashWork();
  assert(calls == "frame " && builtSequence == 2 && havePendingFrame);

  reset(); havePendingMeasurement = true;
  pendingMeasurement.expiresUs = 200000; pendingMeasurement.setWindowEndUtc(1791532800);
  serviceFlashWork(); audioAck(); now = 250; serviceFlashWork();
  assert(calls.empty() && !havePendingFrame && !flashPause.requestedGeneration());

  reset(); protocolStorageReady = false; serviceFlashWork(); assert(calls.empty() && flashWork == FLASH_NONE);
  reset(); protocolFlashDirty = false; serviceFlashWork(); assert(calls.empty() && flashWork == FLASH_NONE);
  reset(); nextProtocolSaveAt = 1000; serviceFlashWork(); assert(calls.empty() && flashWork == FLASH_NONE);
}
'''
with tempfile.TemporaryDirectory(prefix="voorburg-flash-audio-") as directory:
    source = Path(directory) / "check.cpp"
    binary = Path(directory) / "check"
    source.write_text(stub + functions + checks)
    subprocess.run(["c++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
        "-I" + str(root / "Arduino/VoorburgSoundsKit"), str(source), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print("PASS: flash waits for core1 stopped ACK; failure/timeout/expiry defer safely; newest payload wins")
