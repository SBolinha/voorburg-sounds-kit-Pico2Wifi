#!/usr/bin/env python3
"""Check actual sketch expiry/replacement logic without TTN or a USB device."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
sketch = (root / "Arduino/VoorburgSoundsKit/VoorburgSoundsKit.ino").read_text()
start = sketch.index("static void discardPendingMeasurement(")
end = sketch.index("static void serviceFlashWork()", start)
functions = sketch[start:end]
stub = r'''
#include <cassert>
#include <cstdint>
#include "MeasurementHandoff.h"
using __FlashStringHelper = char;
#define F(x) x
#define LOG_PRINT(...) do {} while (0)
#define LOG_PRINTLN(x) do { (void)(x); } while (0)
#define SEND_AFTER 300000UL
const uint64_t MEASUREMENT_PERIOD_US = 300000000ULL;
uint64_t nowUs = 1000000;
uint64_t time_us_64() { return nowUs; }
MeasurementHandoff::LatestMailbox measurementMailbox;
MeasurementHandoff::UtcClock measurementClock;
MeasurementHandoff::Measurement pendingMeasurement;
bool havePendingMeasurement = false, havePendingFrame = false;
bool restoredDeadlineAwaitingClock = false, pendingFrameAcknowledgesDownlink = false;
uint32_t pendingFrameLength = 0, nextUplinkAttemptAt = 0, uplinkRetryDelayMs = 0;
struct Batch { int count; } pendingAnswerBatch = {};
uint32_t retained = 0;
// MAC answers/ACK state remain independent of the prepared frame.
uint32_t unsentMacAnswers = 4; bool pendingDownlinkAck = true;
void retainDeviceState() { ++retained; }
'''
checks = r'''
MeasurementHandoff::Measurement make(uint32_t seq, uint64_t end) {
  MeasurementHandoff::Measurement m;
  m.sequence = seq; m.completedUs = end; m.expiresUs = end + 300000000ULL;
  m.setWindowEndUtc(1791532800u); return m;
}
int main() {
  assert(measurementClock.tryUpdate(nowUs, 1791532800u));
  assert(measurementMailbox.tryPublish(make(1, nowUs)));
  takeNewestMeasurement(); assert(havePendingMeasurement && pendingMeasurement.sequence == 1);
  havePendingFrame = true; pendingFrameLength = 31; pendingAnswerBatch.count = 3;
  assert(measurementMailbox.tryPublish(make(2, nowUs + 1)));
  takeNewestMeasurement();
  assert(havePendingMeasurement && pendingMeasurement.sequence == 2 && !havePendingFrame);
  assert(pendingFrameLength == 0 && pendingAnswerBatch.count == 0 && retained == 1);
  assert(unsentMacAnswers == 4 && pendingDownlinkAck);
  nowUs = pendingMeasurement.expiresUs;
  takeNewestMeasurement(); assert(!havePendingMeasurement && !havePendingFrame);
  assert(unsentMacAnswers == 4 && pendingDownlinkAck);

  // A late NTP anchor maps the stored window end, never delivery time.
  nowUs = 500000000;
  assert(measurementClock.tryUpdate(nowUs, 1791533000u));
  auto m = make(3, nowUs - 100000000); m.setWindowEndUtc(0);
  assert(measurementMailbox.tryPublish(m));
  takeNewestMeasurement();
  assert(havePendingMeasurement && pendingMeasurement.windowEndUtc() == 1791532900u);

  // A watchdog-restored frame must pass its wall-clock deadline before retry.
  pendingMeasurement = make(4, 0); havePendingMeasurement = havePendingFrame = true;
  restoredDeadlineAwaitingClock = true;
  pendingMeasurement.setWindowEndUtc(1791532800u);
  assert(measurementClock.tryUpdate(nowUs, 1791533100u));
  takeNewestMeasurement(); assert(!havePendingMeasurement && !havePendingFrame);
  pendingMeasurement = make(5, 0); havePendingMeasurement = havePendingFrame = true;
  restoredDeadlineAwaitingClock = true; pendingMeasurement.setWindowEndUtc(1791533000u);
  takeNewestMeasurement();
  assert(havePendingFrame && !restoredDeadlineAwaitingClock);
  assert(pendingMeasurement.expiresUs == nowUs + 200000000ULL);
}
'''
with tempfile.TemporaryDirectory(prefix="voorburg-measurement-pipeline-") as directory:
    source = Path(directory)/"check.cpp"; binary = Path(directory)/"check"
    source.write_text(stub + functions + checks)
    subprocess.run(["c++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
        "-I" + str(root / "Arduino/VoorburgSoundsKit"), str(source), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print("PASS: newest measurement replaces old frame; expiry preserves MAC/ACKs; restored frames and UTC end-time checked")
