// Exercise per-window counter resets and preservation across a flash pause.
#include <cmath>
#include <climits>
#include <PicoI2SInput.h>
class Stream {};
static constexpr double PI = 3.14159265358979323846;
static void noInterrupts() { FakeAudio::irqState = 1; }
static void interrupts() { FakeAudio::irqState = 0; }
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wkeyword-macro"
#endif
#define private public
#include "../Arduino/VoorburgSoundsKit/SLM.h"
#undef private
#ifdef __clang__
#pragma clang diagnostic pop
#endif
#include <cstdio>
int main() {
  Stream output;
  static SLM meter(output);
  assert(meter.start());
  meter.sample_count = 3000;
  meter.frames_done = 0;
  const uint64_t previous = meter.frames_done;
  meter.update();
  assert(meter.frames_done > previous);
  assert(meter.statsDB.N() == 1 && meter.hasLevelMeasurement());
  assert(meter.pause());
  const uint64_t samples = meter.sample_count;
  assert(meter.start());
  assert(meter.sample_count == samples && meter.statsDB.N() == 1);
  meter.resetStatistics();
  assert(meter.statsDB.N() == 0 && meter.sample_count == samples);
  assert(meter.stop());
  assert(meter.sample_count == 0 && meter.frames_done == 0 && meter.write_index == 0);
  meter.resetStatistics();
  assert(meter.start());
  assert(meter.samplesRead() == 0 && meter.statsDB.N() == 0);
  assert(meter.stop());
  std::puts("SLM windows: flash pause preservation, independent counter/statistics resets and next-window restart passed");
}
