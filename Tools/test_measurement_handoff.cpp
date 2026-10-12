// Offline checks for the exact bounded cross-core handoff module.
#include "../Arduino/VoorburgSoundsKit/MeasurementHandoff.h"
#include <cassert>
#include <cstdio>
#include <thread>
#include <atomic>
using namespace MeasurementHandoff;
static Measurement sample(uint32_t sequence, uint64_t end) {
  Measurement m;
  m.sequence = sequence; m.completedUs = end; m.expiresUs = end + 300000000ULL;
  for (unsigned i=0; i<5; ++i) m.applicationPayload[i] = static_cast<uint8_t>(sequence + i);
  m.setWindowEndUtc(1791532800u + sequence);
  return m;
}
int main() {
  LatestMailbox mailbox;
  Measurement out;
  assert(!mailbox.tryTakeLatest(out, 0));
  assert(mailbox.tryPublish(sample(1, 100)));
  assert(mailbox.tryPublish(sample(2, 200)));
  assert(mailbox.tryTakeLatest(out, 201) && out.sequence == 2);
  assert(out.windowEndUtc() == 1791532802u);
  assert(!mailbox.tryTakeLatest(out, 202));
  assert(mailbox.tryPublish(sample(3, 300)));
  assert(!mailbox.tryTakeLatest(out, 300000300ULL)); // At expiry, never stale.
  assert(out.sequence == 2);
  Measurement invalid; assert(!mailbox.tryPublish(invalid));

  UtcClock utc;
  uint32_t seconds = 77;
  assert(!utc.trySecondsAt(0, seconds) && seconds == 77);
  assert(!utc.tryUpdate(100, 100, 1000000));
  assert(utc.tryUpdate(1000000, 1791532800u, 750000));
  assert(utc.trySecondsAt(1250000, seconds) && seconds == 1791532801u);
  assert(utc.trySecondsAt(750000, seconds) && seconds == 1791532800u);
  assert(utc.trySecondsAt(0, seconds) && seconds == 1791532799u);
  assert(utc.tryUpdate(1000000, UINT32_MAX, 0));
  assert(!utc.trySecondsAt(2000000, seconds));
  assert(utc.tryUpdate(2000000, 0, 0));
  assert(!utc.trySecondsAt(1000000, seconds));

  FlashPause pause;
  assert(!pause.request(0)); assert(pause.request(1));
  assert(!pause.request(2) && !pause.acknowledged(1));
  assert(!pause.acknowledgeStopped(1, false) && pause.failed(1));
  assert(!pause.acknowledgeStopped(1, true)); // Failure requires a new attempt.
  assert(pause.release(1) && pause.resumeRequested(1));
  assert(pause.request(2));
  assert(!pause.acknowledgeStopped(1, true));
  assert(pause.acknowledgeStopped(2, true) && pause.acknowledged(2));
  assert(!pause.release(1) && !pause.resumeRequested(1));
  assert(pause.release(2) && pause.resumeRequested(2));

  CoreHeartbeat heartbeat;
  assert(!heartbeat.fresh(1, 3000000));
  heartbeat.beat(0xfffffff0ULL);
  assert(heartbeat.fresh(0x100000010ULL, 32));
  assert(!heartbeat.fresh(0x100000011ULL, 32));
  assert(!heartbeat.fresh(0x100000010ULL, 0x80000000u));

  // An occupied slot contains one complete snapshot, never a torn mix. The
  // producer retries CAS contention but never waits for network ownership.
  LatestMailbox concurrent;
  std::atomic<bool> done{false};
  constexpr uint32_t COUNT = 100000;
  std::thread producer([&] {
    for (uint32_t i=1; i<=COUNT; ++i) {
      Measurement m = sample(i, i);
      while (!concurrent.tryPublish(m)) std::this_thread::yield();
    }
    done.store(true, std::memory_order_release);
  });
  uint32_t previous = 0, received = 0;
  auto consume = [&] {
    if (!concurrent.tryTakeLatest(out, 0)) return false;
    assert(out.sequence > previous);
    assert(out.completedUs == out.sequence && out.expiresUs == out.completedUs + 300000000ULL);
    assert(out.windowEndUtc() == 1791532800u + out.sequence);
    for (unsigned i=0; i<5; ++i)
      assert(out.applicationPayload[i] == static_cast<uint8_t>(out.sequence + i));
    previous = out.sequence; ++received; return true;
  };
  while (!done.load(std::memory_order_acquire)) consume();
  for (uint32_t attempt=0; attempt<100000 && previous != COUNT; ++attempt)
    if (!consume()) std::this_thread::yield();
  producer.join();
  assert(received && previous == COUNT);
  std::puts("Measurement handoff: latest-only expiry, UTC mapping, flash generations, heartbeat wrap and concurrent snapshots passed");
}
