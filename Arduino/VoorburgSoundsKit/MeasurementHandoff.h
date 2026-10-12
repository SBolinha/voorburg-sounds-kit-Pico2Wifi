#ifndef VOORBURG_MEASUREMENT_HANDOFF_H
#define VOORBURG_MEASUREMENT_HANDOFF_H

#include <atomic>
#include <stdint.h>

// Fixed storage shared between the audio producer on core 1 and networking on
// core 0. No function here waits, allocates, logs, accesses flash, or calls time.
namespace MeasurementHandoff {

static_assert(std::atomic<uint32_t>::is_always_lock_free,
  "Measurement handoff requires lock-free 32-bit atomics");

struct Measurement {
  static constexpr uint32_t PAYLOAD_BYTES = 9;
  uint8_t applicationPayload[PAYLOAD_BYTES] = {};
  uint32_t sequence = 0;
  uint64_t completedUs = 0;
  uint64_t expiresUs = 0;

  bool expired(uint64_t nowUs) const { return nowUs >= expiresUs; }

  // Bytes 0..4 retain the existing five values. The window-end Unix timestamp
  // occupies bytes 5..8, least significant byte first; zero means not mapped.
  void setWindowEndUtc(uint32_t seconds) {
    applicationPayload[5] = static_cast<uint8_t>(seconds);
    applicationPayload[6] = static_cast<uint8_t>(seconds >> 8);
    applicationPayload[7] = static_cast<uint8_t>(seconds >> 16);
    applicationPayload[8] = static_cast<uint8_t>(seconds >> 24);
  }

  uint32_t windowEndUtc() const {
    return static_cast<uint32_t>(applicationPayload[5]) |
      (static_cast<uint32_t>(applicationPayload[6]) << 8) |
      (static_cast<uint32_t>(applicationPayload[7]) << 16) |
      (static_cast<uint32_t>(applicationPayload[8]) << 24);
  }
};

namespace detail {
class TryLock {
public:
  bool acquire() const {
    uint32_t available = 0;
    // Weak CAS makes exactly one attempt; a spurious failure is an ordinary
    // busy result. The caller retries from a later loop pass.
    return held.compare_exchange_weak(available, 1,
      std::memory_order_acquire, std::memory_order_relaxed);
  }
  void release() const { held.store(0, std::memory_order_release); }
private:
  mutable std::atomic<uint32_t> held{0};
};

inline void copyMeasurement(Measurement &destination, const Measurement &source) {
  for (uint32_t i = 0; i < Measurement::PAYLOAD_BYTES; ++i)
    destination.applicationPayload[i] = source.applicationPayload[i];
  destination.sequence = source.sequence;
  destination.completedUs = source.completedUs;
  destination.expiresUs = source.expiresUs;
}
}

class LatestMailbox {
public:
  // The producer keeps a busy snapshot locally and retries on its next pass.
  // A successful publish replaces any unsent older snapshot in the mailbox.
  // Sequence numbers are assigned monotonically by the single producer.
  bool tryPublish(const Measurement &measurement) {
    if (measurement.expiresUs <= measurement.completedUs || !lock.acquire())
      return false;
    detail::copyMeasurement(latest, measurement);
    occupied = true;
    lock.release();
    return true;
  }

  // Copy and release immediately. Never hold this lock while building a frame,
  // stopping audio, waiting for flash permission, or doing any network work.
  // The consumer must expire its own pending copy before every send, and take
  // the newest mailbox value before rebuilding/retrying a pending frame.
  bool tryTakeLatest(Measurement &out, uint64_t nowUs) {
    if (!lock.acquire()) return false;
    const bool available = occupied && !latest.expired(nowUs);
    if (available) detail::copyMeasurement(out, latest);
    occupied = false; // Also discard an expired value.
    lock.release();
    return available;
  }

private:
  detail::TryLock lock;
  Measurement latest;
  bool occupied = false;
};

class UtcClock {
public:
  // Core 0 publishes an NTP-confirmed anchor sampled against time_us_64().
  // fractionUs is the fractional part of that same UTC sample, not its age.
  // Do not publish an unsynchronized clock. False means invalid/busy: retry
  // from the next core-0 pass. No SDK or libc clock is read by this class.
  bool tryUpdate(uint64_t monotonicUs, uint32_t utcSeconds,
                 uint32_t fractionUs = 0) {
    if (fractionUs >= 1000000u || !lock.acquire()) return false;
    anchorMonotonicUs = monotonicUs;
    anchorSeconds = utcSeconds;
    anchorFractionUs = fractionUs;
    synchronized = true;
    lock.release();
    return true;
  }

  // Map the actual measurement completion, including times before the latest
  // NTP anchor. Copy the anchor under the try-lock; do arithmetic after release.
  // False leaves out unchanged (busy, unsynchronized, or outside Unix-u32).
  bool trySecondsAt(uint64_t completedUs, uint32_t &out) const {
    if (!lock.acquire()) return false;
    const bool known = synchronized;
    const uint64_t mono = anchorMonotonicUs;
    const uint32_t seconds = anchorSeconds;
    const uint32_t fraction = anchorFractionUs;
    lock.release();
    if (!known) return false;

    if (completedUs >= mono) {
      const uint64_t delta = completedUs - mono;
      const uint64_t whole = delta / 1000000u;
      const uint32_t part = static_cast<uint32_t>(delta % 1000000u);
      const uint64_t result = static_cast<uint64_t>(seconds) + whole +
        (fraction + part >= 1000000u ? 1u : 0u);
      if (result > UINT32_MAX) return false;
      out = static_cast<uint32_t>(result);
    } else {
      const uint64_t delta = mono - completedUs;
      const uint64_t whole = delta / 1000000u;
      const uint32_t part = static_cast<uint32_t>(delta % 1000000u);
      const uint64_t subtract = whole + (part > fraction ? 1u : 0u);
      if (subtract > seconds) return false;
      out = seconds - static_cast<uint32_t>(subtract);
    }
    return true;
  }

private:
  detail::TryLock lock;
  uint64_t anchorMonotonicUs = 0;
  uint32_t anchorSeconds = 0;
  uint32_t anchorFractionUs = 0;
  bool synchronized = false;
};

class FlashPause {
public:
  // Core 0 owns request/release and increments the nonzero generation for each
  // attempt. Generations must not be reused during this boot. There may be only
  // one outstanding request; false means invalid generation or already busy.
  bool request(uint32_t generation) {
    if (!generation) return false;
    uint32_t empty = 0;
    return requested.compare_exchange_strong(empty, generation,
      std::memory_order_release, std::memory_order_relaxed);
  }

  bool acknowledged(uint32_t generation) const {
    return generation && requested.load(std::memory_order_acquire) == generation &&
      stopped.load(std::memory_order_acquire) == generation &&
      failedStop.load(std::memory_order_acquire) != generation;
  }

  bool failed(uint32_t generation) const {
    return generation && requested.load(std::memory_order_acquire) == generation &&
      failedStop.load(std::memory_order_acquire) == generation;
  }

  // Release after flash completes, or cancel after timeout/stop failure. A
  // stale release cannot cancel a newer request. Polling/timeout stays in core 0.
  bool release(uint32_t generation) {
    if (!generation) return false;
    return requested.compare_exchange_strong(generation, 0,
      std::memory_order_acq_rel, std::memory_order_relaxed);
  }

  uint32_t requestedGeneration() const {
    return requested.load(std::memory_order_acquire);
  }

  // Core 1 calls this only after stop() has returned: true must mean DMA and
  // PIO are stopped, driver work is quiescent, and interrupts remain enabled.
  // Failure is latched for this generation; core 0 must release and retry with
  // a new generation. Neither this method nor request() invokes the driver.
  bool acknowledgeStopped(uint32_t generation, bool stopSucceeded) {
    if (!generation || requested.load(std::memory_order_acquire) != generation)
      return false;
    if (!stopSucceeded) {
      failedStop.store(generation, std::memory_order_release);
      return false;
    }
    if (failedStop.load(std::memory_order_acquire) == generation) return false;
    stopped.store(generation, std::memory_order_release);
    return requested.load(std::memory_order_acquire) == generation;
  }

  // Core 1 may resume only when all outstanding requests have been released.
  // If core 0 has already requested another generation, stay stopped and ACK
  // that generation with the known stopped state instead of resuming DMA.
  bool resumeRequested(uint32_t pausedGeneration) const {
    return pausedGeneration && requested.load(std::memory_order_acquire) == 0;
  }

private:
  std::atomic<uint32_t> requested{0};
  std::atomic<uint32_t> stopped{0};
  std::atomic<uint32_t> failedStop{0};
};

class CoreHeartbeat {
public:
  // Core 1 publishes once per completed audio-loop pass, including flash-pause
  // polling. The low timestamp word suffices for deadlines shorter than 2^31us.
  void beat(uint64_t nowUs) {
    lastBeatUs.store(static_cast<uint32_t>(nowUs), std::memory_order_release);
    seen.store(1, std::memory_order_release);
  }
  bool started() const { return seen.load(std::memory_order_acquire) != 0; }
  bool fresh(uint64_t nowUs, uint32_t maxAgeUs) const {
    if (!started() || maxAgeUs >= 0x80000000u) return false;
    const uint32_t last = lastBeatUs.load(std::memory_order_acquire);
    // Core 1 may publish a newer beat after core 0 samples nowUs. Treat
    // that beat as fresh instead of unsigned subtraction wrapping to an
    // apparent age of roughly 71 minutes. Deadlines stay below 2^31 us.
    const int32_t age = static_cast<int32_t>(static_cast<uint32_t>(nowUs) - last);
    return age <= 0 || static_cast<uint32_t>(age) <= maxAgeUs;
  }
private:
  std::atomic<uint32_t> lastBeatUs{0};
  std::atomic<uint32_t> seen{0};
};
}

#endif
