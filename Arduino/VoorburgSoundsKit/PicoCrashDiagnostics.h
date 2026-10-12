#ifndef VOORBURG_PICO_CRASH_DIAGNOSTICS_H
#define VOORBURG_PICO_CRASH_DIAGNOSTICS_H

#include <stdint.h>

// RP2350 Cortex-M33 diagnostics. This module never feeds or starts the watchdog.
// Call readPrevious() before begin(), begin() after enabling the 8-second WDT,
// and arm() immediately after each actual WDT feed. Only completed loop passes
// and explicit startup progress may feed/rearm supervision.
namespace PicoCrashDiagnostics {

enum Kind : uint32_t { PREWATCHDOG = 1, HARDFAULT = 2 };
enum Flags : uint32_t {
  FRAME_VALID = 1u << 0,
  EXTENDED_FP_FRAME = 1u << 1,
  PROCESS_STACK = 1u << 2,
  NONSECURE_FRAME = 1u << 3,
  STACKING_ERROR = 1u << 4,
  UNSUPPORTED_FRAME = 1u << 5,
  INVALID_STACK_ADDRESS = 1u << 6
};

struct Snapshot {
  uint32_t kind;
  uint32_t flags;
  uint32_t pc;
  uint32_t lr;
  uint32_t xpsr;
  uint32_t excReturn;
  uint32_t stackPointer;
  uint32_t cfsr;
  uint32_t hfsr;
  uint32_t mmfar;
  uint32_t bfar;
  uint32_t icsr;
  uint32_t capturedAtMicros;
  uint32_t lastProgressMicros;
  uint32_t operation;
  uint32_t core;
};

// Returns a validated retained snapshot only for a real watchdog timer reset.
// Always invalidates the retained record, including after cold or soft resets.
// The caller supplies its reset classification; hardware reason is also checked.
bool readPrevious(Snapshot &out, bool actualWatchdogReset);

// Claims a spare hardware alarm and installs exclusive raw exception handlers.
// Returns timer availability; HardFault installation has a separate status.
// Existing SDK/library alarm and exception handlers are never replaced.
bool begin();
void arm();
bool timerAvailable();
bool hardFaultAvailable();

// Optional annotation. Updating it does not feed the WDT or move the deadline.
void setOperation(uint32_t operation);

// An ordinary IRQ cannot capture a stall while all IRQs are masked, in an
// equal-priority IRQ that never yields, or in a higher-priority exception.
// Hardware WDT recovery still works. Stacking failures are recorded without
// dereferencing an invalid frame; FRAME_VALID must be checked before decoding.
}

#endif
