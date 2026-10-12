#include "PicoCrashDiagnostics.h"

#include <stddef.h>
#include <Arduino.h>

#if !PICO_RP2350 || defined(__riscv) || !defined(__ARM_ARCH_8M_MAIN__)
#error "PicoCrashDiagnostics requires an RP2350 Cortex-M33 build"
#endif

#include <hardware/address_mapped.h>
#include <hardware/exception.h>
#include <hardware/irq.h>
#include <hardware/regs/addressmap.h>
#include <hardware/regs/m33.h>
#include <hardware/regs/watchdog.h>
#include <hardware/structs/m33.h>
#include <hardware/structs/sio.h>
#include <hardware/structs/watchdog.h>
#include <hardware/sync.h>
#include <hardware/timer.h>
#include <hardware/watchdog.h>

// The installed SDK linker script retains .uninitialized_data as NOLOAD. A
// literal .noinit section is not mapped there by Arduino-Pico's linker script.
namespace {
constexpr uint32_t RECORD_MAGIC = 0x50434431u; // "PCD1"
constexpr uint32_t RECORD_VERSION = 1;
constexpr uint32_t CAPTURE_DELAY_US = 7000000u;
static_assert(offsetof(timer_hw_t, timerawl) == 0x28,
  "Raw timer IRQ requires the RP2350 TIMERAWL register offset");
static_assert(offsetof(timer_hw_t, intr) == 0x3c,
  "Raw timer IRQ requires the RP2350 INTR register offset");
constexpr uint32_t STACK_FAULTS =
  M33_CFSR_UFSR_STKOF_BITS | M33_CFSR_BFSR_LSPERR_BITS |
  M33_CFSR_BFSR_STKERR_BITS | M33_CFSR_BFSR_UNSTKERR_BITS |
  // The SDK exposes MMFSR as one byte: MLSPERR=5, MSTKERR=4, MUNSTKERR=3.
  0x38u;

struct RetainedRecord {
  uint32_t magic;
  uint32_t magicInverse;
  uint32_t version;
  uint32_t recordBytes;
  PicoCrashDiagnostics::Snapshot snapshot;
  uint32_t pcInverse;
  uint32_t lrInverse;
  uint32_t xpsrInverse;
  uint32_t crc;
  uint32_t crcInverse;
};
static volatile RetainedRecord retained
  __attribute__((section(".uninitialized_data.pico_crash"), aligned(4)));
static bool installed = false;
static bool faultInstalled = false;
static int alarmNumber = -1;
static uint alarmIrq = 0;
static volatile uint32_t currentOperation[2] = {};
// Preserve the first fault if the other core later reaches its timeout. A
// core1 HardFault can otherwise be overwritten by core0's watchdog capture.
static uint32_t captureOwner = 0;

// This function is safe during a flash erase/write, and uses no lookup table,
// library call, lock, allocation, or floating-point instruction.
static uint32_t __attribute__((section(".time_critical.pico_crash_crc"),
  noinline, target("general-regs-only"))) recordCrc() {
  const volatile uint8_t *bytes =
    reinterpret_cast<const volatile uint8_t *>(&retained);
  uint32_t crc = 0xffffffffu;
  for (size_t i = offsetof(RetainedRecord, version);
       i < offsetof(RetainedRecord, crc); ++i) {
    crc ^= bytes[i];
    for (uint32_t bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ ((0u - (crc & 1u)) & 0xedb88320u);
  }
  return ~crc;
}
}

// Symbols referenced by the naked wrappers must retain plain C names. A small
// dedicated handler stack allows capture even when the interrupted MSP is bad.
// Neither the interrupted stack nor any watchdog scratch register is modified.
extern "C" {
timer_hw_t *pico_crash_timer = nullptr;
volatile uint32_t pico_crash_last_progress = 0;
volatile uint32_t pico_crash_alarm_mask = 0;
alignas(8) uint32_t pico_crash_emergency_stack[256];
extern char __default_isrs_start;
extern char __default_isrs_end;

void __attribute__((noreturn, noinline, used,
  section(".time_critical.pico_crash_capture"),
  target("general-regs-only")))
pico_crash_capture(uint32_t *rawFrame, uint32_t excReturn, uint32_t kind) {
  using namespace PicoCrashDiagnostics;
  uint32_t unclaimed = 0;
  if (!__atomic_compare_exchange_n(&captureOwner, &unclaimed,
      sio_hw->cpuid + 1u, false, __ATOMIC_ACQ_REL, __ATOMIC_RELAXED)) {
    for (;;) __asm volatile("nop");
  }
  retained.magic = 0;
  retained.magicInverse = ~RECORD_MAGIC;
  retained.version = RECORD_VERSION;
  retained.recordBytes = sizeof(RetainedRecord);
  volatile Snapshot &s = retained.snapshot;
  s.kind = kind;
  s.flags = 0;
  s.pc = s.lr = s.xpsr = 0;
  s.excReturn = excReturn;
  s.stackPointer = reinterpret_cast<uintptr_t>(rawFrame);
  s.cfsr = m33_hw->cfsr;
  s.hfsr = m33_hw->hfsr;
  s.mmfar = m33_hw->mmfar;
  s.bfar = m33_hw->bfar;
  s.icsr = m33_hw->icsr;
  s.capturedAtMicros = pico_crash_timer ? pico_crash_timer->timerawl :
    timer_hw->timerawl;
  s.lastProgressMicros = pico_crash_last_progress;
  s.operation = currentOperation[sio_hw->cpuid];
  s.core = sio_hw->cpuid;
  if (excReturn & 4u) s.flags |= PROCESS_STACK;
  if (!(excReturn & 0x40u)) s.flags |= NONSECURE_FRAME;
  const bool fpFrame = !(excReturn & 0x10u);
  if (fpFrame) s.flags |= EXTENDED_FP_FRAME;

  // DCRS=0 describes an additional security transition frame. Arduino-Pico
  // runs this sketch without a TrustZone transition; reject such a frame
  // instead of reporting a guessed PC. Also reject corrupt EXC_RETURN values.
  const bool supported = (excReturn & 0xff000000u) == 0xff000000u &&
    (excReturn & 0x20u) != 0 && (excReturn & 2u) == 0;
  if (!supported) s.flags |= UNSUPPORTED_FRAME;
  const bool stackFault = (s.cfsr & STACK_FAULTS) != 0;
  if (stackFault) s.flags |= STACKING_ERROR;
  const uintptr_t rawAddress = reinterpret_cast<uintptr_t>(rawFrame);
  const size_t frameWords = fpFrame ? 26 : 8;
  const bool addressValid = (rawAddress & 3u) == 0 &&
    rawAddress >= SRAM_BASE &&
    rawAddress <= SRAM_END - frameWords * sizeof(uint32_t);
  if (!addressValid) s.flags |= INVALID_STACK_ADDRESS;

  if (supported && !stackFault && addressValid) {
    // Cortex-M33 keeps the core frame at raw SP in both layouts; the optional
    // 18-word FP area follows xPSR (Arm M33 Generic User Guide, Figure 2-6).
    // Read only LR, PC and xPSR, never memory dumps or application data.
    const volatile uint32_t *frame = rawFrame;
    const uint32_t xpsr = frame[7];
    if (xpsr & (1u << 24)) {
      s.lr = frame[5];
      s.pc = frame[6];
      s.xpsr = xpsr;
      s.flags |= FRAME_VALID;
    } else {
      s.flags |= UNSUPPORTED_FRAME;
    }
  }
  retained.pcInverse = ~s.pc;
  retained.lrInverse = ~s.lr;
  retained.xpsrInverse = ~s.xpsr;
  retained.crc = recordCrc();
  retained.crcInverse = ~retained.crc;
  __asm volatile("dsb" ::: "memory");
  // Publish the marker last. A reset during capture leaves an invalid record.
  retained.magic = RECORD_MAGIC;
  __asm volatile("dsb" ::: "memory");

  if (pico_crash_timer) {
    const uint32_t mask = pico_crash_alarm_mask;
    hw_clear_bits(&pico_crash_timer->inte, mask);
    pico_crash_timer->intr = mask;
    pico_crash_timer->armed = mask;
  }
  // Keep the real interrupted instruction suspended. Do not call logging,
  // flash, networking, watchdog_update(), or a software reboot from this IRQ.
  // The already-running hardware watchdog expires at its original deadline.
  for (;;) __asm volatile("nop");
}

// Capture the original exception stack before any compiler prologue or SDK
// callback. EXC_RETURN bit 2 selects MSP/PSP; bit 6 selects the security bank.
// Disable MSPLIM before switching to the dedicated RAM handler stack.
#define PICO_CRASH_SELECT_FRAME \
  "tst lr, #0x40\n" \
  "beq 2f\n" \
  "tst lr, #4\n" \
  "ite eq\n" \
  "mrseq r0, msp\n" \
  "mrsne r0, psp\n" \
  "b 3f\n" \
  "2:\n" \
  "tst lr, #4\n" \
  "ite eq\n" \
  "mrseq r0, msp_ns\n" \
  "mrsne r0, psp_ns\n" \
  "3:\n" \
  "mov r1, lr\n" \
  "cpsid i\n" \
  "movs r3, #0\n" \
  "msr msplim, r3\n" \
  "ldr r3, =pico_crash_emergency_stack\n" \
  "add.w r3, r3, #1024\n" \
  "msr msp, r3\n"

void __attribute__((naked, used, section(".time_critical.pico_crash_timer_irq")))
pico_crash_timer_irq() {
  __asm volatile(
    // An obsolete pending alarm must not stop a healthy loop. Test actual
    // elapsed time before changing stacks, then return normally if premature.
    "ldr r0, =pico_crash_timer\n"
    "ldr r0, [r0]\n"
    "ldr r1, [r0, #0x28]\n" // TIMERAWL
    "ldr r2, =pico_crash_last_progress\n"
    "ldr r2, [r2]\n"
    "subs r1, r1, r2\n"
    "ldr r2, =7000000\n"
    "cmp r1, r2\n"
    "bhs 1f\n"
    "ldr r1, =pico_crash_alarm_mask\n"
    "ldr r1, [r1]\n"
    "str r1, [r0, #0x3c]\n" // INTR, write one to clear this alarm only
    "bx lr\n"
    "1:\n"
    PICO_CRASH_SELECT_FRAME
    "movs r2, #1\n"
    "b pico_crash_capture\n"
  );
}

void __attribute__((naked, used, section(".time_critical.pico_crash_hardfault")))
pico_crash_hardfault() {
  __asm volatile(
    PICO_CRASH_SELECT_FRAME
    "cpsid f\n"
    "movs r2, #2\n"
    "b pico_crash_capture\n"
  );
}
#undef PICO_CRASH_SELECT_FRAME
}

namespace PicoCrashDiagnostics {

bool readPrevious(Snapshot &out, bool actualWatchdogReset) {
  out = {};
  const bool watchdogTimerReset = actualWatchdogReset &&
    (watchdog_hw->reason & WATCHDOG_REASON_TIMER_BITS) != 0 &&
    watchdog_caused_reboot();
  bool valid = watchdogTimerReset && retained.magic == RECORD_MAGIC &&
    retained.magicInverse == ~RECORD_MAGIC &&
    retained.version == RECORD_VERSION &&
    retained.recordBytes == sizeof(RetainedRecord) &&
    retained.pcInverse == ~retained.snapshot.pc &&
    retained.lrInverse == ~retained.snapshot.lr &&
    retained.xpsrInverse == ~retained.snapshot.xpsr &&
    retained.crcInverse == ~retained.crc && retained.crc == recordCrc() &&
    (retained.snapshot.kind == PREWATCHDOG || retained.snapshot.kind == HARDFAULT);
  if (valid) {
    uint8_t *destination = reinterpret_cast<uint8_t *>(&out);
    const volatile uint8_t *source =
      reinterpret_cast<const volatile uint8_t *>(&retained.snapshot);
    for (size_t i = 0; i < sizeof(Snapshot); ++i) destination[i] = source[i];
  }
  retained.magic = 0;
  __asm volatile("dsb" ::: "memory");
  return valid;
}

bool begin() {
  if (installed) return timerAvailable();
  installed = true;
  const uintptr_t current = reinterpret_cast<uintptr_t>(
    exception_get_vtable_handler(HARDFAULT_EXCEPTION));
  // Mirror the SDK's own default-handler check before invoking its exclusive
  // setter, which otherwise hard-asserts if another owner installed a handler.
  if (current >= reinterpret_cast<uintptr_t>(&__default_isrs_start) &&
      current < reinterpret_cast<uintptr_t>(&__default_isrs_end)) {
    exception_set_exclusive_handler(HARDFAULT_EXCEPTION, pico_crash_hardfault);
    faultInstalled = true;
  }

  // Prefer the second timer so the SDK's default alarm pool remains untouched.
  // Hold skipped cooperative claims until the search has progressed, then
  // release them, avoiding repeated claims of the same unsuitable alarm.
  for (int timerIndex = NUM_GENERIC_TIMERS - 1; timerIndex >= 0; --timerIndex) {
    timer_hw_t *candidate = timer_get_instance(static_cast<uint>(timerIndex));
    uint32_t rejected = 0;
    for (uint attempt = 0; attempt < NUM_ALARMS; ++attempt) {
      const int alarm = timer_hardware_alarm_claim_unused(candidate, false);
      if (alarm < 0) break;
      const uint irq = timer_hardware_alarm_get_irq_num(candidate, alarm);
      if (irq_has_handler(irq) || irq_is_enabled(irq)) {
        rejected |= 1u << alarm;
        continue;
      }
      pico_crash_timer = candidate;
      alarmNumber = alarm;
      alarmIrq = irq;
      pico_crash_alarm_mask = 1u << alarm;
      hw_clear_bits(&candidate->inte, pico_crash_alarm_mask);
      candidate->armed = pico_crash_alarm_mask;
      candidate->intr = pico_crash_alarm_mask;
      hw_clear_bits(&candidate->intf, pico_crash_alarm_mask);
      irq_set_exclusive_handler(irq, pico_crash_timer_irq);
      irq_set_priority(irq, PICO_HIGHEST_IRQ_PRIORITY);
      irq_clear(irq);
      irq_set_enabled(irq, true);
      break;
    }
    for (uint alarm = 0; alarm < NUM_ALARMS; ++alarm) {
      if (rejected & (1u << alarm)) timer_hardware_alarm_unclaim(candidate, alarm);
    }
    if (alarmNumber >= 0) break;
  }
  return timerAvailable();
}

void arm() {
  if (!timerAvailable()) return;
  const uint32_t interrupts = save_and_disable_interrupts();
  timer_hw_t *timer = pico_crash_timer;
  const uint32_t mask = pico_crash_alarm_mask;
  hw_clear_bits(&timer->inte, mask);
  timer->armed = mask;
  timer->intr = mask;
  hw_clear_bits(&timer->intf, mask);
  irq_clear(alarmIrq);
  pico_crash_last_progress = timer->timerawl;
  timer->alarm[alarmNumber] = pico_crash_last_progress + CAPTURE_DELAY_US;
  hw_set_bits(&timer->inte, mask);
  restore_interrupts(interrupts);
}

bool timerAvailable() { return alarmNumber >= 0; }
bool hardFaultAvailable() { return faultInstalled; }
void setOperation(uint32_t operation) { currentOperation[sio_hw->cpuid] = operation; }
}
