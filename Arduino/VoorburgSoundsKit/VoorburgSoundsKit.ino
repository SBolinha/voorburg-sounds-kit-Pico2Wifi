/*
 * Voorburg Sounds Kit: SPH0645 sound meter with a software LoRaWAN endpoint.
 * Uses the Semtech UDP GWMP transport for TTN.
 */
#include <Arduino.h>
#include "EventLog.h"
#include "SLM.h"
#include "LoraSettings.h"
#include "LoRaWANFrame.h"
#include "PicoCrashDiagnostics.h"
#include "MeasurementHandoff.h"
#include <hardware/sync.h>
#include <pico/time.h>
#include <sys/time.h>
#include <atomic>
#include "FrameCounterStore.h"
#include "LoRaWANSessionStore.h"
#include "TTNSemtechUDPClient.h"

EventLog eventLog;

#ifndef USB_LOGGING
#define USB_LOGGING 1
#endif
#ifndef USB_MIC_LOGGING
#define USB_MIC_LOGGING 0
#endif
#ifndef TTN_UPLINK_ENABLED
#define TTN_UPLINK_ENABLED 1
#endif
#ifndef SEND_AFTER
#define SEND_AFTER 300000UL
#endif
#ifndef MIC_SAMPLE_STALL_MS
#define MIC_SAMPLE_STALL_MS 15000UL
#endif
#ifndef TTN_DEVICE_STATUS_MARGIN_DB
// 95% of the LoRaWAN DevStatusAns maximum (+31 dB), rounded to 29 dB.
#define TTN_DEVICE_STATUS_MARGIN_DB 29
#endif
#ifndef TTN_DEVICE_STATUS_BATTERY
// LoRaWAN defines battery=0 as externally powered; this Pico has no battery gauge.
#define TTN_DEVICE_STATUS_BATTERY 0
#endif
#if TTN_DEVICE_STATUS_BATTERY < 0 || TTN_DEVICE_STATUS_BATTERY > 255
#error TTN_DEVICE_STATUS_BATTERY must be 0 (external), 1..254, or 255 (unknown)
#endif
#if TTN_DEVICE_STATUS_MARGIN_DB < -32 || TTN_DEVICE_STATUS_MARGIN_DB > 31
#error TTN_DEVICE_STATUS_MARGIN_DB must be in the LoRaWAN range -32..31 dB
#endif

#if USB_LOGGING
#define LOG_BEGIN() do { Serial.begin(115200); } while (0)
#define LOG_PRINT(...) eventLog.print(__VA_ARGS__)
#define LOG_PRINTLN(...) eventLog.println(__VA_ARGS__)
#else
#define LOG_BEGIN() do {} while (0)
#define LOG_PRINT(...) do {} while (0)
#define LOG_PRINTLN(...) do {} while (0)
#endif

// Arduino-Pico otherwise splits the default stack into 4 KiB per core.
bool core1_separate_stack = true;
SLM slm(Serial);
static MeasurementHandoff::LatestMailbox measurementMailbox;
static MeasurementHandoff::UtcClock measurementClock;
static MeasurementHandoff::FlashPause flashPause;
static MeasurementHandoff::CoreHeartbeat audioHeartbeat;
static std::atomic<uint32_t> audioBootstrapReady{0};
static std::atomic<uint32_t> audioEvents{0};
static std::atomic<uint32_t> audioSamplesReported{0};
static std::atomic<uint32_t> audioLevelsReported{0};
static std::atomic<uint32_t> audioLevelReported{0};
static const uint64_t MEASUREMENT_PERIOD_US = static_cast<uint64_t>(SEND_AFTER) * 1000ULL;
static_assert(SEND_AFTER > 0, "Measurement period must be nonzero");
enum AudioEvent : uint32_t { AUDIO_STARTED=1, AUDIO_START_FAILED=2,
  AUDIO_STALLED=4, AUDIO_EMPTY_WINDOW=8, AUDIO_PAUSE_FAILED=16,
  AUDIO_RESUME_FAILED=32 };
enum AudioOperation : uint32_t { AUDIO_START=1, AUDIO_READ=2,
  AUDIO_STOP=3, AUDIO_PACK=4, AUDIO_FLASH_PAUSED=5, AUDIO_HANDOFF=6 };
struct AudioProgressRecord { uint32_t magic, operation, inverse; };
static volatile AudioProgressRecord audioProgress
  __attribute__((section(".uninitialized_data.audio_progress")));
static uint32_t bootAudioOperation = 0;
static bool audioStallReported = false;
static MeasurementHandoff::Measurement pendingMeasurement;
static bool havePendingMeasurement = false;
static bool restoredDeadlineAwaitingClock = false;
enum FlashWork { FLASH_NONE, FLASH_FRAME, FLASH_PROTOCOL };
static FlashWork flashWork = FLASH_NONE;
static uint32_t flashGeneration = 0, activeFlashGeneration = 0;
static uint32_t flashDeadlineAt = 0;
static uint32_t nextClockAnchorAt = 0;
TTNSemtechUDPClient gateway;
FrameCounterStore frameCounters;
LoRaWANSessionStore sessionStore;
static LoRaWANMacState deviceMac;
static LoRaWANFrame::CounterState downlinkCounters = {};
static LoRaWANMacState::AnswerBatch pendingAnswerBatch = {};
static LoRaWANRadioSettings pendingUplinkRadio = {};
static bool protocolStorageReady = false;
static bool protocolFlashDirty = false;
static uint32_t nextProtocolSaveAt = 0;
static uint32_t sessionIdentity = 0;

static bool havePendingFrame = false;
static bool pendingDownlinkAck = false;
static bool pendingFrameAcknowledgesDownlink = false;
static uint32_t downlinkAckGeneration = 0;
static uint32_t pendingFrameAckGeneration = 0;
static uint32_t nextUplinkAttemptAt = 0;
static uint32_t uplinkRetryDelayMs = 1000UL;
static uint32_t longestCompletedLoopMicros = 0;
static uint8_t pendingFrame[LoRaWANFrame::MAX_FRAME_SIZE];
static size_t pendingFrameLength = 0;
static uint8_t pendingApplicationPayload[MeasurementHandoff::Measurement::PAYLOAD_BYTES] = {};

// The Pico's .uninitialized_data RAM survives a watchdog reboot. Keep the
// complete device state and prepared PHYPayload there so recovery does not
// discard negotiated parameters, MAC answers, counters or a measurement.
// Flash holds the protocol state for power loss; RAM also holds the frame.
struct RetainedPendingUplink {
  uint32_t magic;
  uint16_t version;
  uint16_t recordSize;
  uint32_t identity;
  LoRaWANSessionStore::State protocol;
  uint16_t frameLength;
  uint8_t applicationPayload[MeasurementHandoff::Measurement::PAYLOAD_BYTES];
  uint8_t frame[LoRaWANFrame::MAX_FRAME_SIZE];
  LoRaWANRadioSettings radio;
  LoRaWANMacState::AnswerBatch answers;
  uint32_t frameAckGeneration;
  uint8_t frameAcknowledgesDownlink;
  uint32_t checksum;
};
static RetainedPendingUplink retainedPendingUplink
  __attribute__((section(".uninitialized_data")));
static const uint32_t RETAINED_UPLINK_MAGIC = 0x56535032UL; // "VSP2"
static const uint16_t RETAINED_UPLINK_VERSION = 3;
static RP2040::resetReason_t bootResetReason = RP2040::UNKNOWN_RESET;
static const char *bootLastOperation = nullptr;
static PicoCrashDiagnostics::Snapshot bootFaultSnapshot = {};
static bool bootFaultCaptured = false;
static uint32_t bootPreviousLoopMaximum = 0;
static bool bootDiagnosticsPrinted = false;
static bool restoredPendingUplinkAtBoot = false;

static uint8_t toUQ7_1(double value) {
  if (value < 0.0) value = 0.0;
  if (value > 127.5) value = 127.5;
  return static_cast<uint8_t>(value * 2.0);
}

static uint8_t toUQ5_3(double value) {
  if (value < 0.0) value = 0.0;
  if (value > 31.875) value = 31.875;
  return static_cast<uint8_t>(value * 8.0);
}

static void logPreparedMeasurement(const uint8_t *payload);

static uint32_t retentionCrcByte(uint32_t crc, uint8_t value) {
  crc ^= value;
  for (uint8_t bit = 0; bit < 8; ++bit)
    crc = (crc & 1U) ? (crc >> 1) ^ 0xEDB88320UL : crc >> 1;
  return crc;
}

static uint32_t currentSessionIdentity() {
  // Bind saved counters/settings to this ABP session without storing or
  // printing either secret key. A changed session discards the old journal.
  const char *parts[] = {LORAWAN_DEVADDR_HEX, LORAWAN_NWKSKEY_HEX,
                         LORAWAN_APPSKEY_HEX};
  uint32_t crc = 0xFFFFFFFFUL;
  for (const char *part : parts) {
    for (size_t i = 0; part[i]; ++i)
      crc = retentionCrcByte(crc, static_cast<uint8_t>(part[i]));
    crc = retentionCrcByte(crc, 0);
  }
  return ~crc;
}

static LoRaWANSessionStore::State currentProtocolState() {
  LoRaWANSessionStore::State state = {};
  state.mac = deviceMac.snapshot();
  state.lastDownlinkCounter = downlinkCounters.last;
  state.downlinkCounterSeen = downlinkCounters.seen;
  state.ackGeneration = downlinkAckGeneration;
  state.ackPending = pendingDownlinkAck;
  return state;
}

static void applyProtocolState(const LoRaWANSessionStore::State &state) {
  deviceMac.restore(state.mac);
  downlinkCounters.last = state.lastDownlinkCounter;
  downlinkCounters.seen = state.downlinkCounterSeen != 0;
  downlinkAckGeneration = state.ackGeneration;
  pendingDownlinkAck = state.ackPending != 0;
}

static void retainDeviceState() {
  RetainedPendingUplink record = {};
  record.version = RETAINED_UPLINK_VERSION;
  record.recordSize = sizeof(record);
  record.identity = sessionIdentity;
  record.protocol = currentProtocolState();
  if (havePendingFrame) {
    record.frameLength = static_cast<uint16_t>(pendingFrameLength);
    memcpy(record.applicationPayload, pendingApplicationPayload, sizeof(pendingApplicationPayload));
    memcpy(record.frame, pendingFrame, pendingFrameLength);
    record.radio = pendingUplinkRadio;
    record.answers = pendingAnswerBatch;
    record.frameAckGeneration = pendingFrameAckGeneration;
    record.frameAcknowledgesDownlink = pendingFrameAcknowledgesDownlink;
  }
  record.checksum = LoRaWANSessionStore::crc(&record,
    offsetof(RetainedPendingUplink, checksum));
  retainedPendingUplink.magic = 0;
  // Preserve the exact checksummed representation, including padding bytes.
  memcpy(&retainedPendingUplink, &record, sizeof(record));
  __asm volatile("" ::: "memory");
  retainedPendingUplink.magic = RETAINED_UPLINK_MAGIC;
}

static bool restorePreparedUplink() {
  // The validity marker is excluded from the checksum by clearing it in a
  // local copy. Even a journal with no pending frame retains device state.
  RetainedPendingUplink record;
  memcpy(&record, &retainedPendingUplink, sizeof(record));
  record.magic = 0;
  if (retainedPendingUplink.magic != RETAINED_UPLINK_MAGIC ||
      record.version != RETAINED_UPLINK_VERSION ||
      record.recordSize != sizeof(record) || record.identity != sessionIdentity ||
      record.frameLength > sizeof(pendingFrame) ||
      record.frameAcknowledgesDownlink > 1 ||
      record.answers.count > 15 || record.answers.byteLength > 15 ||
      (record.frameLength && (record.frameLength < 13 || record.frame[0] != 0x40 ||
        (record.frame[5] & 0x0F) != record.answers.byteLength ||
        record.radio.frequencyHz < 863000000UL || record.radio.frequencyHz > 870000000UL ||
        record.radio.spreadingFactor < 7 || record.radio.spreadingFactor > 12 ||
        (record.radio.bandwidthHz != 125000UL && record.radio.bandwidthHz != 250000UL))) ||
      !LoRaWANSessionStore::validState(record.protocol) ||
      record.checksum != LoRaWANSessionStore::crc(&record,
        offsetof(RetainedPendingUplink, checksum))) {
    retainedPendingUplink.magic = 0;
    return false;
  }

  applyProtocolState(record.protocol);
  protocolFlashDirty = true;
  LOG_PRINTLN(F("Restored LoRaWAN MAC settings, downlink counter and pending answers from retained RAM"));
  if (!record.frameLength) return false;
  pendingFrameLength = record.frameLength;
  memcpy(pendingFrame, record.frame, pendingFrameLength);
  memcpy(pendingApplicationPayload, record.applicationPayload, sizeof(pendingApplicationPayload));
  memcpy(pendingMeasurement.applicationPayload, record.applicationPayload, sizeof(pendingApplicationPayload));
  pendingMeasurement.expiresUs = UINT64_MAX;
  havePendingMeasurement = true;
  restoredDeadlineAwaitingClock = true;
  pendingUplinkRadio = record.radio;
  pendingAnswerBatch = record.answers;
  pendingFrameAcknowledgesDownlink = record.frameAcknowledgesDownlink != 0;
  pendingFrameAckGeneration = record.frameAckGeneration;
  havePendingFrame = true;
  restoredPendingUplinkAtBoot = true;
  LOG_PRINT(F("Recovered retained measurement uplink; FCntUp="));
  LOG_PRINT(static_cast<uint32_t>(pendingFrame[6]) |
            (static_cast<uint32_t>(pendingFrame[7]) << 8));
  LOG_PRINTLN(F("; the original application payload and LoRaWAN frame are preserved"));
  logPreparedMeasurement(pendingApplicationPayload);
  return true;
}

static void markProtocolChanged() {
  retainDeviceState();
  protocolFlashDirty = true;
}

static void saveProtocolStateWhenDue() {
  // Called only from serviceFlashWork after core1 has stopped audio and ACKed.
  TTNSemtechUDPClient::markProtocolStorageOperation();
  if (sessionStore.save(currentProtocolState())) {
    protocolFlashDirty = false;
    nextProtocolSaveAt = 0;
  } else {
    LOG_PRINTLN(F("LoRaWAN MAC flash journal save failed; RAM state retained, retry in 30 seconds"));
    nextProtocolSaveAt = millis() + 30000UL;
  }
  TTNSemtechUDPClient::clearOperationMarker();
}

static void printBootDiagnostics() {
  if (bootDiagnosticsPrinted || !Serial) return;
  LOG_PRINT(F("Voorburg Sounds Kit starting (Pico 2 W virtual LoRaWAN endpoint) [build "));
  LOG_PRINT(F(__DATE__));
  LOG_PRINT(' ');
  LOG_PRINT(F(__TIME__));
  LOG_PRINTLN(F("] [MAC revision 2; recovery revision 5; dual-core 1; cleanup 3; UDP only; essential logs]"));
  switch (bootResetReason) {
    case RP2040::PWRON_RESET:
      LOG_PRINTLN(F("Reset reason: power-on"));
      break;
    case RP2040::WDT_RESET:
      LOG_PRINTLN(F("WATCHDOG RESET: hardware watchdog restarted the Pico after 8 seconds without loop progress"));
      if (bootPreviousLoopMaximum) {
        LOG_PRINT(F("Longest completed loop before reset: "));
        LOG_PRINT(bootPreviousLoopMaximum / 1000.0, 3);
        LOG_PRINTLN(F(" ms (the interrupted loop did not complete)"));
      }
      break;
    case RP2040::RUN_PIN_RESET:
      LOG_PRINTLN(F("Reset reason: RUN pin"));
      break;
    case RP2040::SOFT_RESET:
      LOG_PRINTLN(F("Reset reason: software restart"));
      break;
    case RP2040::DEBUG_RESET:
      LOG_PRINTLN(F("Reset reason: debugger"));
      break;
    case RP2040::GLITCH_RESET:
      LOG_PRINTLN(F("Reset reason: glitch"));
      break;
    case RP2040::BROWNOUT_RESET:
      LOG_PRINTLN(F("Reset reason: brownout"));
      break;
    default:
      LOG_PRINT(F("Reset reason: unknown (code "));
      LOG_PRINT(static_cast<int>(bootResetReason));
      LOG_PRINTLN(')');
      break;
  }
  if (bootResetReason == RP2040::WDT_RESET || bootFaultCaptured) {
    if (bootLastOperation) {
      LOG_PRINT(F("Last marked Semtech operation before reset: "));
      LOG_PRINTLN(bootLastOperation);
    } else if (bootResetReason == RP2040::WDT_RESET) {
      LOG_PRINTLN(F("No valid operation marker was retained for this reset"));
    }
    if (bootResetReason == RP2040::WDT_RESET && bootAudioOperation) {
      LOG_PRINT(F("Last core 1 audio operation before reset (1=start,2=read/FFT,3=stop,4=pack,5=flash pause,6=handoff): "));
      LOG_PRINTLN(bootAudioOperation);
    }
    if (bootFaultCaptured) {
      LOG_PRINT(F("CPU capture: "));
      LOG_PRINT(bootFaultSnapshot.kind == PicoCrashDiagnostics::HARDFAULT ?
        F("HardFault") : F("watchdog deadline"));
      LOG_PRINT(F("; core=")); LOG_PRINT(bootFaultSnapshot.core);
      LOG_PRINT(F("; marked operation=")); LOG_PRINT(bootFaultSnapshot.operation);
      if (bootFaultSnapshot.flags & PicoCrashDiagnostics::FRAME_VALID) {
        LOG_PRINT(F("; PC=0x")); LOG_PRINT(bootFaultSnapshot.pc, HEX);
        LOG_PRINT(F("; LR=0x")); LOG_PRINT(bootFaultSnapshot.lr, HEX);
        LOG_PRINT(F("; interrupted exception="));
        LOG_PRINT(bootFaultSnapshot.xpsr & 0x1FFUL);
      } else {
        LOG_PRINT(F("; interrupted stack frame was not readable"));
      }
      LOG_PRINTLN();
      LOG_PRINT(F("CPU fault registers: CFSR=0x"));
      LOG_PRINT(bootFaultSnapshot.cfsr, HEX);
      LOG_PRINT(F("; HFSR=0x")); LOG_PRINT(bootFaultSnapshot.hfsr, HEX);
      LOG_PRINT(F("; BFAR=0x")); LOG_PRINTLN(bootFaultSnapshot.bfar, HEX);
    } else if (bootResetReason == RP2040::WDT_RESET) {
      LOG_PRINTLN(F("No CPU instruction capture retained; a masked interrupt or unavailable capture timer can prevent it"));
    }
    if (restoredPendingUplinkAtBoot) {
      LOG_PRINTLN(F("Restart recovery restored an unsent measurement; its UTC expiry will be checked while core 1 measures independently"));
    }
  }
  bootDiagnosticsPrinted = true;
}

static void serviceUsbLog() {
#if USB_LOGGING
  TTNSemtechUDPClient::markUsbLogOperation();
  eventLog.service();
  TTNSemtechUDPClient::clearOperationMarker();
#endif
}

// Raw application bytes before LoRaWAN encryption; TTN decodes these bytes.
static void logPreparedMeasurement(const uint8_t *payload) {
  LOG_PRINT(F("PAYLOAD_HEX="));
  for (size_t i = 0; i < MeasurementHandoff::Measurement::PAYLOAD_BYTES; ++i) {
    if (payload[i] < 0x10) LOG_PRINT('0');
    LOG_PRINT(payload[i], HEX);
  }
  LOG_PRINTLN();
}

static bool processMacCommandStream(const uint8_t *commands, size_t length) {
  // Validate the entire stream before applying settings or creating answers.
  size_t offset = 0;
  while (offset < length) {
    const uint8_t cid = commands[offset++];
    size_t requestBytes = 0;
    switch (cid) {
      case 0x05: requestBytes = 4; break;
      case 0x06: requestBytes = 0; break;
      case 0x07: requestBytes = 5; break;
      case 0x08: requestBytes = 1; break;
      default:
        LOG_PRINT(F("Unsupported downlink MAC CID 0x"));
        LOG_PRINT(cid, HEX);
        LOG_PRINTLN(F("; this command stream was not applied"));
        return false;
    }
    if (length - offset < requestBytes) {
      LOG_PRINT(F("Truncated downlink MAC CID 0x"));
      LOG_PRINT(cid, HEX);
      LOG_PRINTLN(F("; this command stream was not applied"));
      return false;
    }
    offset += requestBytes;
  }
  if (!deviceMac.processCommands(commands, length,
      static_cast<uint8_t>(TTN_DEVICE_STATUS_BATTERY),
      static_cast<int8_t>(TTN_DEVICE_STATUS_MARGIN_DB))) {
    LOG_PRINTLN(F("MAC answer queue could not accept this stream; no requested settings were applied"));
    return false;
  }

  return true;
}

static bool prepareUplink() {
  // Core0 owns LoRaWAN/MAC/counters. Audio already copied this snapshot and
  // continues independently. Flash permission is held by serviceFlashWork.
  if (!havePendingMeasurement || pendingMeasurement.expired(time_us_64())) return false;
#if TTN_UPLINK_ENABLED
  if (!pendingMeasurement.windowEndUtc()) return false;
#endif
  const uint8_t *payload = pendingMeasurement.applicationPayload;
  TTNSemtechUDPClient::markCounterStorageOperation();
  const uint32_t frameCounter = frameCounters.reserve();
  TTNSemtechUDPClient::clearOperationMarker();
  if (frameCounter == UINT32_MAX) {
    LOG_PRINTLN(F("ERROR: could not persist LoRaWAN frame counter"));
    return false;
  }
  pendingFrameAcknowledgesDownlink = pendingDownlinkAck;
  pendingFrameAckGeneration = downlinkAckGeneration;
  uint8_t fOpts[15] = {};
  const size_t fOptsLength = deviceMac.prepareAnswers(fOpts, pendingAnswerBatch);
  pendingUplinkRadio = deviceMac.nextUplinkRadio();
  TTNSemtechUDPClient::markFrameBuildOperation();
  const bool frameBuilt = LoRaWANFrame::buildUplink(payload, MeasurementHandoff::Measurement::PAYLOAD_BYTES, frameCounter,
                                 pendingFrame, pendingFrameLength,
                                 pendingFrameAcknowledgesDownlink,
                                 fOpts, fOptsLength);
  TTNSemtechUDPClient::clearOperationMarker();
  if (!frameBuilt) {
    LOG_PRINTLN(F("ERROR: unable to construct ABP frame; check the local keys"));
    return false;
  }
  memcpy(pendingApplicationPayload, payload, sizeof(pendingApplicationPayload));
  havePendingFrame = true;
#if TTN_UPLINK_ENABLED
  retainDeviceState();
#endif
  logPreparedMeasurement(payload);
  return true;
}

// This is the software equivalent of the receiver reached by the Pico's
// virtual LoRa transmission. It is called only after the scheduled virtual
// airtime has elapsed; gateway TxAck reports transmit completion, while
// end-device MIC/counter processing remains separate.
static bool processVirtualRadioFrame(const uint8_t *frame, size_t frameLength,
                                    const LoRaWANDownlinkSettings &settings) {
  if (!deviceMac.acceptsDownlink(settings, micros())) {
    LOG_PRINTLN(F("Pico end-device ignored downlink outside its RX1/RX2 timing or radio settings"));
    return false;
  }
  LoRaWANFrame::DownlinkInfo downlink = {};
  LoRaWANFrame::DownlinkRejectReason rejectReason =
    LoRaWANFrame::DownlinkRejectReason::None;
  if (!LoRaWANFrame::receiveDownlink(frame, frameLength, downlink,
                                     &rejectReason, &downlinkCounters)) {
    LOG_PRINT(F("Pico end-device rejected virtual downlink: "));
    LOG_PRINTLN(LoRaWANFrame::downlinkRejectReasonName(rejectReason));
    return false;
  }

  deviceMac.onValidDownlink();

  if (!downlink.duplicate && downlink.fOptsLength) {
    processMacCommandStream(downlink.fOpts, downlink.fOptsLength);
  }
  if (!downlink.duplicate && downlink.hasFPort && downlink.fPort == 0 && downlink.payloadLength) {
    processMacCommandStream(downlink.payload, downlink.payloadLength);
  }
  if (downlink.confirmed) {
    pendingDownlinkAck = true;
    ++downlinkAckGeneration;
  }
  markProtocolChanged();
  return true;
}

static bool handleVirtualRadioFrame(const uint8_t *frame, size_t frameLength,
                                   const LoRaWANDownlinkSettings &settings) {
  // Called after the scheduled transmission airtime, before gateway TX_ACK.
  return processVirtualRadioFrame(frame, frameLength, settings);
}



static void markAudioOperation(uint32_t operation) {
  PicoCrashDiagnostics::setOperation(operation);
  audioProgress.magic = 0;
  audioProgress.operation = operation;
  audioProgress.inverse = ~operation;
  __dmb();
  audioProgress.magic = 0x41554431UL;
}

// Only core1 accesses SLM, its DMA/PIO driver, or floating-point audio stats.
void setup1() {}
void loop1() {
  static bool initialized = false, capturing = false, publishWaiting = false;
  static uint32_t nextStartAt = 0, retryDelay = 1000, lastSampleCount = 0;
  static uint32_t lastSampleAt = 0, pausedGeneration = 0, sequence = 0;
  static uint64_t nextWindowEndUs = 0;
  static MeasurementHandoff::Measurement produced;
  const uint64_t nowUs = time_us_64();
  const uint32_t now = millis();
  if (!audioBootstrapReady.load(std::memory_order_acquire)) {
    // Preserve the previous boot's operation record until core0 reads it.
    audioHeartbeat.beat(nowUs);
    return;
  }
  if (!initialized) {
    initialized = true;
    nextWindowEndUs = nowUs + MEASUREMENT_PERIOD_US;
    nextStartAt = now;
  }

  const uint32_t request = flashPause.requestedGeneration();
  if (request && request != pausedGeneration) {
    markAudioOperation(AUDIO_STOP);
    const bool stopped = slm.pause();
    capturing = false;
    pausedGeneration = request;
    flashPause.acknowledgeStopped(request, stopped);
    if (!stopped) audioEvents.fetch_or(AUDIO_PAUSE_FAILED, std::memory_order_release);
  }
  if (pausedGeneration && flashPause.resumeRequested(pausedGeneration)) {
    pausedGeneration = 0;
    markAudioOperation(AUDIO_START);
    capturing = slm.start();
    lastSampleCount = slm.samplesRead();
    lastSampleAt = now;
    if (!capturing) {
      audioEvents.fetch_or(AUDIO_RESUME_FAILED, std::memory_order_release);
      nextStartAt = now + 1000;
    }
  }
  if (pausedGeneration) {
    markAudioOperation(AUDIO_FLASH_PAUSED);
  } else {
    if (!capturing && static_cast<int32_t>(now - nextStartAt) >= 0) {
      markAudioOperation(AUDIO_START);
      capturing = slm.start();
      if (capturing) {
        audioEvents.fetch_or(AUDIO_STARTED, std::memory_order_release);
        lastSampleCount = slm.samplesRead();
        lastSampleAt = now;
        retryDelay = 1000;
      } else {
        audioEvents.fetch_or(AUDIO_START_FAILED, std::memory_order_release);
        nextStartAt = now + retryDelay;
        if (retryDelay < 30000) retryDelay *= 2;
        if (retryDelay > 30000) retryDelay = 30000;
      }
    }
    if (capturing) {
      markAudioOperation(AUDIO_READ);
      slm.update();
      const uint32_t samples = slm.samplesRead();
      audioSamplesReported.store(samples, std::memory_order_relaxed);
      audioLevelsReported.store(static_cast<uint32_t>(slm.statsDB.N()), std::memory_order_relaxed);
      audioLevelReported.store(toUQ7_1(slm.latestLevelRelative), std::memory_order_relaxed);
      if (samples != lastSampleCount) {
        lastSampleCount = samples;
        lastSampleAt = now;
      } else if (now - lastSampleAt >= MIC_SAMPLE_STALL_MS) {
        markAudioOperation(AUDIO_STOP);
        slm.stop();
        capturing = false;
        audioEvents.fetch_or(AUDIO_STALLED, std::memory_order_release);
        nextStartAt = now + 1000;
      }
    }
  }

  // Anchor the schedule to time, not delivery completion. A flash pause does
  // not move the boundary; an empty window produces no invented measurement.
  if (nowUs >= nextWindowEndUs) {
    markAudioOperation(AUDIO_PACK);
    const uint64_t windowEndUs = nextWindowEndUs;
    nextWindowEndUs += ((nowUs - nextWindowEndUs) / MEASUREMENT_PERIOD_US + 1) * MEASUREMENT_PERIOD_US;
    if (slm.statsDB.N() > 1) {
      produced = {};
      if (++sequence == 0) ++sequence;
      produced.sequence = sequence;
      produced.completedUs = windowEndUs;
      produced.expiresUs = nextWindowEndUs;
      const double leq = 120.0 + 10.0 * log10(slm.statsRMS.Mean() * 2.0);
      produced.applicationPayload[0] = toUQ7_1(slm.statsDB.Mean());
      produced.applicationPayload[1] = toUQ7_1(slm.statsDB.Min());
      produced.applicationPayload[2] = toUQ7_1(slm.statsDB.Max());
      produced.applicationPayload[3] = toUQ5_3(slm.statsDB.StdDev(true));
      produced.applicationPayload[4] = toUQ7_1(leq);
      uint32_t utc;
      if (measurementClock.trySecondsAt(windowEndUs, utc)) produced.setWindowEndUtc(utc);
      publishWaiting = true;
    } else {
      publishWaiting = false;
      audioEvents.fetch_or(AUDIO_EMPTY_WINDOW, std::memory_order_release);
    }
    // The completed payload owns its bytes. Reset capture/analysis for the
    // next window immediately; never wait for core0 to deliver that payload.
    markAudioOperation(AUDIO_STOP);
    const bool stopped = slm.stop();
    capturing = false;
    slm.resetStatistics();
    audioSamplesReported.store(0, std::memory_order_relaxed);
    audioLevelsReported.store(0, std::memory_order_relaxed);
    if (!pausedGeneration && stopped) {
      markAudioOperation(AUDIO_START);
      capturing = slm.start();
      lastSampleCount = slm.samplesRead();
      lastSampleAt = millis();
      if (capturing) audioEvents.fetch_or(AUDIO_STARTED, std::memory_order_release);
    }
    if (!pausedGeneration && !capturing) {
      nextStartAt = millis() + 1000;
      audioEvents.fetch_or(AUDIO_START_FAILED, std::memory_order_release);
    }
  }
  if (publishWaiting) {
    markAudioOperation(AUDIO_HANDOFF);
    if (produced.expired(time_us_64())) publishWaiting = false;
    else if (measurementMailbox.tryPublish(produced)) publishWaiting = false;
  }
  audioHeartbeat.beat(time_us_64());
}

static void publishClockAnchor() {
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - nextClockAnchorAt) < 0) return;
  // The core's NTP wall-clock offset is 64 bits. Sample it only on core0,
  // with its callbacks briefly masked so the offset cannot tear mid-read.
  timeval wall = {};
  const uint32_t saved = save_and_disable_interrupts();
  const uint64_t mono = time_us_64();
  gettimeofday(&wall, nullptr);
  restore_interrupts(saved);
  if (wall.tv_sec >= 1700000000LL && static_cast<uint64_t>(wall.tv_sec) <= UINT32_MAX &&
      measurementClock.tryUpdate(mono, static_cast<uint32_t>(wall.tv_sec), wall.tv_usec)) {
    nextClockAnchorAt = now + 1000;
  }
}

static void logAudioEvents() {
  const uint32_t flags = audioEvents.exchange(0, std::memory_order_acquire);
  if (flags & AUDIO_START_FAILED) LOG_PRINTLN(F("Core 1 could not start microphone capture; retry scheduled"));
  if (flags & AUDIO_STALLED) LOG_PRINTLN(F("Core 1 microphone samples stalled; capture recovery scheduled"));
  if (flags & AUDIO_EMPTY_WINDOW) LOG_PRINTLN(F("Core 1 measurement window had insufficient audio levels; no measurement published"));
  if (flags & AUDIO_PAUSE_FAILED) LOG_PRINTLN(F("Core 1 audio stop failed; flash write deferred"));
  if (flags & AUDIO_RESUME_FAILED) LOG_PRINTLN(F("Core 1 audio resume failed; capture recovery scheduled"));
#if USB_LOGGING && USB_MIC_LOGGING
  static uint32_t nextLogAt = 0;
  if (static_cast<int32_t>(millis() - nextLogAt) >= 0) {
    nextLogAt = millis() + 1000;
    LOG_PRINT(F("Core 1 mic selected samples=")); LOG_PRINT(audioSamplesReported.load());
    LOG_PRINT(F("; levels this window=")); LOG_PRINT(audioLevelsReported.load());
    LOG_PRINT(F("; latest level=")); LOG_PRINTLN(audioLevelReported.load() / 2.0, 1);
  }
#endif
}

static void discardPendingMeasurement(const __FlashStringHelper *reason) {
  LOG_PRINTLN(reason);
  havePendingFrame = false;
  havePendingMeasurement = false;
  restoredDeadlineAwaitingClock = false;
  pendingFrameLength = 0;
  pendingAnswerBatch = {};
  pendingFrameAcknowledgesDownlink = false;
  nextUplinkAttemptAt = 0;
  // Do not submit or clear MAC answers/confirmed ACKs for an unsent frame.
  retainDeviceState();
}

static void takeNewestMeasurement() {
  MeasurementHandoff::Measurement newest;
  const uint64_t nowUs = time_us_64();
  if (measurementMailbox.tryTakeLatest(newest, nowUs)) {
    if (havePendingMeasurement) discardPendingMeasurement(F("New five-minute measurement replaced the unsent older payload"));
    pendingMeasurement = newest;
    havePendingMeasurement = true;
    uplinkRetryDelayMs = 1000;
  }
  if (!havePendingMeasurement) return;
  if (restoredDeadlineAwaitingClock) {
    uint32_t nowUtc;
    if (!measurementClock.trySecondsAt(nowUs, nowUtc)) return;
    const uint64_t expiresUtc = static_cast<uint64_t>(pendingMeasurement.windowEndUtc()) + SEND_AFTER / 1000;
    if (!pendingMeasurement.windowEndUtc() || nowUtc >= expiresUtc) {
      discardPendingMeasurement(F("Restored measurement exceeded its five-minute delivery deadline; discarded"));
      return;
    }
    pendingMeasurement.expiresUs = nowUs + (expiresUtc - nowUtc) * 1000000ULL;
    pendingMeasurement.completedUs = pendingMeasurement.expiresUs - MEASUREMENT_PERIOD_US;
    restoredDeadlineAwaitingClock = false;
  }
  if (pendingMeasurement.expired(nowUs)) {
    discardPendingMeasurement(F("Measurement delivery deadline reached; discarded unsent payload and waiting for next window"));
    return;
  }
  if (!pendingMeasurement.windowEndUtc()) {
    uint32_t utc;
    if (measurementClock.trySecondsAt(pendingMeasurement.completedUs, utc))
      pendingMeasurement.setWindowEndUtc(utc);
  }
}

static void serviceFlashWork() {
  const uint32_t now = millis();
  if (flashWork == FLASH_NONE) {
    if (havePendingMeasurement && !havePendingFrame &&
        (!TTN_UPLINK_ENABLED || pendingMeasurement.windowEndUtc()) &&
        static_cast<int32_t>(now - nextUplinkAttemptAt) >= 0) flashWork = FLASH_FRAME;
    else if (protocolStorageReady && protocolFlashDirty &&
        static_cast<int32_t>(now - nextProtocolSaveAt) >= 0) flashWork = FLASH_PROTOCOL;
    else return;
    if (++flashGeneration == 0) ++flashGeneration;
    activeFlashGeneration = flashGeneration;
    if (!flashPause.request(activeFlashGeneration)) { flashWork = FLASH_NONE; return; }
    flashDeadlineAt = now + 1000;
    return;
  }
  if (flashPause.failed(activeFlashGeneration) ||
      static_cast<int32_t>(now - flashDeadlineAt) >= 0) {
    flashPause.release(activeFlashGeneration);
    flashWork = FLASH_NONE;
    nextProtocolSaveAt = now + 1000;
    nextUplinkAttemptAt = now + 1000;
    LOG_PRINTLN(F("Audio flash-pause handshake did not finish; write deferred"));
    return;
  }
  if (!flashPause.acknowledged(activeFlashGeneration)) return;
  if (flashWork == FLASH_PROTOCOL) saveProtocolStateWhenDue();
  else if (havePendingMeasurement && !havePendingFrame && !pendingMeasurement.expired(time_us_64())) {
    if (!prepareUplink()) nextUplinkAttemptAt = now + 1000;
#if !TTN_UPLINK_ENABLED
    if (havePendingFrame) {
      havePendingFrame = false;
      havePendingMeasurement = false;
      pendingFrameLength = 0;
    }
#endif
  }
  flashPause.release(activeFlashGeneration);
  flashWork = FLASH_NONE;
}

void setup() {
  LOG_BEGIN();
#if USB_LOGGING
  // A watchdog reboot disconnects USB CDC and makes `screen` exit. Wait for a
  // newly opened serial session so the next boot can show the reset diagnosis.
  const uint32_t serialAttachStartedAt = millis();
  while (!Serial && millis() - serialAttachStartedAt < 10000UL) delay(10);
  delay(250);
#endif
  bootResetReason = rp2040.getResetReason();
  if (bootResetReason == RP2040::WDT_RESET && audioProgress.magic == 0x41554431UL &&
      audioProgress.inverse == ~audioProgress.operation) bootAudioOperation = audioProgress.operation;
  audioProgress.magic = 0;
  bootFaultCaptured = PicoCrashDiagnostics::readPrevious(
    bootFaultSnapshot, bootResetReason == RP2040::WDT_RESET);
  bootLastOperation = TTNSemtechUDPClient::consumePreviousOperation();
  bootPreviousLoopMaximum = TTNSemtechUDPClient::consumePreviousLoopMaximum();
  printBootDiagnostics();
#if USB_LOGGING
  serviceUsbLog();
#endif
  // Start supervision before peripheral startup. A stuck I2S/Wi-Fi startup
  // then causes a hardware reset instead of leaving the Pico idle.
  rp2040.wdt_begin(8000);
  const bool timeoutCaptureAvailable = PicoCrashDiagnostics::begin();
  PicoCrashDiagnostics::arm();
  (void)timeoutCaptureAvailable;
  LOG_PRINTLN(F("TTN uplink transport: Semtech UDP Packet Forwarder protocol (GWMP v2)"));
  const LoRaWANRadioSettings initialRadio = {
    TTN_UPLINK_FREQUENCY_HZ, TTN_UPLINK_BANDWIDTH_HZ,
    TTN_UPLINK_SPREADING_FACTOR, 0
  };
  deviceMac.reset(initialRadio);
  sessionIdentity = currentSessionIdentity();
  const size_t journalBytes = LoRaWANSessionStore::storageBytes();
  if (FrameCounterStore::reservedBytes() + journalBytes > 4096 ||
      !frameCounters.begin(journalBytes)) {
    LOG_PRINTLN(F("ERROR: frame-counter storage initialization failed"));
  } else {
    protocolStorageReady = true;
    sessionStore.begin(FrameCounterStore::reservedBytes(), sessionIdentity);
    LoRaWANSessionStore::State saved = {};
    if (sessionStore.load(saved)) {
      applyProtocolState(saved);
      LOG_PRINTLN(F("Restored LoRaWAN MAC settings, downlink counter and pending answers from flash"));
    }
  }
  // Restore the newer RAM record before networking can deliver a downlink.
  restorePreparedUplink();
#if TTN_UPLINK_ENABLED
  gateway.setDeviceMacState(&deviceMac);
  gateway.setVirtualDownlinkHandler(handleVirtualRadioFrame);
  gateway.begin();
  if (!gateway.configured()) {
    LOG_PRINTLN(F("Edit LoraSettings.h; network connection is disabled until configuration is valid"));
  }
#else
  LOG_PRINTLN(F("TTN uplink transport disabled; USB packet inspection only"));
#endif
  audioBootstrapReady.store(1, std::memory_order_release);
}

void loop() {
  const uint32_t loopStartedAt = micros();
#if USB_LOGGING
  printBootDiagnostics();
  serviceUsbLog();
#endif
#if TTN_UPLINK_ENABLED
  gateway.maintain();
  TTNSemtechUDPClient::clearOperationMarker();
#endif
  publishClockAnchor();
  logAudioEvents();
  takeNewestMeasurement();
  serviceFlashWork();

#if TTN_UPLINK_ENABLED
  if (havePendingFrame && !restoredDeadlineAwaitingClock && gateway.ready()) {
    const bool routeOpen = gateway.prepareUplinkRoute();
    // Recheck expiry after route work, immediately before submitting bytes.
    if (pendingMeasurement.expired(time_us_64())) {
      discardPendingMeasurement(F("Measurement expired while opening TTN route; discarded"));
    } else if (routeOpen &&
        (nextUplinkAttemptAt == 0 || static_cast<int32_t>(millis() - nextUplinkAttemptAt) >= 0)) {
      const bool sent = gateway.publishUplink(pendingFrame, pendingFrameLength, pendingUplinkRadio);
      TTNSemtechUDPClient::clearOperationMarker();
      if (sent) {
        const bool sentProtocolAnswer = pendingAnswerBatch.count != 0 || pendingFrameAcknowledgesDownlink;
        deviceMac.answersSubmitted(pendingAnswerBatch);
        havePendingFrame = false;
        havePendingMeasurement = false;
        pendingFrameLength = 0;
        pendingAnswerBatch = {};
        nextUplinkAttemptAt = 0;
        uplinkRetryDelayMs = 1000;
        if (pendingFrameAcknowledgesDownlink && pendingFrameAckGeneration == downlinkAckGeneration)
          pendingDownlinkAck = false;
        pendingFrameAcknowledgesDownlink = false;
        pendingFrameAckGeneration = 0;
        retainDeviceState();
        if (sentProtocolAnswer) protocolFlashDirty = true;
      } else {
        LOG_PRINTLN(F("Uplink submission failed; retaining this measurement until its five-minute deadline"));
        nextUplinkAttemptAt = millis() + uplinkRetryDelayMs;
        if (uplinkRetryDelayMs < 30000) uplinkRetryDelayMs *= 2;
        if (uplinkRetryDelayMs > 30000) uplinkRetryDelayMs = 30000;
      }
    }
  }
#endif
  TTNSemtechUDPClient::clearOperationMarker();
#if USB_LOGGING
  serviceUsbLog();
#endif
  const uint32_t duration = micros() - loopStartedAt;
  if (duration > longestCompletedLoopMicros) {
    longestCompletedLoopMicros = duration;
    TTNSemtechUDPClient::retainLoopMaximum(duration);
  }
  if (audioHeartbeat.fresh(time_us_64(), 3000000UL)) {
    rp2040.wdt_reset();
    PicoCrashDiagnostics::arm();
    audioStallReported = false;
  } else if (!audioStallReported) {
    LOG_PRINTLN(F("Core 1 heartbeat stale; withholding watchdog feed for automatic recovery"));
    audioStallReported = true;
  }
}
