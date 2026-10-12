#include "../Arduino/VoorburgSoundsKit/LoRaWANMacState.h"
#include <assert.h>
#include <stdio.h>
#include <type_traits>
#include <vector>

static_assert(std::is_trivial<LoRaWANMacState::Snapshot>::value,
              "Snapshot must remain suitable for the retained RAM journal");
static_assert(std::is_standard_layout<LoRaWANMacState::Snapshot>::value,
              "Snapshot must have a plain memory layout");
static_assert(std::is_trivial<LoRaWANMacState::AnswerBatch>::value,
              "AnswerBatch must remain suitable for the retained RAM journal");

typedef std::vector<uint8_t> Bytes;

static void appendFrequency(Bytes &bytes, uint32_t frequency) {
  const uint32_t units = frequency / 100;
  bytes.push_back(static_cast<uint8_t>(units));
  bytes.push_back(static_cast<uint8_t>(units >> 8));
  bytes.push_back(static_cast<uint8_t>(units >> 16));
}

static Bytes rxParam(uint8_t settings, uint32_t frequency) {
  Bytes bytes = {0x05, settings};
  appendFrequency(bytes, frequency);
  return bytes;
}

static Bytes newChannel(uint8_t index, uint32_t frequency, uint8_t range = 0x50) {
  Bytes bytes = {0x07, index};
  appendFrequency(bytes, frequency);
  bytes.push_back(range);
  return bytes;
}

static void append(Bytes &left, const Bytes &right) {
  left.insert(left.end(), right.begin(), right.end());
}

static bool process(LoRaWANMacState &state, const Bytes &bytes,
                    uint8_t battery = 0, int8_t margin = 29) {
  return state.processCommands(bytes.data(), bytes.size(), battery, margin);
}

static Bytes prepared(const LoRaWANMacState &state, LoRaWANMacState::AnswerBatch &batch) {
  uint8_t out[15] = {};
  const size_t length = state.prepareAnswers(out, batch);
  assert(length == batch.byteLength);
  return Bytes(out, out + length);
}

static LoRaWANRadioSettings radio(uint32_t frequency = 868100000UL,
                                  uint8_t sf = 7, uint32_t bandwidth = 125000UL) {
  LoRaWANRadioSettings result = {frequency, bandwidth, sf, 0};
  return result;
}

static LoRaWANDownlinkSettings downlink(uint32_t timestamp,
                                       LoRaWANRadioSettings settings = radio(),
                                       uint32_t airtime = 100000UL) {
  LoRaWANDownlinkSettings result = {settings, timestamp, airtime};
  return result;
}

static void testObservedTtnBatch() {
  LoRaWANMacState state;
  Bytes requests = rxParam(0x03, 869525000UL);
  for (uint8_t index = 3; index < 8; ++index)
    append(requests, newChannel(index, 867100000UL + (index - 3) * 200000UL));
  append(requests, Bytes{0x08, 0x01, 0x06});
  assert(process(state, requests));
  assert(state.rx2DataRate() == 3 && state.pendingAnswerCount() == 8);
  LoRaWANMacState::AnswerBatch first = {};
  const Bytes expectedFirst = {0x05, 0x07, 0x07, 0x03, 0x07, 0x03, 0x07,
                               0x03, 0x07, 0x03, 0x07, 0x03, 0x08};
  assert(prepared(state, first) == expectedFirst);
  assert(first.count == 7 && first.byteLength == 13);
  state.answersSubmitted(first);
  assert(state.pendingAnswerCount() == 3);
  LoRaWANMacState::AnswerBatch second = {};
  assert(prepared(state, second) == (Bytes{0x06, 0x00, 0x1D, 0x05, 0x07, 0x08}));
  state.answersSubmitted(second);
  assert(state.pendingAnswerCount() == 2);
  LoRaWANMacState::AnswerBatch repeat = {};
  assert(prepared(state, repeat) == (Bytes{0x05, 0x07, 0x08}));
  state.onValidDownlink();
  assert(state.pendingAnswerCount() == 0);

  LoRaWANMacState repeated;
  assert(process(repeated, requests));
  LoRaWANMacState::AnswerBatch preparedBeforeRepeat = {};
  prepared(repeated, preparedBeforeRepeat);
  repeated.onValidDownlink();
  assert(process(repeated, requests));
  assert(repeated.pendingAnswerCount() == 8); // Every channel request is deduplicated.
  repeated.answersSubmitted(preparedBeforeRepeat);
  assert(repeated.pendingAnswerCount() == 8); // Older frame cannot consume newer channel answers.
  LoRaWANMacState::AnswerBatch newest = {};
  assert(prepared(repeated, newest) == expectedFirst);
}

static void testStickyAnswersAndReplacements() {
  LoRaWANMacState state;
  assert(process(state, Bytes{0x08, 0x05}));
  state.onValidDownlink(); // An answer not yet sent must survive a downlink.
  assert(state.pendingAnswerCount() == 1);
  LoRaWANMacState::AnswerBatch oldTiming = {};
  assert(prepared(state, oldTiming) == (Bytes{0x08}));
  assert(process(state, Bytes{0x08, 0x06}));
  state.answersSubmitted(oldTiming); // A prepared older frame cannot consume the replacement.
  state.onValidDownlink();
  assert(state.pendingAnswerCount() == 1 && state.rxDelaySeconds() == 6);
  LoRaWANMacState::AnswerBatch newTiming = {};
  prepared(state, newTiming);
  assert(newTiming.ids[0] != oldTiming.ids[0]);
  state.answersSubmitted(newTiming);
  state.onValidDownlink();
  assert(state.pendingAnswerCount() == 0);
  assert(process(state, Bytes{0x08, 0x06})); // Repeated request after reception creates a new obligation.
  assert(state.pendingAnswerCount() == 1);

  LoRaWANMacState status;
  assert(process(status, Bytes{0x06}, 0, -32));
  LoRaWANMacState::AnswerBatch oldStatus = {};
  assert(prepared(status, oldStatus) == (Bytes{0x06, 0x00, 0x20}));
  assert(process(status, Bytes{0x06}, 254, 31));
  status.answersSubmitted(oldStatus);
  LoRaWANMacState::AnswerBatch latestStatus = {};
  assert(prepared(status, latestStatus) == (Bytes{0x06, 0xFE, 0x1F}));
  status.answersSubmitted(latestStatus);
  assert(status.pendingAnswerCount() == 0);
}

static void testRxParameterValidation() {
  LoRaWANMacState state;
  assert(process(state, rxParam(0x23, 869525000UL)));
  assert(state.rx1Offset() == 2 && state.rx2DataRate() == 3);
  LoRaWANMacState::AnswerBatch batch = {};
  assert(prepared(state, batch) == (Bytes{0x05, 0x07}));
  assert(process(state, rxParam(0x67, 862900000UL)));
  assert(prepared(state, batch) == (Bytes{0x05, 0x00}));
  assert(state.rx1Offset() == 2 && state.rx2DataRate() == 3 &&
         state.rx2FrequencyHz() == 869525000UL);
  assert(process(state, rxParam(0x07, 869525000UL))); // DR7/FSK must be rejected.
  assert(prepared(state, batch) == (Bytes{0x05, 0x05}));
  assert(process(state, rxParam(0x56, 863000000UL)));
  assert(state.rx1Offset() == 5 && state.rx2DataRate() == 6);
  assert(process(state, Bytes{0x08, 0xF0})); // RFU ignored, zero delay means one second.
  assert(state.rxDelaySeconds() == 1);
  assert(process(state, Bytes{0x08, 0x0F}));
  assert(state.rxDelaySeconds() == 15);
}

static void testChannelsAndRoundRobin() {
  LoRaWANMacState state;
  for (uint8_t i = 0; i < 3; ++i) {
    assert(state.channel(i).frequencyHz == 868100000UL + i * 200000UL);
    assert(state.channel(i).minDataRate == 0 && state.channel(i).maxDataRate == 5);
    assert(process(state, newChannel(i, 0)));
    assert(state.channel(i).frequencyHz == 868100000UL + i * 200000UL);
  }
  assert(process(state, newChannel(3, 867100000UL)));
  assert(process(state, newChannel(4, 867300000UL, 0x43))); // DR3..4, skip at DR5.
  for (uint8_t i = 0; i < 8; ++i) {
    const LoRaWANRadioSettings settings = state.nextUplinkRadio();
    assert(settings.channelIndex == i % 4);
    assert(settings.spreadingFactor == 7 && settings.bandwidthHz == 125000UL);
    assert(settings.frequencyHz == state.channel(settings.channelIndex).frequencyHz);
  }
  assert(process(state, newChannel(3, 0)));
  for (uint8_t i = 0; i < 6; ++i) assert(state.nextUplinkRadio().channelIndex < 3);
  assert(process(state, newChannel(16, 867100000UL)));
  assert(process(state, newChannel(5, 867500000UL, 0x70)));
  assert(state.channel(5).frequencyHz == 0);
  assert(process(state, newChannel(5, 867500000UL, 0x23)));
  assert(state.channel(5).frequencyHz == 0);

  LoRaWANMacState configured;
  configured.reset(radio(867900000UL));
  assert(configured.nextUplinkRadio().frequencyHz == 867900000UL);
  configured.reset(radio(868100000UL, 7, 250000UL));
  assert(configured.nextUplinkRadio().bandwidthHz == 250000UL);
  assert(process(configured, newChannel(3, 0)));
  const LoRaWANRadioSettings fallback = configured.nextUplinkRadio();
  assert(fallback.spreadingFactor == 7 && fallback.bandwidthHz == 125000UL);
  configured.reset(radio(868100000UL, 6, 125000UL));
  assert(configured.nextUplinkRadio().spreadingFactor == 7);
}

static void testAtomicStreamsAndQueueCapacity() {
  LoRaWANMacState state;
  const LoRaWANMacState::Snapshot original = state.snapshot();
  Bytes truncated = rxParam(0x13, 869100000UL);
  append(truncated, Bytes{0x07, 0x03});
  assert(!process(state, truncated));
  LoRaWANMacState::Snapshot actual = state.snapshot();
  assert(memcmp(&original, &actual, sizeof(actual)) == 0);
  assert(!process(state, Bytes{0x08, 0x05, 0xFF}));
  assert(!state.processCommands(nullptr, 1, 0, 0));
  LoRaWANMacState unknownBattery;
  assert(process(unknownBattery, Bytes{0x06}, 255, 0));
  LoRaWANMacState::AnswerBatch batteryBatch = {};
  assert(prepared(unknownBattery, batteryBatch) == Bytes({0x06, 0xFF, 0x00}));
  LoRaWANMacState restoredBattery;
  assert(restoredBattery.restore(unknownBattery.snapshot()));
  assert(!process(state, Bytes{0x06}, 0, 32));
  assert(state.processCommands(nullptr, 0, 0, 0));

  for (uint8_t index = 16; index < 16 + LoRaWANMacState::ANSWER_CAPACITY; ++index)
    assert(process(state, newChannel(index, 867100000UL)));
  assert(state.pendingAnswerCount() == LoRaWANMacState::ANSWER_CAPACITY);
  const LoRaWANMacState::Snapshot full = state.snapshot();
  assert(!process(state, rxParam(0x23, 869100000UL)));
  actual = state.snapshot();
  assert(memcmp(&full, &actual, sizeof(actual)) == 0);
  assert(process(state, newChannel(16, 867300000UL))); // Replacement does not need a queue slot.
  assert(state.pendingAnswerCount() == LoRaWANMacState::ANSWER_CAPACITY);
  LoRaWANMacState::AnswerBatch batch = {};
  assert(prepared(state, batch).size() == 14);
  state.answersSubmitted(batch);
  assert(process(state, rxParam(0x23, 869100000UL)));
  assert(state.rx2FrequencyHz() == 869100000UL);
}

static void testReceiveWindows() {
  LoRaWANMacState state;
  const LoRaWANRadioSettings uplink = radio();
  LoRaWANDownlinkSettings settings = downlink(2000000UL);
  assert(!state.acceptsDownlink(settings, 2100000UL));
  state.noteUplink(1000000UL, uplink);
  assert(state.acceptsDownlink(settings, 2100000UL));
  assert(!state.acceptsDownlink(settings, 2099999UL));
  assert(state.acceptsDownlink(settings, 2350000UL));
  assert(!state.acceptsDownlink(settings, 2350001UL));
  settings.timestamp = 1950000UL;
  assert(state.acceptsDownlink(settings, settings.timestamp + settings.airtimeMicros));
  settings.timestamp = 1949999UL;
  assert(!state.acceptsDownlink(settings, settings.timestamp + settings.airtimeMicros));
  settings.timestamp = 2050000UL;
  assert(state.acceptsDownlink(settings, settings.timestamp + settings.airtimeMicros));
  ++settings.timestamp;
  assert(!state.acceptsDownlink(settings, settings.timestamp + settings.airtimeMicros));
  settings = downlink(3000000UL, radio(869525000UL, 12));
  assert(state.acceptsDownlink(settings, 3100000UL));
  settings.radio.spreadingFactor = 9;
  assert(!state.acceptsDownlink(settings, 3100000UL));
  settings = downlink(2000000UL, radio(868300000UL));
  assert(!state.acceptsDownlink(settings, 2100000UL));
  settings = downlink(1000000UL); // An immediate response at uplink time is outside both windows.
  assert(!state.acceptsDownlink(settings, 1100000UL));
  settings = downlink(2000000UL, radio(), 0);
  assert(!state.acceptsDownlink(settings, 2000000UL));
  state.onValidDownlink();
  settings = downlink(3000000UL, radio(869525000UL, 12));
  assert(!state.acceptsDownlink(settings, 3100000UL)); // RX2 closes after accepted RX1.

  assert(process(state, rxParam(0x06, 869525000UL)));
  state.noteUplink(5000000UL, radio(868100000UL, 7, 250000UL));
  settings = downlink(6000000UL, radio(868100000UL, 7, 250000UL));
  assert(state.acceptsDownlink(settings, 6100000UL));
  settings = downlink(7000000UL, radio(869525000UL, 7, 250000UL));
  assert(state.acceptsDownlink(settings, 7100000UL));
  settings.radio.bandwidthHz = 125000UL;
  assert(!state.acceptsDownlink(settings, 7100000UL));
}

static void testCycleSnapshotsAndTimestampWrap() {
  LoRaWANMacState state;
  state.noteUplink(1000000UL, radio());
  assert(process(state, rxParam(0x23, 869100000UL)));
  assert(process(state, Bytes{0x08, 0x05}));
  LoRaWANDownlinkSettings previous = downlink(2000000UL);
  assert(state.acceptsDownlink(previous, 2100000UL)); // Existing cycle retains its prior configuration.
  state.noteUplink(10000000UL, radio());
  LoRaWANDownlinkSettings rx1 = downlink(15000000UL, radio(868100000UL, 9));
  assert(state.acceptsDownlink(rx1, 15100000UL));
  LoRaWANDownlinkSettings rx2 = downlink(16000000UL, radio(869100000UL, 9));
  assert(state.acceptsDownlink(rx2, 16100000UL));
  state.noteUplink(10000000UL, radio(868100000UL, 12));
  rx1.radio.spreadingFactor = 12; // Offset is clamped at DR0.
  assert(state.acceptsDownlink(rx1, 15100000UL));

  LoRaWANMacState wrapped;
  const uint32_t beforeWrap = 0xFFF00000UL;
  wrapped.noteUplink(beforeWrap, radio());
  const uint32_t target = static_cast<uint32_t>(beforeWrap + 2000000UL);
  LoRaWANDownlinkSettings window = downlink(target, radio(869525000UL, 12));
  assert(wrapped.acceptsDownlink(window, target + 100000UL));
  assert(!wrapped.acceptsDownlink(window, target + 350001UL));
  window.timestamp = target - 50000UL;
  assert(wrapped.acceptsDownlink(window, window.timestamp + 100000UL));
}

static void testSnapshotRestoreAndIdWrap() {
  LoRaWANMacState original;
  assert(process(original, rxParam(0x23, 869100000UL)));
  assert(process(original, Bytes{0x08, 0x05, 0x06}));
  assert(process(original, newChannel(3, 867100000UL)));
  LoRaWANMacState::AnswerBatch sent = {};
  prepared(original, sent);
  original.answersSubmitted(sent);
  assert(process(original, Bytes{0x06}));
  original.nextUplinkRadio();
  original.noteUplink(1000000UL, radio());
  const LoRaWANMacState::Snapshot saved = original.snapshot();
  LoRaWANMacState restored;
  assert(restored.restore(saved));
  assert(restored.rx2FrequencyHz() == 869100000UL && restored.rx2DataRate() == 3 &&
         restored.rx1Offset() == 2 && restored.rxDelaySeconds() == 5);
  assert(restored.channel(3).frequencyHz == 867100000UL);
  assert(restored.nextUplinkRadio().channelIndex == 1);
  LoRaWANMacState::AnswerBatch replay = {};
  assert(prepared(restored, replay) == (Bytes{0x06, 0x00, 0x1D, 0x05, 0x07, 0x08}));
  assert(!restored.acceptsDownlink(downlink(6000000UL, radio(868100000UL, 9)), 6100000UL));
  restored.onValidDownlink();
  assert(restored.pendingAnswerCount() == 1); // Unsent DevStatus survives; sent sticky answers clear.
  LoRaWANMacState::Snapshot malformed = saved;
  malformed.channels[0].frequencyHz = 0;
  assert(!restored.restore(malformed));
  malformed = saved;
  malformed.answerCount = LoRaWANMacState::ANSWER_CAPACITY + 1;
  assert(!restored.restore(malformed));

  LoRaWANMacState pendingFrame;
  assert(process(pendingFrame, Bytes{0x06, 0x08, 0x03}));
  LoRaWANMacState::AnswerBatch retainedBatch = {};
  prepared(pendingFrame, retainedBatch);
  const LoRaWANMacState::Snapshot beforeSubmission = pendingFrame.snapshot();
  LoRaWANMacState rebooted;
  assert(rebooted.restore(beforeSubmission));
  rebooted.answersSubmitted(retainedBatch); // The retained frame's IDs still identify its answers.
  assert(rebooted.pendingAnswerCount() == 1);
  rebooted.onValidDownlink();
  assert(rebooted.pendingAnswerCount() == 0);

  LoRaWANMacState wrappedIds;
  assert(process(wrappedIds, Bytes{0x08, 0x01})); // ID 1 remains queued.
  LoRaWANMacState::Snapshot wrapping = wrappedIds.snapshot();
  wrapping.nextAnswerId = UINT32_MAX;
  assert(wrappedIds.restore(wrapping));
  assert(process(wrappedIds, Bytes{0x06})); // ID UINT32_MAX.
  assert(process(wrappedIds, newChannel(3, 867100000UL))); // Skip zero and existing ID 1.
  const LoRaWANMacState::Snapshot ids = wrappedIds.snapshot();
  assert(ids.answers[0].id == 1 && ids.answers[1].id == UINT32_MAX && ids.answers[2].id == 2);
}

int main() {
  testObservedTtnBatch();
  testStickyAnswersAndReplacements();
  testRxParameterValidation();
  testChannelsAndRoundRobin();
  testAtomicStreamsAndQueueCapacity();
  testReceiveWindows();
  testCycleSnapshotsAndTimestampWrap();
  testSnapshotRestoreAndIdWrap();
  puts("LoRaWAN MAC state: 8 test groups passed");
  return 0;
}
