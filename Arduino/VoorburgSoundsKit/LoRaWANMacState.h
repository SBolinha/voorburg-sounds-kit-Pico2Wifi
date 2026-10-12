#ifndef VOORBURG_LORAWAN_MAC_STATE_H
#define VOORBURG_LORAWAN_MAC_STATE_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

struct LoRaWANRadioSettings {
  uint32_t frequencyHz;
  uint32_t bandwidthHz;
  uint8_t spreadingFactor;
  uint8_t channelIndex;
};

struct LoRaWANDownlinkSettings {
  LoRaWANRadioSettings radio;
  uint32_t timestamp;
  uint32_t airtimeMicros;
};

// EU863-870 software radio state for the MAC requests used by this endpoint.
// DR7 uses FSK and is deliberately outside this LoRa-only implementation.
class LoRaWANMacState {
public:
  static const uint8_t CHANNEL_COUNT = 16;
  static const uint8_t ANSWER_CAPACITY = 32;
  static const uint32_t SNAPSHOT_VERSION = 1;
  static const uint32_t WINDOW_TOLERANCE_US = 50000UL;
  static const uint32_t DELIVERY_TOLERANCE_US = 250000UL;

  struct Channel {
    uint32_t frequencyHz;
    uint8_t minDataRate;
    uint8_t maxDataRate;
  };

  struct Answer {
    uint32_t id;
    uint8_t bytes[3];
    uint8_t length;
    uint8_t key; // Channel index for NewChannelAns; zero for other answers.
    uint8_t sticky;
    uint8_t sent;
  };

  // Plain structures can be included in the sketch's checksummed RAM journal.
  // Receive-cycle timestamps are excluded because the clock resets on reboot.
  struct Snapshot {
    uint32_t version;
    uint32_t nextAnswerId;
    uint32_t rx2FrequencyHz;
    Channel channels[CHANNEL_COUNT];
    Answer answers[ANSWER_CAPACITY];
    uint8_t rx2DataRate;
    uint8_t rx1Offset;
    uint8_t rxDelaySeconds;
    uint8_t uplinkDataRate;
    uint8_t nextChannel;
    uint8_t answerCount;
  };

  struct AnswerBatch {
    uint32_t ids[15];
    uint8_t count;
    uint8_t byteLength;
  };

  LoRaWANMacState() {
    LoRaWANRadioSettings initial = {868100000UL, 125000UL, 7, 0};
    reset(initial);
  }

  void reset(const LoRaWANRadioSettings &initial) {
    memset(&state_, 0, sizeof(state_));
    state_.version = SNAPSHOT_VERSION;
    state_.nextAnswerId = 1;
    state_.rx2FrequencyHz = 869525000UL;
    state_.rx2DataRate = 0;
    state_.rxDelaySeconds = 1;
    for (uint8_t i = 0; i < 3; ++i) {
      state_.channels[i].frequencyHz = defaultFrequency(i);
      state_.channels[i].maxDataRate = 5;
    }
    uint8_t dataRate = 5;
    radioDataRate(initial, dataRate);
    state_.uplinkDataRate = dataRate;
    const uint32_t frequency = validFrequency(initial.frequencyHz)
      ? initial.frequencyHz : defaultFrequency(0);
    uint8_t selected = CHANNEL_COUNT;
    for (uint8_t i = 0; i < 3; ++i) {
      if (state_.channels[i].frequencyHz == frequency && dataRate <= 5) {
        selected = i;
        break;
      }
    }
    if (selected == CHANNEL_COUNT) {
      selected = initial.channelIndex >= 3 && initial.channelIndex < CHANNEL_COUNT
        ? initial.channelIndex : 3;
      state_.channels[selected].frequencyHz = frequency;
      state_.channels[selected].maxDataRate = 6;
    }
    state_.nextChannel = selected;
    cycleValid_ = false;
  }

  Snapshot snapshot() const { return state_; }

  bool restore(const Snapshot &saved) {
    if (!validSnapshot(saved)) return false;
    state_ = saved;
    cycleValid_ = false;
    return true;
  }

  LoRaWANRadioSettings nextUplinkRadio() {
    for (uint8_t pass = 0; pass < 2; ++pass) {
      for (uint8_t n = 0; n < CHANNEL_COUNT; ++n) {
        const uint8_t index = static_cast<uint8_t>(
          (state_.nextChannel + n) % CHANNEL_COUNT);
        const Channel &candidate = state_.channels[index];
        if (!candidate.frequencyHz || state_.uplinkDataRate < candidate.minDataRate ||
            state_.uplinkDataRate > candidate.maxDataRate) continue;
        state_.nextChannel = static_cast<uint8_t>((index + 1) % CHANNEL_COUNT);
        return radioForDataRate(candidate.frequencyHz, state_.uplinkDataRate, index);
      }
      // Default channels always support DR5, including after removal of a
      // configured DR6 channel. This keeps the returned radio usable.
      state_.uplinkDataRate = 5;
    }
    return radioForDataRate(defaultFrequency(0), 5, 0);
  }

  void noteUplink(uint32_t timestamp, const LoRaWANRadioSettings &radio) {
    uint8_t uplinkDataRate = 0;
    cycleValid_ = validFrequency(radio.frequencyHz) &&
      radioDataRate(radio, uplinkDataRate);
    if (!cycleValid_) return;
    const uint8_t rx1DataRate = uplinkDataRate > state_.rx1Offset
      ? static_cast<uint8_t>(uplinkDataRate - state_.rx1Offset) : 0;
    rx1Radio_ = radioForDataRate(radio.frequencyHz, rx1DataRate, radio.channelIndex);
    rx2Radio_ = radioForDataRate(state_.rx2FrequencyHz, state_.rx2DataRate, 0);
    rx1Timestamp_ = timestamp + static_cast<uint32_t>(state_.rxDelaySeconds) * 1000000UL;
    rx2Timestamp_ = rx1Timestamp_ + 1000000UL;
  }

  bool acceptsDownlink(const LoRaWANDownlinkSettings &settings,
                       uint32_t nowMicros) const {
    if (!cycleValid_ || !settings.airtimeMicros || settings.airtimeMicros > 60000000UL)
      return false;
    const bool rx1 = nearTimestamp(settings.timestamp, rx1Timestamp_) &&
      sameRadio(settings.radio, rx1Radio_);
    const bool rx2 = nearTimestamp(settings.timestamp, rx2Timestamp_) &&
      sameRadio(settings.radio, rx2Radio_);
    if (!rx1 && !rx2) return false;
    const int32_t deliveredAfterEnd = static_cast<int32_t>(
      nowMicros - (settings.timestamp + settings.airtimeMicros));
    return deliveredAfterEnd >= 0 &&
      static_cast<uint32_t>(deliveredAfterEnd) <= DELIVERY_TOLERANCE_US;
  }

  // Invoke only after Class-A admission and DevAddr/MIC/counter validation,
  // before processing that downlink's new commands. Newly queued obligations
  // therefore cannot be cleared by the downlink which created them.
  void onValidDownlink() {
    for (uint8_t i = 0; i < state_.answerCount;) {
      if (state_.answers[i].sticky && state_.answers[i].sent) removeAnswer(state_, i);
      else ++i;
    }
    cycleValid_ = false;
  }

  // A stream is a transaction: malformed, unsupported or unqueueable requests
  // leave both negotiated parameters and answer obligations unchanged.
  bool processCommands(const uint8_t *commands, size_t length,
                       uint8_t battery, int8_t margin) {
    if ((!commands && length) || margin < -32 || margin > 31)
      return false;
    size_t offset = 0;
    while (offset < length) {
      const uint8_t cid = commands[offset++];
      const size_t payloadLength = commandPayloadLength(cid);
      if (payloadLength == SIZE_MAX || length - offset < payloadLength) return false;
      offset += payloadLength;
    }
    Snapshot proposed = state_;
    offset = 0;
    while (offset < length) {
      const uint8_t cid = commands[offset++];
      uint8_t answer[3] = {cid, 0, 0};
      uint8_t answerLength = 0;
      uint8_t key = 0;
      bool sticky = false;
      if (cid == 0x05) {
        const uint8_t settings = commands[offset++];
        const uint32_t frequency = read24(commands + offset) * 100UL;
        offset += 3;
        const uint8_t rx1Offset = (settings >> 4) & 0x07;
        const uint8_t rx2DataRate = settings & 0x0F;
        const bool frequencyAck = validFrequency(frequency);
        const bool rateAck = rx2DataRate <= 6;
        const bool offsetAck = rx1Offset <= 5;
        answer[1] = static_cast<uint8_t>((frequencyAck ? 1 : 0) |
          (rateAck ? 2 : 0) | (offsetAck ? 4 : 0));
        answerLength = 2;
        sticky = true;
        if (frequencyAck && rateAck && offsetAck) {
          proposed.rx2FrequencyHz = frequency;
          proposed.rx2DataRate = rx2DataRate;
          proposed.rx1Offset = rx1Offset;
        }
      } else if (cid == 0x06) {
        answer[1] = battery;
        answer[2] = static_cast<uint8_t>(margin) & 0x3F;
        answerLength = 3;
      } else if (cid == 0x07) {
        key = commands[offset++];
        const uint32_t frequency = read24(commands + offset) * 100UL;
        offset += 3;
        const uint8_t range = commands[offset++];
        const uint8_t minimum = range & 0x0F;
        const uint8_t maximum = range >> 4;
        const bool frequencyAck = key >= 3 && key < CHANNEL_COUNT &&
          (!frequency || validFrequency(frequency));
        const bool rateAck = minimum <= maximum && maximum <= 6;
        answer[1] = static_cast<uint8_t>((frequencyAck ? 1 : 0) | (rateAck ? 2 : 0));
        answerLength = 2;
        if (frequencyAck && rateAck) {
          proposed.channels[key].frequencyHz = frequency;
          proposed.channels[key].minDataRate = minimum;
          proposed.channels[key].maxDataRate = maximum;
        }
      } else { // RxTimingSetupReq (0x08).
        proposed.rxDelaySeconds = commands[offset++] & 0x0F;
        if (!proposed.rxDelaySeconds) proposed.rxDelaySeconds = 1;
        answerLength = 1;
        sticky = true;
      }
      if (!queueAnswer(proposed, answer, answerLength, key, sticky)) return false;
    }
    state_ = proposed;
    return true;
  }

  size_t prepareAnswers(uint8_t out[15], AnswerBatch &batch) const {
    memset(&batch, 0, sizeof(batch));
    if (!out) return 0;
    for (uint8_t sentPass = 0; sentPass < 2; ++sentPass) {
      for (uint8_t i = 0; i < state_.answerCount; ++i) {
        const Answer &answer = state_.answers[i];
        if (answer.sent != sentPass || batch.byteLength + answer.length > 15) continue;
        memcpy(out + batch.byteLength, answer.bytes, answer.length);
        batch.ids[batch.count++] = answer.id;
        batch.byteLength = static_cast<uint8_t>(batch.byteLength + answer.length);
      }
    }
    return batch.byteLength;
  }

  void answersSubmitted(const AnswerBatch &batch) {
    if (batch.count > 15 || batch.byteLength > 15) return;
    for (uint8_t n = 0; n < batch.count; ++n) {
      for (uint8_t i = 0; i < state_.answerCount; ++i) {
        Answer &answer = state_.answers[i];
        if (answer.id != batch.ids[n]) continue;
        if (answer.sticky) answer.sent = 1;
        else removeAnswer(state_, i);
        break;
      }
    }
  }

  uint32_t rx2FrequencyHz() const { return state_.rx2FrequencyHz; }
  uint8_t rx2DataRate() const { return state_.rx2DataRate; }
  uint8_t rx1Offset() const { return state_.rx1Offset; }
  uint8_t rxDelaySeconds() const { return state_.rxDelaySeconds; }
  const Channel &channel(uint8_t index) const {
    return state_.channels[index < CHANNEL_COUNT ? index : 0];
  }
  uint8_t pendingAnswerCount() const { return state_.answerCount; }

  static bool radioDataRate(const LoRaWANRadioSettings &radio, uint8_t &rate) {
    if (radio.bandwidthHz == 125000UL && radio.spreadingFactor >= 7 &&
        radio.spreadingFactor <= 12) {
      rate = static_cast<uint8_t>(12 - radio.spreadingFactor);
      return true;
    }
    if (radio.bandwidthHz == 250000UL && radio.spreadingFactor == 7) {
      rate = 6;
      return true;
    }
    return false;
  }

private:
  Snapshot state_;
  bool cycleValid_;
  LoRaWANRadioSettings rx1Radio_;
  LoRaWANRadioSettings rx2Radio_;
  uint32_t rx1Timestamp_;
  uint32_t rx2Timestamp_;

  static uint32_t defaultFrequency(uint8_t index) {
    return 868100000UL + static_cast<uint32_t>(index) * 200000UL;
  }
  static bool validFrequency(uint32_t frequency) {
    return frequency >= 863000000UL && frequency <= 870000000UL;
  }
  static uint32_t read24(const uint8_t *bytes) {
    return static_cast<uint32_t>(bytes[0]) |
      (static_cast<uint32_t>(bytes[1]) << 8) |
      (static_cast<uint32_t>(bytes[2]) << 16);
  }
  static size_t commandPayloadLength(uint8_t cid) {
    switch (cid) {
      case 0x05: return 4;
      case 0x06: return 0;
      case 0x07: return 5;
      case 0x08: return 1;
      default: return SIZE_MAX;
    }
  }
  static LoRaWANRadioSettings radioForDataRate(uint32_t frequency,
                                              uint8_t rate, uint8_t channel) {
    LoRaWANRadioSettings radio = {};
    radio.frequencyHz = frequency;
    radio.bandwidthHz = rate == 6 ? 250000UL : 125000UL;
    radio.spreadingFactor = rate == 6 ? 7 : static_cast<uint8_t>(12 - rate);
    radio.channelIndex = channel;
    return radio;
  }
  static bool sameRadio(const LoRaWANRadioSettings &left,
                        const LoRaWANRadioSettings &right) {
    return left.frequencyHz == right.frequencyHz &&
      left.bandwidthHz == right.bandwidthHz &&
      left.spreadingFactor == right.spreadingFactor;
  }
  static bool nearTimestamp(uint32_t actual, uint32_t expected) {
    const int32_t delta = static_cast<int32_t>(actual - expected);
    return delta >= -static_cast<int32_t>(WINDOW_TOLERANCE_US) &&
      delta <= static_cast<int32_t>(WINDOW_TOLERANCE_US);
  }
  static void removeAnswer(Snapshot &state, uint8_t index) {
    --state.answerCount;
    memmove(state.answers + index, state.answers + index + 1,
      static_cast<size_t>(state.answerCount - index) * sizeof(Answer));
    memset(state.answers + state.answerCount, 0, sizeof(Answer));
  }
  static uint32_t allocateAnswerId(Snapshot &state) {
    while (true) {
      const uint32_t candidate = state.nextAnswerId++;
      if (!candidate) continue;
      bool used = false;
      for (uint8_t i = 0; i < state.answerCount; ++i)
        if (state.answers[i].id == candidate) used = true;
      if (!used) return candidate;
    }
  }
  static bool queueAnswer(Snapshot &state, const uint8_t *bytes, uint8_t length,
                          uint8_t key, bool sticky) {
    uint8_t index = 0;
    while (index < state.answerCount &&
           (state.answers[index].bytes[0] != bytes[0] || state.answers[index].key != key))
      ++index;
    if (index == state.answerCount && state.answerCount == ANSWER_CAPACITY) return false;
    const uint32_t id = allocateAnswerId(state);
    if (index == state.answerCount) ++state.answerCount;
    Answer &answer = state.answers[index];
    memset(&answer, 0, sizeof(answer));
    answer.id = id;
    memcpy(answer.bytes, bytes, length);
    answer.length = length;
    answer.key = key;
    answer.sticky = sticky ? 1 : 0;
    return true;
  }
  static bool validSnapshot(const Snapshot &state) {
    if (state.version != SNAPSHOT_VERSION || !validFrequency(state.rx2FrequencyHz) ||
        state.rx2DataRate > 6 || state.rx1Offset > 5 || !state.rxDelaySeconds ||
        state.rxDelaySeconds > 15 || state.uplinkDataRate > 6 ||
        state.nextChannel >= CHANNEL_COUNT || state.answerCount > ANSWER_CAPACITY)
      return false;
    for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
      const Channel &channel = state.channels[i];
      if (channel.minDataRate > channel.maxDataRate || channel.maxDataRate > 6 ||
          (channel.frequencyHz && !validFrequency(channel.frequencyHz))) return false;
      if (i < 3 && (channel.frequencyHz != defaultFrequency(i) ||
                    channel.minDataRate != 0 || channel.maxDataRate != 5)) return false;
    }
    for (uint8_t i = 0; i < state.answerCount; ++i) {
      const Answer &answer = state.answers[i];
      const uint8_t cid = answer.bytes[0];
      const uint8_t expectedLength = cid == 0x05 || cid == 0x07 ? 2 :
        (cid == 0x06 ? 3 : (cid == 0x08 ? 1 : 0));
      const bool expectedSticky = cid == 0x05 || cid == 0x08;
      if (!answer.id || !expectedLength || answer.length != expectedLength ||
          answer.sticky != static_cast<uint8_t>(expectedSticky) || answer.sent > 1 ||
          (!expectedSticky && answer.sent) || (cid != 0x07 && answer.key)) return false;
      if ((cid == 0x05 && answer.bytes[1] > 7) ||
          (cid == 0x07 && answer.bytes[1] > 3) ||
          (cid == 0x06 && answer.bytes[2] > 63)) return false;
      for (uint8_t j = 0; j < i; ++j)
        if (state.answers[j].id == answer.id ||
            (state.answers[j].bytes[0] == cid && state.answers[j].key == answer.key)) return false;
    }
    return true;
  }
};

#endif
