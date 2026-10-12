#ifndef VOORBURG_LORA_AIRTIME_H
#define VOORBURG_LORA_AIRTIME_H

#include <Arduino.h>
#include "LoRaWANMacState.h"

// EU868 LoRa downlink airtime: explicit header, no PHY CRC, eight-symbol preamble.
class LoRaAirtime {
public:
  static uint32_t estimateLoRaAirtimeMicros(size_t frameLength,
                                         const LoRaWANRadioSettings &radio) {
    const uint32_t sf = radio.spreadingFactor;
    const uint32_t bw = radio.bandwidthHz;
    if (sf < 7 || sf > 12 || (bw != 125000UL && bw != 250000UL && bw != 500000UL) ||
        frameLength == 0 || frameLength > 255) return 0;
    const uint32_t de = (1ULL << sf) * 1000000ULL >=
      static_cast<uint64_t>(bw) * 16000ULL ? 1 : 0;
    const int32_t numerator = static_cast<int32_t>(8 * frameLength) -
      static_cast<int32_t>(4 * sf) + 28;
    const uint32_t denominator = 4 * (sf - 2 * de);
    const uint32_t blocks = numerator > 0 ?
      (static_cast<uint32_t>(numerator) + denominator - 1) / denominator : 0;
    const uint64_t quarterSymbols = 4ULL * (8 + 8 + blocks * 5) + 17;
    const uint64_t denominatorUs = 4ULL * bw;
    const uint64_t airtimeUs =
      (quarterSymbols * (1ULL << sf) * 1000000ULL + denominatorUs - 1) /
      denominatorUs;
    return airtimeUs > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(airtimeUs);
  }

};
#endif
