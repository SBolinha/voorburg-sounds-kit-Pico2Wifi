#include <assert.h>
#include <stdio.h>
#include <vector>

// Public test keys. The private sketch settings are skipped by their guard.
#define VOORBURG_LORA_SETTINGS_H
#define USB_LOGGING 0
#define LORAWAN_DEVADDR_HEX "01020304"
#define LORAWAN_NWKSKEY_HEX "2B7E151628AED2A6ABF7158809CF4F3C"
#define LORAWAN_APPSKEY_HEX "00112233445566778899AABBCCDDEEFF"
#include "../Arduino/VoorburgSoundsKit/LoRaWANFrame.h"

static std::vector<uint8_t> hex(const char *text) {
  std::vector<uint8_t> out;
  for (size_t i = 0; text[i]; i += 2) {
    unsigned value = 0;
    assert(sscanf(text + i, "%2x", &value) == 1);
    out.push_back(static_cast<uint8_t>(value));
  }
  return out;
}

int main() {
  const std::vector<uint8_t> expected = hex(
    "40A0190B260D00190507070307030703070307030801C640B9BA0E3E2941CD");
  const std::vector<uint8_t> answers = hex("05070703070307030703070308");
  const uint8_t application[] = {0x44, 0x3C, 0x82, 0x21, 0x4E};
  uint8_t frame[255];
  size_t length = 0;
  assert(LoRaWANFrame::buildUplink(application, sizeof(application), 6400,
    frame, length, false, answers.data(), answers.size()));
  assert(length == expected.size() && memcmp(frame, expected.data(), length) == 0);

  // Timestamped application data, independently generated with OpenSSL's
  // AES-ECB and CMAC commands using the same public fixture keys.
  const uint8_t timestamped[] = {0x44, 0x3C, 0x82, 0x21, 0x4E,
                                0x00, 0x9F, 0xC8, 0x6A};
  const std::vector<uint8_t> expectedTimestamped =
    hex("40A0190B260001190141696DEF9CED351F6179965A78");
  assert(LoRaWANFrame::buildUplink(timestamped, sizeof(timestamped), 6401,
    frame, length));
  assert(length == expectedTimestamped.size() &&
         memcmp(frame, expectedTimestamped.data(), length) == 0);

  const std::vector<uint8_t> commands = hex(
    "0503D2AD840703184F84500704E85684500705B85E84500706886684500707586E8450080506");
  const std::vector<uint8_t> macDown = hex(
    "60A0190B260007000086E9CFD817AC463D7D9CBD9D7E4646EFC2435E042E056927E9F5BAAB70FB57EC7D61F3D6B968BE5AC84F");
  LoRaWANFrame::DownlinkInfo info = {};
  LoRaWANFrame::DownlinkRejectReason reason;
  LoRaWANFrame::CounterState counters = {};
  assert(LoRaWANFrame::receiveDownlink(macDown.data(), macDown.size(), info, &reason, &counters));
  assert(info.frameCounter == 7 && info.hasFPort && info.fPort == 0 && !info.duplicate);
  assert(info.payloadLength == commands.size() && memcmp(info.payload, commands.data(), commands.size()) == 0);
  assert(!LoRaWANFrame::receiveDownlink(macDown.data(), macDown.size(), info, &reason, &counters));
  assert(reason == LoRaWANFrame::DownlinkRejectReason::InvalidFrameCounter);

  const std::vector<uint8_t> confirmed = hex("A0A0190B2600080001BF70CC73560661A453");
  assert(LoRaWANFrame::receiveDownlink(confirmed.data(), confirmed.size(), info, &reason, &counters));
  assert(info.confirmed && !info.duplicate && info.frameCounter == 8);
  assert(info.payloadLength == sizeof(application) && memcmp(info.payload, application, sizeof(application)) == 0);
  LoRaWANFrame::CounterState restarted = counters;
  assert(LoRaWANFrame::receiveDownlink(confirmed.data(), confirmed.size(), info, &reason, &restarted));
  assert(info.confirmed && info.duplicate);
  std::vector<uint8_t> corrupt = confirmed;
  corrupt.back() ^= 1;
  assert(!LoRaWANFrame::receiveDownlink(corrupt.data(), corrupt.size(), info, &reason, &restarted));
  assert(reason == LoRaWANFrame::DownlinkRejectReason::MicMismatch && restarted.last == 8);
  corrupt = confirmed;
  corrupt[0] |= 1;
  assert(!LoRaWANFrame::receiveDownlink(corrupt.data(), corrupt.size(), info, &reason, &restarted));
  assert(reason == LoRaWANFrame::DownlinkRejectReason::UnsupportedFrameType);

  const std::vector<uint8_t> wrapped = hex("60A0190B2600000001B6168EED23DC2471E3");
  restarted.last = 65535;
  restarted.seen = true;
  assert(LoRaWANFrame::receiveDownlink(wrapped.data(), wrapped.size(), info, &reason, &restarted));
  assert(info.frameCounter == 65536);
  puts("LoRaWAN frames: independent AES/CMAC fixture, FOpts, decryption, restart counters and confirmed retransmission passed");
}
