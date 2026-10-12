/*
 * Minimal LoRaWAN 1.0.x ABP uplink encoder for the TTN virtual-gateway path.
 * Uses the maintained Arduino Cryptography Library for AES-128/CMAC building
 * blocks. Application bytes are passed through unchanged; the sketch carries
 * the original five measurement values followed by a UTC window timestamp.
 */
#ifndef VOORBURG_LORAWAN_FRAME_H
#define VOORBURG_LORAWAN_FRAME_H

#include <Arduino.h>
#include <string.h>
#include <AES.h>
#include "LoraSettings.h"
#if USB_LOGGING
#include "EventLog.h"
#endif

class LoRaWANFrame {
public:
  static const size_t MAX_FRAME_SIZE = 255;
  struct DownlinkInfo {
    uint8_t fPort;
    bool hasFPort;
    bool confirmed;
    bool duplicate;
    uint32_t frameCounter;
    uint8_t fOpts[15];
    size_t fOptsLength;
    uint8_t payload[242];
    size_t payloadLength;
  };

  // The caller journals this together with the negotiated MAC settings. A
  // watchdog restart must not make an already received downlink new again.
  struct CounterState {
    uint32_t last;
    bool seen;
  };

  enum class DownlinkRejectReason : uint8_t {
    None,
    InvalidLength,
    UnsupportedFrameType,
    InvalidSessionSettings,
    DevAddrMismatch,
    InvalidFrameLayout,
    InvalidFrameCounter,
    CryptoFailure,
    MicMismatch
  };

  static const char *downlinkRejectReasonName(DownlinkRejectReason reason) {
    switch (reason) {
      case DownlinkRejectReason::None: return "none";
      case DownlinkRejectReason::InvalidLength: return "invalid frame length";
      case DownlinkRejectReason::UnsupportedFrameType: return "unsupported LoRaWAN frame type";
      case DownlinkRejectReason::InvalidSessionSettings: return "invalid local session settings";
      case DownlinkRejectReason::DevAddrMismatch: return "DevAddr mismatch";
      case DownlinkRejectReason::InvalidFrameLayout: return "invalid FHDR/FPort layout";
      case DownlinkRejectReason::InvalidFrameCounter: return "replayed or invalid FCntDown";
      case DownlinkRejectReason::CryptoFailure: return "AES-CMAC/AES operation failed";
      case DownlinkRejectReason::MicMismatch: return "MIC mismatch";
    }
    return "unknown downlink rejection";
  }

  static bool buildUplink(const uint8_t *payload, size_t payloadLength,
                          uint32_t frameCounter, uint8_t *frame,
                          size_t &frameLength, bool acknowledgeDownlink = false,
                          const uint8_t *fOpts = nullptr, size_t fOptsLength = 0) {
    if (!payload || !frame || payloadLength == 0 || payloadLength > 51 ||
        fOptsLength > 15 || (fOptsLength && !fOpts)) return false;

    uint8_t nwkKey[16], appKey[16];
    uint32_t devAddr;
    if (!decodeHex(LORAWAN_NWKSKEY_HEX, nwkKey, sizeof(nwkKey)) ||
        !decodeHex(LORAWAN_APPSKEY_HEX, appKey, sizeof(appKey)) ||
        !decodeDevAddr(LORAWAN_DEVADDR_HEX, devAddr)) return false;

    // MHDR: unconfirmed data up, LoRaWAN R1.
    size_t p = 0;
    frame[p++] = 0x40;
    // FHDR: DevAddr is transmitted least-significant byte first.
    frame[p++] = static_cast<uint8_t>(devAddr);
    frame[p++] = static_cast<uint8_t>(devAddr >> 8);
    frame[p++] = static_cast<uint8_t>(devAddr >> 16);
    frame[p++] = static_cast<uint8_t>(devAddr >> 24);
    frame[p++] = static_cast<uint8_t>((acknowledgeDownlink ? 0x20 : 0x00) |
                                       (fOptsLength & 0x0F));
    frame[p++] = static_cast<uint8_t>(frameCounter);
    frame[p++] = static_cast<uint8_t>(frameCounter >> 8);
    if (fOptsLength) {
      memcpy(frame + p, fOpts, fOptsLength);
      p += fOptsLength;
    }
    frame[p++] = 0x01; // FPort: application data.

    AES128 appCipher;
    if (!appCipher.setKey(appKey, sizeof(appKey))) return false;
    uint8_t a[16] = {};
    uint8_t stream[16];
    a[0] = 0x01;
    a[5] = 0x00; // Uplink direction.
    a[6] = static_cast<uint8_t>(devAddr);
    a[7] = static_cast<uint8_t>(devAddr >> 8);
    a[8] = static_cast<uint8_t>(devAddr >> 16);
    a[9] = static_cast<uint8_t>(devAddr >> 24);
    a[10] = static_cast<uint8_t>(frameCounter);
    a[11] = static_cast<uint8_t>(frameCounter >> 8);
    a[12] = static_cast<uint8_t>(frameCounter >> 16);
    a[13] = static_cast<uint8_t>(frameCounter >> 24);
    a[15] = 1;
    for (size_t offset = 0; offset < payloadLength; offset += 16) {
      a[15] = static_cast<uint8_t>((offset / 16) + 1);
      appCipher.encryptBlock(stream, a);
      const size_t remaining = payloadLength - offset;
      const size_t blockLength = remaining < 16 ? remaining : 16;
      for (size_t i = 0; i < blockLength; ++i) frame[p++] = payload[offset + i] ^ stream[i];
    }

    // MIC = first four bytes of AES-CMAC(NwkSKey, B0 | MHDR..FRMPayload).
    uint8_t b0[16] = {};
    b0[0] = 0x49;
    b0[5] = 0x00; // Uplink direction.
    b0[6] = static_cast<uint8_t>(devAddr);
    b0[7] = static_cast<uint8_t>(devAddr >> 8);
    b0[8] = static_cast<uint8_t>(devAddr >> 16);
    b0[9] = static_cast<uint8_t>(devAddr >> 24);
    b0[10] = static_cast<uint8_t>(frameCounter);
    b0[11] = static_cast<uint8_t>(frameCounter >> 8);
    b0[12] = static_cast<uint8_t>(frameCounter >> 16);
    b0[13] = static_cast<uint8_t>(frameCounter >> 24);
    b0[15] = static_cast<uint8_t>(p);

    AES128 nwkCipher;
    if (!nwkCipher.setKey(nwkKey, sizeof(nwkKey))) return false;
    // Run the RFC 4493 known-answer test once before producing a live frame.
    // This prevents sending frames if the AES/CMAC path is malfunctioning.
    static bool cmacTested = false;
    static bool cmacTestPassed = false;
    if (!cmacTested) {
      cmacTestPassed = cmacKnownAnswerTest();
      cmacTested = true;
#if USB_LOGGING
      if (!cmacTestPassed)
        eventLog.println(F("ERROR: LoRaWAN AES-CMAC known-answer test failed; uplink blocked"));
#endif
    }
    if (!cmacTestPassed) return false;

    uint8_t authenticated[16 + MAX_FRAME_SIZE];
    memcpy(authenticated, b0, sizeof(b0));
    memcpy(authenticated + sizeof(b0), frame, p);
    uint8_t state[16];
    if (!computeCmac(nwkCipher, authenticated, sizeof(b0) + p, state)) return false;
    for (uint8_t i = 0; i < 4; ++i) frame[p++] = state[i];
    frameLength = p;
    return p <= MAX_FRAME_SIZE;
  }

  // Receive a TTN-scheduled LoRaWAN 1.0.x downlink in the software loopback.
  // Validate DevAddr and MIC, enforce an increasing FCntDown, and decrypt the
  // application payload before returning it to the device-side handler.
  static bool receiveDownlink(const uint8_t *frame, size_t frameLength,
                              DownlinkInfo &info,
                              DownlinkRejectReason *rejectReason = nullptr,
                              CounterState *counterState = nullptr) {
    info = {};
    CounterState &counter = counterState ? *counterState : defaultCounterState();
    if (rejectReason) *rejectReason = DownlinkRejectReason::None;
    if (!frame || frameLength < 12 || frameLength > MAX_FRAME_SIZE)
      return rejectDownlink(rejectReason, DownlinkRejectReason::InvalidLength);
    const uint8_t mType = frame[0] & 0xE0;
    if ((mType != 0x60 && mType != 0xA0) || (frame[0] & 0x1F) != 0)
      return rejectDownlink(rejectReason, DownlinkRejectReason::UnsupportedFrameType);

    uint8_t nwkKey[16], appKey[16];
    uint32_t devAddr;
    if (!decodeHex(LORAWAN_NWKSKEY_HEX, nwkKey, sizeof(nwkKey)) ||
        !decodeHex(LORAWAN_APPSKEY_HEX, appKey, sizeof(appKey)) ||
        !decodeDevAddr(LORAWAN_DEVADDR_HEX, devAddr))
      return rejectDownlink(rejectReason, DownlinkRejectReason::InvalidSessionSettings);
    for (uint8_t i = 0; i < 4; ++i) {
      if (frame[1 + i] != static_cast<uint8_t>(devAddr >> (8 * i)))
        return rejectDownlink(rejectReason, DownlinkRejectReason::DevAddrMismatch);
    }

    const uint8_t fOptsLength = frame[5] & 0x0F;
    size_t pos = 8 + fOptsLength;
    if (pos > frameLength - 4)
      return rejectDownlink(rejectReason, DownlinkRejectReason::InvalidFrameLayout);
    const bool hasFPort = pos < frameLength - 4;
    const uint8_t fPort = hasFPort ? frame[pos++] : 0;
    const size_t payloadLength = hasFPort ? frameLength - 4 - pos : 0;
    if (payloadLength > sizeof(info.payload) ||
        fOptsLength > sizeof(info.fOpts) ||
        (fOptsLength && hasFPort && fPort == 0))
      return rejectDownlink(rejectReason, DownlinkRejectReason::InvalidFrameLayout);

    const uint16_t fCnt16 = static_cast<uint16_t>(frame[6]) |
      (static_cast<uint16_t>(frame[7]) << 8);
    uint32_t fCnt = fCnt16;
    bool duplicate = false;
    if (counter.seen) {
      fCnt = (counter.last & 0xFFFF0000UL) | fCnt16;
      duplicate = fCnt == counter.last;
      if (duplicate && mType != 0xA0)
        return rejectDownlink(rejectReason, DownlinkRejectReason::InvalidFrameCounter);
      if (fCnt < counter.last) {
        if (counter.last >= 0xFFFF0000UL)
          return rejectDownlink(rejectReason, DownlinkRejectReason::InvalidFrameCounter);
        fCnt += 0x10000UL;
      }
      if (!duplicate && fCnt <= counter.last)
        return rejectDownlink(rejectReason, DownlinkRejectReason::InvalidFrameCounter);
    }

    uint8_t b0[16] = {};
    b0[0] = 0x49;
    b0[5] = 0x01; // Downlink direction.
    memcpy(b0 + 6, frame + 1, 4);
    b0[10] = static_cast<uint8_t>(fCnt);
    b0[11] = static_cast<uint8_t>(fCnt >> 8);
    b0[12] = static_cast<uint8_t>(fCnt >> 16);
    b0[13] = static_cast<uint8_t>(fCnt >> 24);
    b0[15] = static_cast<uint8_t>(frameLength - 4);
    AES128 nwkCipher;
    if (!nwkCipher.setKey(nwkKey, sizeof(nwkKey)))
      return rejectDownlink(rejectReason, DownlinkRejectReason::CryptoFailure);

    // Reuse the RFC 4493 CMAC implementation already validated for uplinks.
    uint8_t authenticated[16 + MAX_FRAME_SIZE];
    memcpy(authenticated, b0, sizeof(b0));
    memcpy(authenticated + sizeof(b0), frame, frameLength - 4);
    uint8_t state[16];
    if (!computeCmac(nwkCipher, authenticated,
                     sizeof(b0) + frameLength - 4, state))
      return rejectDownlink(rejectReason, DownlinkRejectReason::CryptoFailure);
    if (memcmp(state, frame + frameLength - 4, 4) != 0)
      return rejectDownlink(rejectReason, DownlinkRejectReason::MicMismatch);

    if (payloadLength) {
      AES128 payloadCipher;
      const uint8_t *key = fPort == 0 ? nwkKey : appKey;
      if (!payloadCipher.setKey(key, 16))
        return rejectDownlink(rejectReason, DownlinkRejectReason::CryptoFailure);
      uint8_t a[16] = {};
      uint8_t stream[16];
      a[0] = 0x01;
      a[5] = 0x01;
      memcpy(a + 6, frame + 1, 4);
      a[10] = static_cast<uint8_t>(fCnt);
      a[11] = static_cast<uint8_t>(fCnt >> 8);
      a[12] = static_cast<uint8_t>(fCnt >> 16);
      a[13] = static_cast<uint8_t>(fCnt >> 24);
      for (size_t offset = 0; offset < payloadLength; offset += 16) {
        a[15] = static_cast<uint8_t>((offset / 16) + 1);
        payloadCipher.encryptBlock(stream, a);
        const size_t n = payloadLength - offset < 16 ? payloadLength - offset : 16;
        for (size_t i = 0; i < n; ++i) info.payload[offset + i] = frame[pos + offset + i] ^ stream[i];
      }
    }

    counter.last = fCnt;
    counter.seen = true;
    info.fPort = fPort;
    info.hasFPort = hasFPort;
    info.confirmed = mType == 0xA0;
    info.duplicate = duplicate;
    info.frameCounter = fCnt;
    info.fOptsLength = fOptsLength;
    if (fOptsLength) memcpy(info.fOpts, frame + 8, fOptsLength);
    info.payloadLength = payloadLength;
    return true;
  }

private:
  static bool rejectDownlink(DownlinkRejectReason *reason,
                             DownlinkRejectReason value) {
    if (reason) *reason = value;
    return false;
  }

  static CounterState &defaultCounterState() {
    static CounterState state = {};
    return state;
  }
  static int hexNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  }

  static void doubleCmacBlock(uint8_t block[16]) {
    uint8_t carry = 0;
    for (int8_t i = 15; i >= 0; --i) {
      const uint8_t nextCarry = static_cast<uint8_t>(block[i] >> 7);
      block[i] = static_cast<uint8_t>((block[i] << 1) | carry);
      carry = nextCarry;
    }
    if (carry) block[15] ^= 0x87;
  }

  // RFC 4493 AES-CMAC, using the AES block cipher already provided by the
  // Arduino Crypto library. Message bytes and subkeys use network byte order.
  static bool computeCmac(AES128 &cipher, const uint8_t *message,
                          size_t messageLength, uint8_t result[16]) {
    if ((!message && messageLength) || !result) return false;

    uint8_t zero[16] = {};
    uint8_t k1[16], k2[16];
    cipher.encryptBlock(k1, zero);
    doubleCmacBlock(k1);
    memcpy(k2, k1, sizeof(k2));
    doubleCmacBlock(k2);

    const size_t blockCount = messageLength ? (messageLength + 15) / 16 : 1;
    const bool completeLastBlock = messageLength != 0 && (messageLength % 16) == 0;
    const size_t precedingBlocks = blockCount - 1;
    uint8_t state[16] = {};
    uint8_t work[16];

    for (size_t blockIndex = 0; blockIndex < precedingBlocks; ++blockIndex) {
      const uint8_t *input = message + blockIndex * 16;
      for (uint8_t i = 0; i < 16; ++i) work[i] = state[i] ^ input[i];
      cipher.encryptBlock(state, work);
    }

    uint8_t last[16] = {};
    const size_t offset = precedingBlocks * 16;
    const size_t remaining = messageLength - offset;
    if (remaining) memcpy(last, message + offset, remaining);
    if (completeLastBlock) {
      for (uint8_t i = 0; i < 16; ++i) last[i] ^= k1[i];
    } else {
      last[remaining] = 0x80;
      for (uint8_t i = 0; i < 16; ++i) last[i] ^= k2[i];
    }
    for (uint8_t i = 0; i < 16; ++i) work[i] = state[i] ^ last[i];
    cipher.encryptBlock(result, work);
    return true;
  }

  static bool cmacKnownAnswerTest() {
    static const uint8_t key[16] = {
      0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
      0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
    };
    static const uint8_t message[40] = {
      0x6B, 0xC1, 0xBE, 0xE2, 0x2E, 0x40, 0x9F, 0x96,
      0xE9, 0x3D, 0x7E, 0x11, 0x73, 0x93, 0x17, 0x2A,
      0xAE, 0x2D, 0x8A, 0x57, 0x1E, 0x03, 0xAC, 0x9C,
      0x9E, 0xB7, 0x6F, 0xAC, 0x45, 0xAF, 0x8E, 0x51,
      0x30, 0xC8, 0x1C, 0x46, 0xA3, 0x5C, 0xE4, 0x11
    };
    static const uint8_t expected[16] = {
      0xDF, 0xA6, 0x67, 0x47, 0xDE, 0x9A, 0xE6, 0x30,
      0x30, 0xCA, 0x32, 0x61, 0x14, 0x97, 0xC8, 0x27
    };
    AES128 cipher;
    if (!cipher.setKey(key, sizeof(key))) return false;
    uint8_t result[16];
    return computeCmac(cipher, message, sizeof(message), result) &&
      memcmp(result, expected, sizeof(expected)) == 0;
  }

  static bool decodeHex(const char *text, uint8_t *out, size_t bytes) {
    if (strlen(text) != bytes * 2) return false;
    for (size_t i = 0; i < bytes; ++i) {
      const int hi = hexNibble(text[2 * i]);
      const int lo = hexNibble(text[2 * i + 1]);
      if (hi < 0 || lo < 0) return false;
      out[i] = static_cast<uint8_t>((hi << 4) | lo);
    }
    return true;
  }

  static bool decodeDevAddr(const char *text, uint32_t &out) {
    if (strlen(text) != 8) return false;
    out = 0;
    for (uint8_t i = 0; i < 8; ++i) {
      const int n = hexNibble(text[i]);
      if (n < 0) return false;
      out = (out << 4) | static_cast<uint32_t>(n);
    }
    return out != 0;
  }
};

#endif
