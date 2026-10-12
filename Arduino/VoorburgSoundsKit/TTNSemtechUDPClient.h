/*
 * Semtech UDP Packet Forwarder uplink client for the Pico 2 W software test
 * endpoint. It forwards the LoRaWAN PHYPayload in a GWMP PUSH_DATA/rxpk
 * envelope and listens for PUSH_ACK. No LoRa RF hardware is present.
 */
#ifndef VOORBURG_TTN_SEMTECH_UDP_CLIENT_H
#define VOORBURG_TTN_SEMTECH_UDP_CLIENT_H

#include <Arduino.h>
#include <WiFi.h>
#include "PicoDatagramSocket.h"
#include "PicoCrashDiagnostics.h"
#include <WiFiNTP.h>
#include <lwip_wrap.h>
#include <hardware/structs/watchdog.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include "LoraSettings.h"
#include "EventLog.h"
#include "LoRaWANMacState.h"
#include "LoRaAirtime.h"

// Routine socket/route tracing is optional; errors and publication results stay visible.
#ifndef TTN_NETWORK_VERBOSE
#define TTN_NETWORK_VERBOSE 0
#endif

#ifndef TTN_GATEWAY_EUI_HEX
#define TTN_GATEWAY_EUI_HEX ""
#endif
#ifndef TTN_SEMTECH_UDP_HOST
#define TTN_SEMTECH_UDP_HOST "eu1.cloud.thethings.network"
#endif
#ifndef TTN_SEMTECH_UDP_PORT
#define TTN_SEMTECH_UDP_PORT 1700
#endif
#ifndef TTN_SEMTECH_SIMULATED_RSSI_DBM
#define TTN_SEMTECH_SIMULATED_RSSI_DBM -100
#endif
#ifndef TTN_DEVICE_STATUS_MARGIN_DB
#define TTN_DEVICE_STATUS_MARGIN_DB 29
#endif
#ifndef TTN_SEMTECH_POST_UPLINK_ROUTE_MS
#define TTN_SEMTECH_POST_UPLINK_ROUTE_MS 31000UL
#endif

class TTNSemtechUDPClient {
public:
  typedef bool (*VirtualDownlinkHandler)(const uint8_t *, size_t,
                                       const LoRaWANDownlinkSettings &);

  // The Pico's watchdog scratch registers survive a watchdog reboot. The SDK
  // reserves register 4 (and 5-7 during reboot); diagnostics use 0-3.
  // It lets the next boot identify the exact UDP operation interrupted by a
  // hardware watchdog reset without writing diagnostic data to flash.
  static const char *consumePreviousOperation() {
    const uint32_t marker = watchdog_hw->scratch[0];
    const uint32_t stage = watchdog_hw->scratch[1];
    const uint32_t inverse = watchdog_hw->scratch[2];
    watchdog_hw->scratch[0] = 0;
    watchdog_hw->scratch[1] = 0;
    watchdog_hw->scratch[2] = 0;
    if (marker != DIAGNOSTIC_MAGIC || inverse != ~stage) return nullptr;
    return operationName(stage);
  }

  static void clearOperationMarker() {
    // Keep a valid context even between calls: a background IRQ can stall
    // while the foreground is outside a marked network/audio operation.
    markOperation(OP_BACKGROUND);
  }

  static void markUsbLogOperation() { markOperation(OP_USB_LOG); }
  static void markCounterStorageOperation() { markOperation(OP_COUNTER_FLASH); }
  static void markFrameBuildOperation() { markOperation(OP_FRAME_BUILD); }
  static void markProtocolStorageOperation() { markOperation(OP_PROTOCOL_FLASH); }
  static void markAudioStopOperation() { markOperation(OP_AUDIO_STOP); }
  static void markAudioStartOperation() { markOperation(OP_AUDIO_START); }
  static void markAudioReadOperation() { markOperation(OP_AUDIO_READ); }
  static void markRuntimeDiagnosticsOperation() { markOperation(OP_RUNTIME_STATS); }
  static uint32_t consumePreviousLoopMaximum() {
    const uint32_t value = watchdog_hw->scratch[3];
    watchdog_hw->scratch[3] = 0;
    return value;
  }
  static void retainLoopMaximum(uint32_t micros) {
    watchdog_hw->scratch[3] = micros;
  }

  void setVirtualDownlinkHandler(VirtualDownlinkHandler handler) {
    virtualDownlinkHandler = handler;
  }

  void setDeviceMacState(LoRaWANMacState *state) { deviceMacState = state; }

  void begin() {
    configurationValid = wifiSettingsValid() && parseGatewayEui();
    if (!configurationValid) {
      report(F("Semtech UDP disabled: set Wi-Fi credentials and a 16-hex-digit TTN_GATEWAY_EUI_HEX"));
      return;
    }
    if (TTN_SEMTECH_UDP_HOST[0] == '\0' || TTN_SEMTECH_UDP_PORT != 1700) {
      configurationValid = false;
      report(F("Semtech UDP requires the registered TTN Gateway Server host and UDP port 1700"));
      return;
    }
    reportVerbose(F("Semtech UDP identifies this gateway by its registered EUI"));
    reportVerbose(F("Semtech rxpk RSSI/SNR are configured test values; this Pico has no LoRa RF receiver"));

    markOperation(OP_WIFI_JOIN);
    WiFi.setTimeout(WIFI_JOIN_TIMEOUT_MS);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    nextWiFiAttemptAt = millis() + 2000UL;
  }

  bool configured() const { return configurationValid; }

  bool ready() const {
    return configurationValid && wifiWasConnected && ntpSynced;
  }

  // Request a route handshake only when a completed measurement is waiting.
  // The caller must hold the frame until this returns true.
  bool prepareUplinkRoute() {
    if (!ready()) return false;
    routeRequestedForUplink = true;
    if (!routeOpen && !routePreparationReported) {
      routePreparationReported = true;
      routeStepsReported = false;
      nextRouteDiagnosticAt = millis() + 3000UL;
      reportVerbose(F("Opening TTN gateway route before uplink publication"));
    }
    return routeOpen;
  }

  bool lastPushAcknowledged() const { return pushAcknowledged; }

  void maintain() {
    if (!configurationValid) return;
    const uint32_t now = millis();

    markOperation(OP_WIFI_STATE);
    if (WiFi.status() != WL_CONNECTED) {
      if (wifiWasConnected) {
        report(F("Wi-Fi disconnected; closing Semtech UDP socket and reconnecting"));
        markOperation(OP_SOCKET_SETUP);
        udp.stop();
        uplinkUdp.stop();
        udpStarted = false;
        uplinkUdpStarted = false;
        serverResolved = false;
        lastPullSentAt = 0;
        pendingPullToken = 0;
        routeOpen = false;
        routeKeepaliveUntil = 0;
        nextSocketAttemptAt = 0;
        nextUplinkSocketAttemptAt = 0;
        nextDnsAttemptAt = 0;
        socketRetryDelayMs = 500UL;
        uplinkSocketRetryDelayMs = 500UL;
      }
      wifiWasConnected = false;
      if (static_cast<int32_t>(now - nextWiFiAttemptAt) >= 0) {
        markOperation(OP_WIFI_JOIN);
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        nextWiFiAttemptAt = now + wifiRetryDelayMs;
        if (wifiRetryDelayMs < 30000UL) wifiRetryDelayMs *= 2;
        if (wifiRetryDelayMs > 30000UL) wifiRetryDelayMs = 30000UL;
      }
      return;
    }

    if (!wifiWasConnected) {
      wifiWasConnected = true;
      wifiRetryDelayMs = 2000UL;
      report(F("Wi-Fi connected"));
    }

    if (!ntpStarted && static_cast<int32_t>(now - nextNtpResolveAt) >= 0) {
      // NTP.begin(hostname) uses the core's 15-second DNS default. Resolve
      // one hostname per pass with an explicit limit, then use its IP API.
      markOperation(OP_NTP_SETUP);
      IPAddress ntpAddress;
      const char *hostname = ntpServerIndex ? "time.nist.gov" : "pool.ntp.org";
      if (WiFi.hostByName(hostname, ntpAddress, NTP_DNS_TIMEOUT_MS) == 1) {
        NTP.begin(ntpAddress);
        ntpStarted = true;
        reportVerbose(F("Clock synchronization started (NTP; timestamps are UTC)"));
      } else {
        ntpServerIndex ^= 1;
        nextNtpResolveAt = now + 5000UL;
        report(F("NTP hostname resolution timed out; retrying later"));
      }
    }
    if (!ntpSynced && time(nullptr) >= 1704067200) {
      ntpSynced = true;
      report(F("System clock synchronized via NTP"));
    }

    // Allocate separate receive and uplink sockets while the microphone is
    // still measuring. The port-1700 socket owns PULL_DATA/PULL_ACK/PULL_RESP/
    // TX_ACK; PUSH_DATA and its PUSH_ACK use a separate ephemeral socket. This
    // keeps the listening return path separate from uplink acknowledgements.
    // Binding and DNS do not open a TTN gateway route.
    if (!udpStarted) {
      if (static_cast<int32_t>(now - nextSocketAttemptAt) >= 0) {
        reportVerbose(F("Binding Semtech downlink listener on local UDP port 1700"));
        markOperation(OP_SOCKET_SETUP);
        if (!udp.begin(TTN_SEMTECH_UDP_PORT)) {
          report(F("Could not open local UDP port 1700; retrying"));
          nextSocketAttemptAt = now + socketRetryDelayMs;
          if (socketRetryDelayMs < 30000UL) socketRetryDelayMs *= 2;
          if (socketRetryDelayMs > 30000UL) socketRetryDelayMs = 30000UL;
        } else {
          udpStarted = true;
          nextSocketAttemptAt = 0;
          socketRetryDelayMs = 500UL;
          reportVerbose(F("Semtech port-1700 downlink listener is ready"));
        }
      }
      if (!udpStarted) return;
    }

    if (!uplinkUdpStarted) {
      if (static_cast<int32_t>(now - nextUplinkSocketAttemptAt) >= 0) {
        reportVerbose(F("Opening separate Semtech uplink UDP socket"));
        markOperation(OP_SOCKET_SETUP);
        if (!uplinkUdp.begin(0)) {
          report(F("Could not open separate Semtech uplink socket; retrying"));
          uplinkUdp.stop();
          nextUplinkSocketAttemptAt = now + uplinkSocketRetryDelayMs;
          if (uplinkSocketRetryDelayMs < 30000UL) uplinkSocketRetryDelayMs *= 2;
          if (uplinkSocketRetryDelayMs > 30000UL) uplinkSocketRetryDelayMs = 30000UL;
        } else {
          uplinkUdpStarted = true;
          nextUplinkSocketAttemptAt = 0;
          uplinkSocketRetryDelayMs = 500UL;
          reportVerbose(F("Separate Semtech uplink socket is ready"));
        }
      }
      if (!uplinkUdpStarted) return;
    }

    if (!serverResolved) {
      if (static_cast<int32_t>(now - nextDnsAttemptAt) < 0) return;
      reportVerbose(F("Resolving TTN Gateway Server hostname (2.5-second limit)"));
      markOperation(OP_DNS_SETUP);
      // Arduino-Pico 6.2.0 defaults hostByName() to 15 seconds, longer than
      // this RP2350's maximum 8.3-second hardware watchdog. Bound each lookup
      // so a slow DNS response is retried by the loop instead of rebooting the
      // Pico while a measurement is waiting.
      if (WiFi.hostByName(TTN_SEMTECH_UDP_HOST, serverAddress, DNS_TIMEOUT_MS) != 1) {
        report(F("Could not resolve TTN Gateway Server hostname; retrying DNS"));
        nextDnsAttemptAt = now + 5000UL;
        return;
      }
      serverResolved = true;
      nextDnsAttemptAt = 0;
      reportVerbose(F("TTN Semtech UDP server resolved; no gateway route opened yet"));
    }

    if (!ntpSynced) {
      if (!waitingForClockReported) {
        reportVerbose(F("Waiting for NTP time before starting Semtech UDP"));
        waitingForClockReported = true;
      }
      return;
    }

    if (!routeRequestedForUplink && routeKeepaliveUntil != 0 &&
        static_cast<int32_t>(now - routeKeepaliveUntil) >= 0) {
      routeKeepaliveUntil = 0;
      routeOpen = false;
      pendingPullToken = 0;
    }

    // Local sockets and DNS are ready in advance, but do not advertise the
    // gateway until an uplink is waiting. After publication, keep PULL_DATA
    // active through the downlink response window, then let the server-side
    // route expire while retaining local sockets for the next publication.
    const bool routeNeeded = routeRequestedForUplink || routeKeepaliveUntil != 0;
    if (!routeNeeded) return;

    if (!routeStepsReported) {
      reportVerbose(F("Route step 1/3: polling port 1700 for pending TTN packets"));
    }
    markOperation(OP_ROUTE_RECEIVE);
    processIncomingPackets();
    reportReceiveDrops();
    if (!routeStepsReported) {
      reportVerbose(F("Route step 2/3: checking scheduled downlink work"));
    }
    markOperation(OP_ROUTE_DOWNLINK);
    processScheduledDownlink();
    // Semtech UDP gateways refresh the NAT path with PULL_DATA, typically
    // every five seconds. Do this only while preparing an uplink or awaiting
    // its scheduled downlink response.
    if (lastPullSentAt == 0 || now - lastPullSentAt >= PULL_INTERVAL_MS) {
      if (!routeStepsReported) {
        reportVerbose(F("Route step 3/3: sending GWMP PULL_DATA to open the route"));
        routeStepsReported = true;
      }
      sendPullData();
    }
    if (routeRequestedForUplink && !routeOpen &&
        static_cast<int32_t>(now - nextRouteDiagnosticAt) >= 0) {
      report(F("TTN gateway route is not open; uplink held pending PULL_ACK"));
      nextRouteDiagnosticAt = now + 15000UL;
    }
  }

  bool publishUplink(const uint8_t *frame, size_t frameLength) {
    const LoRaWANRadioSettings radio = {
      TTN_UPLINK_FREQUENCY_HZ, TTN_UPLINK_BANDWIDTH_HZ,
      TTN_UPLINK_SPREADING_FACTOR, 0
    };
    return publishUplink(frame, frameLength, radio);
  }

  bool publishUplink(const uint8_t *frame, size_t frameLength,
                     const LoRaWANRadioSettings &radio) {
    if (!ready() || !routeOpen || !udpStarted || !uplinkUdpStarted ||
        !frame || frameLength == 0 || frameLength > 255 ||
        !radioSettingsValid(radio)) return false;

    char encodedFrame[344];
    if (!encodeBase64(frame, frameLength, encodedFrame, sizeof(encodedFrame))) {
      report(F("Could not base64-encode LoRaWAN frame for Semtech UDP"));
      return false;
    }

    const uint32_t receivedAt = micros();
    const time_t seconds = time(nullptr);
    struct tm utc;
    char timeSeconds[24];
    char timeText[32];
    if (seconds < 1704067200 || !gmtime_r(&seconds, &utc) ||
        strftime(timeSeconds, sizeof(timeSeconds), "%Y-%m-%dT%H:%M:%S", &utc) == 0) {
      report(F("No synchronized UTC time for Semtech rxpk metadata"));
      return false;
    }
    snprintf(timeText, sizeof(timeText), "%s.000000Z", timeSeconds);

    char frequencyText[20];
    const uint32_t frequencyMHz = radio.frequencyHz / 1000000UL;
    const uint32_t frequencyFraction = radio.frequencyHz % 1000000UL;
    snprintf(frequencyText, sizeof(frequencyText), "%lu.%06lu",
             static_cast<unsigned long>(frequencyMHz),
             static_cast<unsigned long>(frequencyFraction));

    char dataRate[20];
    snprintf(dataRate, sizeof(dataRate), "SF%uBW%lu",
             static_cast<unsigned>(radio.spreadingFactor),
             static_cast<unsigned long>(radio.bandwidthHz / 1000UL));

    char rssiText[12];
    snprintf(rssiText, sizeof(rssiText), "%d",
             static_cast<int>(TTN_SEMTECH_SIMULATED_RSSI_DBM));
    const int lsnrTenths = static_cast<int>(TTN_DEVICE_STATUS_MARGIN_DB) * 10;
    const unsigned int lsnrMagnitude = static_cast<unsigned int>(
      lsnrTenths < 0 ? -lsnrTenths : lsnrTenths);
    char lsnrText[16];
    snprintf(lsnrText, sizeof(lsnrText), "%s%u.%u",
             lsnrTenths < 0 ? "-" : "", lsnrMagnitude / 10,
             lsnrMagnitude % 10);

    char json[640];
    const int jsonLength = snprintf(
      json, sizeof(json),
      "{\"rxpk\":[{\"time\":\"%s\",\"tmst\":%lu,\"freq\":%s,\"chan\":%u,\"rfch\":0,\"stat\":1,\"modu\":\"LORA\",\"datr\":\"%s\",\"codr\":\"%s\",\"rssi\":%s,\"lsnr\":%s,\"size\":%u,\"data\":\"%s\"}]}",
      timeText, static_cast<unsigned long>(receivedAt), frequencyText,
      static_cast<unsigned>(radio.channelIndex),
      dataRate, TTN_UPLINK_CODING_RATE, rssiText, lsnrText,
      static_cast<unsigned>(frameLength), encodedFrame);
    if (jsonLength <= 0 || static_cast<size_t>(jsonLength) >= sizeof(json)) {
      report(F("Semtech UDP rxpk JSON exceeded its buffer"));
      return false;
    }

    uint8_t packet[12];
    makeHeader(packet, PUSH_DATA);
    uint16_t token = readToken(packet);
    pushAcknowledged = false;
    waitingForPushAck = true;
    pendingPushToken = token;

    if (!sendDatagram(uplinkUdp, packet, sizeof(packet),
                      reinterpret_cast<const uint8_t *>(json),
                      static_cast<size_t>(jsonLength), false)) {
      waitingForPushAck = false;
      pendingPushToken = 0;
      report(F("Semtech PUSH_DATA was not submitted; retaining frame for retry"));
      return false;
    }

    // PUSH_DATA has now been submitted. Keep PULL_DATA active long enough for
    // TTN's RX1/RX2 downlink and TX_ACK, then allow the gateway route to expire.
    if (deviceMacState) deviceMacState->noteUplink(receivedAt, radio);
    routeRequestedForUplink = false;
    routeKeepaliveUntil = millis() + postUplinkRouteHoldMs();

    reportVerbose(F("Semtech UDP PUSH_DATA sent; waiting briefly for PUSH_ACK"));
    const uint32_t start = millis();
    while (!pushAcknowledged && millis() - start < PUSH_ACK_WAIT_MS) {
      // PUSH_ACK returns to the source port of PUSH_DATA. Downlink traffic
      // remains on the port-1700 listener and is serviced independently.
      processIncomingPackets();
      processPushAcknowledgements();
      processScheduledDownlink();
      if (!pushAcknowledged) delay(1);
    }
    waitingForPushAck = false;

    if (pushAcknowledged) {
      reportVerbose(F("Semtech UDP PUSH_ACK received; TTN acknowledged the datagram, not device acceptance"));
    } else {
      // GWMP ACKs are diagnostic; Semtech's protocol does not define retries.
      // Treat a successful local send as submitted to avoid replaying the same
      // FCntUp repeatedly when only the UDP acknowledgement was lost.
      report(F("No Semtech UDP PUSH_ACK received; datagram was sent once and will not be replayed"));
    }
    return true;
  }

private:
  static const uint32_t DIAGNOSTIC_MAGIC = 0x56534731UL; // "VSG1"
  enum OperationStage : uint32_t {
    OP_ROUTE_RECEIVE = 1,
    OP_ROUTE_DOWNLINK = 2,
    OP_PULL_BEGIN_PACKET = 3,
    OP_PULL_WRITE_HEADER = 4,
    OP_PULL_END_PACKET = 5,
    OP_PUSH_BEGIN_PACKET = 6,
    OP_PUSH_WRITE = 7,
    OP_PUSH_END_PACKET = 8,
    OP_TX_ACK_SEND = 9,
    OP_PULL_RESP_PARSE = 10,
    OP_SCHEDULED_DOWNLINK = 11,
    OP_USB_LOG = 12,
    OP_COUNTER_FLASH = 13,
    OP_FRAME_BUILD = 14,
    OP_PROTOCOL_FLASH = 15,
    OP_AUDIO_STOP = 16,
    OP_AUDIO_START = 17,
    OP_AUDIO_READ = 18,
    OP_RUNTIME_STATS = 19,
    OP_BACKGROUND = 20,
    OP_WIFI_STATE = 21,
    OP_WIFI_JOIN = 22,
    OP_NTP_SETUP = 23,
    OP_SOCKET_SETUP = 24,
    OP_DNS_SETUP = 25
  };
  static const uint8_t PROTOCOL_VERSION = 2;
  static const uint8_t PUSH_DATA = 0x00;
  static const uint8_t PUSH_ACK = 0x01;
  static const uint8_t PULL_DATA = 0x02;
  static const uint8_t PULL_RESP = 0x03;
  static const uint8_t PULL_ACK = 0x04;
  static const uint8_t TX_ACK = 0x05;
  static const uint32_t PULL_INTERVAL_MS = 5000UL;
  static const uint32_t PUSH_ACK_WAIT_MS = 500UL;
  static const int DNS_TIMEOUT_MS = 2500;
  static const int NTP_DNS_TIMEOUT_MS = 1500;
  static const unsigned long WIFI_JOIN_TIMEOUT_MS = 1500;
  static const uint32_t POST_DOWNLINK_ROUTE_GRACE_MS = 5000UL;
  static const size_t MAX_PULL_RESP_JSON = 768;
  static const uint32_t MAX_SCHEDULE_AHEAD_US = 60000000UL;
  static const uint32_t MAX_START_LATENESS_US = 250000UL;
  static const uint8_t MAX_PACKETS_PER_PASS = 4;

  PicoDatagramSocket udp;       // PULL_DATA and downlink traffic use local port 1700.
  PicoDatagramSocket uplinkUdp; // PUSH_DATA/PUSH_ACK use a separate ephemeral local port.
  IPAddress serverAddress;
  uint8_t gatewayEui[8] = {};
  bool configurationValid = false;
  bool wifiWasConnected = false;
  bool ntpStarted = false;
  bool ntpSynced = false;
  bool waitingForClockReported = false;
  bool udpStarted = false;
  bool uplinkUdpStarted = false;
  bool serverResolved = false;
  bool pushAcknowledged = false;
  bool waitingForPushAck = false;
  uint16_t pendingPushToken = 0;
  uint16_t pendingPullToken = 0;
  uint16_t nextTokenValue = 0;
  uint32_t nextWiFiAttemptAt = 0;
  uint32_t nextSocketAttemptAt = 0;
  uint32_t nextUplinkSocketAttemptAt = 0;
  uint32_t nextDnsAttemptAt = 0;
  uint32_t nextNtpResolveAt = 0;
  uint8_t ntpServerIndex = 0;
  uint32_t wifiRetryDelayMs = 2000UL;
  uint32_t socketRetryDelayMs = 500UL;
  uint32_t uplinkSocketRetryDelayMs = 500UL;
  uint32_t lastPullSentAt = 0;
  uint32_t routeKeepaliveUntil = 0;
  uint32_t nextRouteDiagnosticAt = 0;
  bool routeRequestedForUplink = false;
  bool routePreparationReported = false;
  bool routeStepsReported = false;
  bool routeOpen = false;
  uint8_t consecutiveSendFailures = 0;
  uint32_t reportedReceiveDrops = 0;
  uint32_t nextReceiveDropReportAt = 0;
  VirtualDownlinkHandler virtualDownlinkHandler = nullptr;
  LoRaWANMacState *deviceMacState = nullptr;
  bool virtualTxPending = false;
  bool virtualTxStarted = false;
  uint16_t virtualTxToken = 0;
  uint32_t virtualTxAtMicros = 0;
  LoRaWANDownlinkSettings virtualTxSettings = {};
  uint8_t virtualTxFrame[255] = {};
  size_t virtualTxFrameLength = 0;

  static void markOperation(OperationStage stage) {
    watchdog_hw->scratch[0] = DIAGNOSTIC_MAGIC;
    watchdog_hw->scratch[1] = static_cast<uint32_t>(stage);
    watchdog_hw->scratch[2] = ~static_cast<uint32_t>(stage);
    PicoCrashDiagnostics::setOperation(static_cast<uint32_t>(stage));
  }

  static const char *operationName(uint32_t stage) {
    switch (stage) {
      case OP_ROUTE_RECEIVE: return "polling incoming packets on UDP port 1700";
      case OP_ROUTE_DOWNLINK: return "processing scheduled downlink work";
      case OP_PULL_BEGIN_PACKET: return "PULL_DATA beginPacket()";
      case OP_PULL_WRITE_HEADER: return "writing the PULL_DATA header";
      case OP_PULL_END_PACKET: return "PULL_DATA endPacket() / lwIP UDP send";
      case OP_PUSH_BEGIN_PACKET: return "PUSH_DATA beginPacket()";
      case OP_PUSH_WRITE: return "writing the PUSH_DATA datagram";
      case OP_PUSH_END_PACKET: return "PUSH_DATA endPacket() / lwIP UDP send";
      case OP_TX_ACK_SEND: return "sending Semtech TX_ACK";
      case OP_PULL_RESP_PARSE: return "decoding a Semtech PULL_RESP";
      case OP_SCHEDULED_DOWNLINK: return "delivering a scheduled downlink";
      case OP_USB_LOG: return "servicing USB serial log output";
      case OP_COUNTER_FLASH: return "reserving an uplink counter in flash";
      case OP_FRAME_BUILD: return "building and authenticating the LoRaWAN frame";
      case OP_PROTOCOL_FLASH: return "saving the LoRaWAN protocol flash journal";
      case OP_AUDIO_STOP: return "stopping I2S microphone DMA";
      case OP_AUDIO_START: return "starting I2S microphone DMA";
      case OP_AUDIO_READ: return "reading and analyzing I2S samples";
      case OP_RUNTIME_STATS: return "collecting runtime memory diagnostics";
      case OP_BACKGROUND: return "between foreground operations (background interrupt or core service possible)";
      case OP_WIFI_STATE: return "checking Wi-Fi connection state";
      case OP_WIFI_JOIN: return "joining or reconnecting Wi-Fi";
      case OP_NTP_SETUP: return "resolving or starting NTP";
      case OP_SOCKET_SETUP: return "opening or closing a UDP socket";
      case OP_DNS_SETUP: return "resolving the TTN Gateway Server";
      default: return "unknown marked UDP operation";
    }
  }

  static bool wifiSettingsValid() {
    return WIFI_SSID[0] != '\0' && strncmp(WIFI_SSID, "YOUR_", 5) != 0 &&
      WIFI_PASSWORD[0] != '\0' && strcmp(WIFI_PASSWORD, "YOUR_WIFI_PASSWORD") != 0;
  }

  static int hexNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  }

  bool parseGatewayEui() {
    if (strlen(TTN_GATEWAY_EUI_HEX) != 16) return false;
    for (size_t i = 0; i < sizeof(gatewayEui); ++i) {
      const int high = hexNibble(TTN_GATEWAY_EUI_HEX[2 * i]);
      const int low = hexNibble(TTN_GATEWAY_EUI_HEX[2 * i + 1]);
      if (high < 0 || low < 0) return false;
      gatewayEui[i] = static_cast<uint8_t>((high << 4) | low);
    }
    return true;
  }

  static bool radioSettingsValid(const LoRaWANRadioSettings &radio) {
    return radio.frequencyHz >= 863000000UL &&
      radio.frequencyHz <= 870000000UL &&
      (radio.bandwidthHz == 125000UL ||
       radio.bandwidthHz == 250000UL ||
       radio.bandwidthHz == 500000UL) &&
      radio.spreadingFactor >= 7 && radio.spreadingFactor <= 12 &&
      (strcmp(TTN_UPLINK_CODING_RATE, "4/5") == 0 ||
       strcmp(TTN_UPLINK_CODING_RATE, "4/6") == 0 ||
       strcmp(TTN_UPLINK_CODING_RATE, "4/7") == 0 ||
       strcmp(TTN_UPLINK_CODING_RATE, "4/8") == 0);
  }

  uint32_t postUplinkRouteHoldMs() const {
    // RXDelay=15 places RX2 sixteen seconds after the uplink. Retain the
    // route through a maximum-length SF12/BW125 downlink and its TX_ACK.
    const LoRaWANRadioSettings slowest = {869525000UL, 125000UL, 12, 0};
    const uint32_t airtimeMs =
      (LoRaAirtime::estimateLoRaAirtimeMicros(255, slowest) + 999UL) / 1000UL;
    uint32_t receiveWindowMs = 16000UL;
    if (deviceMacState) {
      const uint32_t configuredWindowMs =
        (static_cast<uint32_t>(deviceMacState->rxDelaySeconds()) + 1) * 1000UL;
      if (configuredWindowMs > receiveWindowMs) receiveWindowMs = configuredWindowMs;
    }
    const uint32_t minimum = receiveWindowMs + airtimeMs + POST_DOWNLINK_ROUTE_GRACE_MS;
    return TTN_SEMTECH_POST_UPLINK_ROUTE_MS > minimum ?
      TTN_SEMTECH_POST_UPLINK_ROUTE_MS : minimum;
  }

  uint16_t nextToken() {
    if (nextTokenValue == 0) {
      nextTokenValue = static_cast<uint16_t>(micros() ^ millis() ^
        static_cast<uint32_t>(WiFi.localIP()[3]));
      if (nextTokenValue == 0) nextTokenValue = 1;
    }
    ++nextTokenValue;
    return nextTokenValue;
  }

  uint16_t makeHeader(uint8_t *header, uint8_t identifier) {
    const uint16_t token = nextToken();
    header[0] = PROTOCOL_VERSION;
    header[1] = static_cast<uint8_t>(token >> 8);
    header[2] = static_cast<uint8_t>(token);
    header[3] = identifier;
    if (identifier == PUSH_DATA || identifier == PULL_DATA || identifier == TX_ACK) {
      memcpy(header + 4, gatewayEui, sizeof(gatewayEui));
    }
    return token;
  }

  static uint16_t readToken(const uint8_t *header) {
    return static_cast<uint16_t>((static_cast<uint16_t>(header[1]) << 8) | header[2]);
  }

  bool sendDatagram(PicoDatagramSocket &socket, const uint8_t *header, size_t headerLength,
                    const uint8_t *payload = nullptr, size_t payloadLength = 0,
                    bool listenerSocket = true) {
    if (!serverResolved) {
      report(F("Semtech UDP send failed: server address is unresolved"));
      return false;
    }
    const uint8_t identifier = headerLength >= 4 ? header[3] : 0xFF;
    if (identifier == PULL_DATA) markOperation(OP_PULL_BEGIN_PACKET);
    else if (identifier == PUSH_DATA) markOperation(OP_PUSH_BEGIN_PACKET);
    else if (identifier == TX_ACK) markOperation(OP_TX_ACK_SEND);
    if (!socket.beginPacket(serverAddress, TTN_SEMTECH_UDP_PORT)) {
      report(F("Semtech UDP send failed at beginPacket"));
      recoverSocket(socket, listenerSocket);
      return false;
    }
    if (identifier == PULL_DATA) markOperation(OP_PULL_WRITE_HEADER);
    else if (identifier == PUSH_DATA) markOperation(OP_PUSH_WRITE);
    if (socket.write(header, headerLength) != headerLength) {
      report(F("Semtech UDP send failed while writing GWMP header"));
      recoverSocket(socket, listenerSocket);
      return false;
    }
    if (payloadLength && (!payload || socket.write(payload, payloadLength) != payloadLength)) {
      report(F("Semtech UDP send failed while writing JSON payload"));
      recoverSocket(socket, listenerSocket);
      return false;
    }
    if (identifier == PULL_DATA) markOperation(OP_PULL_END_PACKET);
    else if (identifier == PUSH_DATA) markOperation(OP_PUSH_END_PACKET);
    if (socket.endPacket() != 1) {
      report(F("Semtech UDP send failed at endPacket"));
      reportNetworkState();
      recoverSocket(socket, listenerSocket);
      return false;
    }
    consecutiveSendFailures = 0;
    if (listenerSocket) socketRetryDelayMs = 500UL;
    else uplinkSocketRetryDelayMs = 500UL;
    return true;
  }

  void recoverSocket(PicoDatagramSocket &socket, bool listenerSocket) {
    if (consecutiveSendFailures < 255) ++consecutiveSendFailures;
    // A failed send means lwIP did not submit this datagram. Discard only the
    // affected PCB. A failed listener send invalidates the TTN route; a failed
    // uplink send leaves the listener and its route intact for a safe retry.
    markOperation(OP_SOCKET_SETUP);
    socket.stop();
    if (listenerSocket) {
      udpStarted = false;
      routeOpen = false;
      pendingPullToken = 0;
      lastPullSentAt = 0;
      nextSocketAttemptAt = millis() + socketRetryDelayMs;
      if (socketRetryDelayMs < 30000UL) socketRetryDelayMs *= 2;
      if (socketRetryDelayMs > 30000UL) socketRetryDelayMs = 30000UL;
    } else {
      uplinkUdpStarted = false;
      pendingPushToken = 0;
      waitingForPushAck = false;
      nextUplinkSocketAttemptAt = millis() + uplinkSocketRetryDelayMs;
      if (uplinkSocketRetryDelayMs < 30000UL) uplinkSocketRetryDelayMs *= 2;
      if (uplinkSocketRetryDelayMs > 30000UL) uplinkSocketRetryDelayMs = 30000UL;
    }
    if (consecutiveSendFailures == 1 || consecutiveSendFailures % 10 == 0) {
      report(F("Rebuilding the affected UDP socket; retaining frame and route request"));
    }
  }

  static void reportNetworkState() {
#if USB_LOGGING
    const IPAddress localIp = WiFi.localIP();
    eventLog.print(F("UDP failure state: Wi-Fi status="));
    eventLog.print(static_cast<int>(WiFi.status()));
    eventLog.print(F(", local IP="));
    eventLog.print(static_cast<unsigned int>(localIp[0]));
    eventLog.print('.');
    eventLog.print(static_cast<unsigned int>(localIp[1]));
    eventLog.print('.');
    eventLog.print(static_cast<unsigned int>(localIp[2]));
    eventLog.print('.');
    eventLog.println(static_cast<unsigned int>(localIp[3]));
#endif
  }

  void sendPullData() {
    uint8_t packet[12];
    pendingPullToken = makeHeader(packet, PULL_DATA);
    reportVerbose(F("Submitting Semtech PULL_DATA from local UDP port 1700"));
    if (sendDatagram(udp, packet, sizeof(packet))) {
      lastPullSentAt = millis();
    } else {
      report(F("Semtech UDP PULL_DATA send failed"));
      lastPullSentAt = 0;
      pendingPullToken = 0;
    }
  }

  void processIncomingPackets() {
    for (uint8_t processed = 0; processed < MAX_PACKETS_PER_PASS; ++processed) {
      markOperation(OP_ROUTE_RECEIVE);
      const int packetLength = parsePacketSafely(udp);
      if (packetLength <= 0) break;
      uint8_t header[4] = {};
      const int bytesRead = readPacketSafely(udp, header, sizeof(header));
      if (bytesRead < 4 || header[0] != PROTOCOL_VERSION) {
        drainListenerPacket(static_cast<size_t>(packetLength - (bytesRead > 0 ? bytesRead : 0)));
        continue;
      }

      const uint16_t token = readToken(header);
      const size_t remaining = static_cast<size_t>(packetLength - bytesRead);
      if (header[3] == PULL_ACK) {
        drainListenerPacket(remaining);
        if (pendingPullToken != 0 && token == pendingPullToken) {
          pendingPullToken = 0;
          if (!routeOpen) {
            routeOpen = true;
            routePreparationReported = false;
            reportVerbose(F("TTN acknowledged Semtech UDP PULL_DATA; gateway route is open"));
          }
        }
      } else if (header[3] == PUSH_ACK) {
        drainListenerPacket(remaining);
        if (waitingForPushAck && token == pendingPushToken) {
          pushAcknowledged = true;
        }
      } else if (header[3] == PULL_RESP) {
        markOperation(OP_PULL_RESP_PARSE);
        receivePullResponse(token, static_cast<size_t>(packetLength - 4));
      } else {
        drainListenerPacket(remaining);
      }
    }
  }

  void processPushAcknowledgements() {
    for (uint8_t processed = 0; processed < MAX_PACKETS_PER_PASS; ++processed) {
      const int packetLength = parsePacketSafely(uplinkUdp);
      if (packetLength <= 0) break;
      uint8_t header[4] = {};
      const int bytesRead = readPacketSafely(uplinkUdp, header, sizeof(header));
      drainPacket(uplinkUdp, static_cast<size_t>(packetLength - (bytesRead > 0 ? bytesRead : 0)));
      if (bytesRead >= 4 && header[0] == PROTOCOL_VERSION &&
          header[3] == PUSH_ACK && waitingForPushAck &&
          readToken(header) == pendingPushToken) {
        pushAcknowledged = true;
      }
    }
  }

  void drainListenerPacket(size_t bytesRemaining) {
    drainPacket(udp, bytesRemaining);
  }

  void reportReceiveDrops() {
    const uint32_t total = udp.droppedPackets() + uplinkUdp.droppedPackets();
    if (total == reportedReceiveDrops ||
        static_cast<int32_t>(millis() - nextReceiveDropReportAt) < 0) return;
#if USB_LOGGING
    eventLog.print(F("UDP receive buffers dropped "));
    eventLog.print(static_cast<unsigned long>(total - reportedReceiveDrops));
    eventLog.println(F(" oversized, incomplete or excess packets; processing continues"));
#endif
    reportedReceiveDrops = total;
    nextReceiveDropReportAt = millis() + 15000UL;
  }

  // Polling only accesses complete packets in the fixed RAM queue. These
  // operations neither enter lwIP nor execute Wi-Fi background work.
  static int parsePacketSafely(PicoDatagramSocket &socket) {
    return socket.parsePacket();
  }

  static int readPacketSafely(PicoDatagramSocket &socket, uint8_t *buffer, size_t length) {
    return socket.read(buffer, length);
  }

  static void drainPacket(PicoDatagramSocket &socket, size_t bytesRemaining) {
    uint8_t discard[32];
    while (bytesRemaining) {
      const size_t request = bytesRemaining < sizeof(discard) ? bytesRemaining : sizeof(discard);
      const int got = readPacketSafely(socket, discard, request);
      if (got <= 0) break;
      bytesRemaining -= static_cast<size_t>(got);
    }
  }

  static const char *findJsonField(const char *begin, const char *end,
                                   const char *key) {
    if (!begin || !end || !key) return nullptr;
    char quotedKey[24];
    const int keyLength = snprintf(quotedKey, sizeof(quotedKey), "\"%s\"", key);
    if (keyLength <= 0 || static_cast<size_t>(keyLength) >= sizeof(quotedKey)) return nullptr;
    const char *cursor = begin;
    while (cursor < end) {
      const char *found = strstr(cursor, quotedKey);
      if (!found || found >= end) return nullptr;
      const char *value = found + keyLength;
      while (value < end && (*value == ' ' || *value == '\t' ||
                             *value == '\r' || *value == '\n')) ++value;
      if (value < end && *value == ':') {
        ++value;
        while (value < end && (*value == ' ' || *value == '\t' ||
                               *value == '\r' || *value == '\n')) ++value;
        return value < end ? value : nullptr;
      }
      cursor = found + keyLength;
    }
    return nullptr;
  }

  static bool jsonStringField(const char *begin, const char *end,
                              const char *key, char *out, size_t capacity) {
    const char *value = findJsonField(begin, end, key);
    if (!value || *value != '"' || !out || capacity == 0) return false;
    ++value;
    size_t length = 0;
    while (value < end && *value != '"') {
      if (*value == '\\' || length + 1 >= capacity) return false;
      out[length++] = *value++;
    }
    if (value >= end || *value != '"') return false;
    out[length] = '\0';
    return true;
  }

  static bool jsonBoolField(const char *begin, const char *end,
                            const char *key, bool &out) {
    const char *value = findJsonField(begin, end, key);
    if (!value) return false;
    if (static_cast<size_t>(end - value) >= 4 && strncmp(value, "true", 4) == 0 &&
        jsonValueEnded(value + 4, end)) {
      out = true;
      return true;
    }
    if (static_cast<size_t>(end - value) >= 5 && strncmp(value, "false", 5) == 0 &&
        jsonValueEnded(value + 5, end)) {
      out = false;
      return true;
    }
    return false;
  }

  static bool jsonUnsignedField(const char *begin, const char *end,
                                const char *key, uint32_t &out) {
    const char *value = findJsonField(begin, end, key);
    if (!value || value >= end || *value < '0' || *value > '9') return false;
    char *parsedEnd = nullptr;
    errno = 0;
    const unsigned long parsed = strtoul(value, &parsedEnd, 10);
    if (errno == ERANGE || parsedEnd == value || parsedEnd > end ||
        parsed > UINT32_MAX || !jsonValueEnded(parsedEnd, end)) return false;
    out = static_cast<uint32_t>(parsed);
    return true;
  }

  static bool jsonFrequencyField(const char *begin, const char *end,
                                 const char *key, double &out) {
    const char *value = findJsonField(begin, end, key);
    if (!value || value >= end) return false;
    char *parsedEnd = nullptr;
    errno = 0;
    const double parsed = strtod(value, &parsedEnd);
    if (errno == ERANGE || !isfinite(parsed) || parsedEnd == value ||
        parsedEnd > end || !jsonValueEnded(parsedEnd, end)) return false;
    out = parsed;
    return true;
  }

  static bool jsonValueEnded(const char *value, const char *end) {
    while (value < end && (*value == ' ' || *value == '\t' ||
                           *value == '\r' || *value == '\n')) ++value;
    return value == end || (value < end && *value == ',');
  }

  static int base64Value(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
  }

  static bool decodeBase64(const char *text, uint8_t *out,
                           size_t capacity, size_t &outLength) {
    if (!text || !out) return false;
    const size_t textLength = strlen(text);
    if (!textLength || textLength > 344 || (textLength % 4) == 1) return false;
    uint32_t accumulator = 0;
    uint8_t bits = 0;
    uint8_t padding = 0;
    outLength = 0;
    for (size_t i = 0; i < textLength; ++i) {
      if (text[i] == '=') {
        ++padding;
        if (padding > 2 || textLength - i > 2) return false;
        continue;
      }
      if (padding) return false;
      const int value = base64Value(text[i]);
      if (value < 0) return false;
      accumulator = (accumulator << 6) | static_cast<uint32_t>(value);
      bits = static_cast<uint8_t>(bits + 6);
      if (bits >= 8) {
        bits = static_cast<uint8_t>(bits - 8);
        if (outLength >= capacity) return false;
        out[outLength++] = static_cast<uint8_t>(accumulator >> bits);
      }
    }
    if ((padding && textLength % 4 != 0) ||
        (padding == 1 && bits != 2) || (padding == 2 && bits != 4) ||
        bits == 6 || (bits && (accumulator & ((1UL << bits) - 1UL)) != 0)) return false;
    return outLength > 0;
  }

  void receivePullResponse(uint16_t token, size_t jsonLength) {
    if (jsonLength == 0 || jsonLength >= MAX_PULL_RESP_JSON) {
      drainListenerPacket(jsonLength);
      report(F("Semtech PULL_RESP JSON is empty or too large"));
      sendTxAck(token, "TX_FREQ");
      return;
    }
    char json[MAX_PULL_RESP_JSON];
    const int got = readPacketSafely(udp, reinterpret_cast<uint8_t *>(json), jsonLength);
    if (got != static_cast<int>(jsonLength)) {
      drainListenerPacket(jsonLength - (got > 0 ? static_cast<size_t>(got) : 0));
      report(F("Could not read complete Semtech PULL_RESP JSON"));
      sendTxAck(token, "TX_FREQ");
      return;
    }
    json[jsonLength] = '\0';

    const char *txpk = strstr(json, "\"txpk\"");
    const char *txpkStart = txpk ? strchr(txpk, '{') : nullptr;
    const char *txpkEnd = txpkStart ? strchr(txpkStart, '}') : nullptr;
    if (!txpkStart || !txpkEnd) {
      report(F("Semtech PULL_RESP has no valid txpk object"));
      sendTxAck(token, "TX_FREQ");
      return;
    }

    double frequencyMHz = 0.0;
    if (!jsonFrequencyField(txpkStart, txpkEnd, "freq", frequencyMHz) ||
        frequencyMHz < 863.0 || frequencyMHz > 870.0) {
      report(F("Semtech PULL_RESP requires a valid EU863-870 frequency"));
      sendTxAck(token, "TX_FREQ");
      return;
    }

    char modulation[8];
    char dataRate[20];
    char codingRate[8];
    unsigned sf = 0, bandwidthKhz = 0;
    char extra = 0;
    bool inverted = false;
    if (!jsonStringField(txpkStart, txpkEnd, "modu", modulation, sizeof(modulation)) ||
        strcmp(modulation, "LORA") != 0 ||
        !jsonStringField(txpkStart, txpkEnd, "datr", dataRate, sizeof(dataRate)) ||
        sscanf(dataRate, "SF%uBW%u%c", &sf, &bandwidthKhz, &extra) != 2 ||
        sf < 7 || sf > 12 ||
        (bandwidthKhz != 125 && bandwidthKhz != 250 && bandwidthKhz != 500) ||
        !jsonStringField(txpkStart, txpkEnd, "codr", codingRate, sizeof(codingRate)) ||
        strcmp(codingRate, "4/5") != 0 ||
        !jsonBoolField(txpkStart, txpkEnd, "ipol", inverted) || !inverted) {
      report(F("Semtech PULL_RESP has unsupported LoRaWAN downlink radio settings"));
      sendTxAck(token, "TX_FREQ");
      return;
    }
    bool noCrc = true;
    uint32_t preamble = 8;
    if ((findJsonField(txpkStart, txpkEnd, "ncrc") &&
         (!jsonBoolField(txpkStart, txpkEnd, "ncrc", noCrc) || !noCrc)) ||
        (findJsonField(txpkStart, txpkEnd, "prea") &&
         (!jsonUnsignedField(txpkStart, txpkEnd, "prea", preamble) || preamble != 8))) {
      report(F("Virtual downlinks require no PHY CRC and an eight-symbol preamble"));
      sendTxAck(token, "TX_FREQ");
      return;
    }

    char encoded[345];
    uint8_t frame[255];
    size_t frameLength = 0;
    uint32_t declaredSize = 0;
    if (!jsonStringField(txpkStart, txpkEnd, "data", encoded, sizeof(encoded)) ||
        !decodeBase64(encoded, frame, sizeof(frame), frameLength) ||
        (findJsonField(txpkStart, txpkEnd, "size") &&
         (!jsonUnsignedField(txpkStart, txpkEnd, "size", declaredSize) ||
          declaredSize != frameLength))) {
      report(F("Semtech PULL_RESP txpk.data is not a valid base64 frame"));
      sendTxAck(token, "TX_FREQ");
      return;
    }

    if (virtualTxPending) {
      report(F("Another virtual downlink is already scheduled"));
      sendTxAck(token, "COLLISION_PACKET");
      return;
    }

    bool immediate = false;
    if (findJsonField(txpkStart, txpkEnd, "imme") &&
        !jsonBoolField(txpkStart, txpkEnd, "imme", immediate)) {
      sendTxAck(token, "TOO_LATE");
      return;
    }
    uint32_t targetMicros = micros();
    if (!immediate) {
      if (!jsonUnsignedField(txpkStart, txpkEnd, "tmst", targetMicros)) {
        report(F("Semtech PULL_RESP has no immediate flag or tmst"));
        sendTxAck(token, "TOO_LATE");
        return;
      }
      const int32_t delta = static_cast<int32_t>(targetMicros - micros());
      if (delta < 0) {
        report(F("Semtech PULL_RESP arrived after its requested tmst"));
        sendTxAck(token, "TOO_LATE");
        return;
      }
      if (static_cast<uint32_t>(delta) > MAX_SCHEDULE_AHEAD_US) {
        report(F("Semtech PULL_RESP tmst is more than 60 seconds ahead"));
        sendTxAck(token, "TOO_EARLY");
        return;
      }
    }

    memcpy(virtualTxFrame, frame, frameLength);
    virtualTxFrameLength = frameLength;
    virtualTxToken = token;
    virtualTxAtMicros = targetMicros;
    virtualTxSettings.radio = {
      static_cast<uint32_t>(frequencyMHz * 1000000.0 + 0.5),
      static_cast<uint32_t>(bandwidthKhz * 1000UL),
      static_cast<uint8_t>(sf), 0
    };
    virtualTxSettings.timestamp = targetMicros;
    virtualTxSettings.airtimeMicros = LoRaAirtime::estimateLoRaAirtimeMicros(
      frameLength, virtualTxSettings.radio);
    virtualTxStarted = false;
    virtualTxPending = true;
    uint32_t routeHoldMs = postUplinkRouteHoldMs();
    if (!immediate) {
      const int32_t delta = static_cast<int32_t>(targetMicros - micros());
      if (delta > 0) {
        const uint32_t scheduledHoldMs =
          (static_cast<uint32_t>(delta) + virtualTxSettings.airtimeMicros + 999UL) / 1000UL +
          POST_DOWNLINK_ROUTE_GRACE_MS;
        if (scheduledHoldMs > routeHoldMs) routeHoldMs = scheduledHoldMs;
      }
    }
    routeKeepaliveUntil = millis() + routeHoldMs;
    reportVerbose(F("TTN downlink accepted for scheduled virtual gateway transmission"));
    processScheduledDownlink();
  }

  void processScheduledDownlink() {
    if (!virtualTxPending) return;
    const int32_t sinceScheduledStart = static_cast<int32_t>(micros() - virtualTxAtMicros);
    if (sinceScheduledStart < 0) return;

    markOperation(OP_SCHEDULED_DOWNLINK);

    if (!virtualTxStarted) {
      if (static_cast<uint32_t>(sinceScheduledStart) > MAX_START_LATENESS_US) {
        report(F("Virtual gateway missed the scheduled downlink start"));
        sendTxAck(virtualTxToken, "TOO_LATE");
        virtualTxPending = false;
        virtualTxFrameLength = 0;
        return;
      }
      virtualTxStarted = true;
      reportVerbose(F("Scheduled virtual LoRa downlink transmission started"));
    }
    if (static_cast<uint32_t>(sinceScheduledStart) < virtualTxSettings.airtimeMicros) return;

    const bool accepted = virtualDownlinkHandler &&
      virtualDownlinkHandler(virtualTxFrame, virtualTxFrameLength, virtualTxSettings);
    if (!accepted) report(F("Virtual transmission completed; endpoint did not accept the frame"));
    // TX_ACK describes gateway transmission. A local device rejection or an
    // absent receiver does not invalidate a completed scheduled transmit.
    sendTxAck(virtualTxToken, "NONE");
    reportVerbose(F("Semtech TX_ACK sent after virtual downlink airtime completed"));
    virtualTxPending = false;
    virtualTxStarted = false;
    virtualTxFrameLength = 0;
  }

  void sendTxAck(uint16_t token, const char *error) {
    uint8_t packet[12];
    packet[0] = PROTOCOL_VERSION;
    packet[1] = static_cast<uint8_t>(token >> 8);
    packet[2] = static_cast<uint8_t>(token);
    packet[3] = TX_ACK;
    memcpy(packet + 4, gatewayEui, sizeof(gatewayEui));
    char json[48];
    const int jsonLength = snprintf(json, sizeof(json),
      "{\"txpk_ack\":{\"error\":\"%s\"}}", error ? error : "TX_FREQ");
    markOperation(OP_TX_ACK_SEND);
    if (jsonLength <= 0 || static_cast<size_t>(jsonLength) >= sizeof(json) ||
        !sendDatagram(udp, packet, sizeof(packet),
                      reinterpret_cast<const uint8_t *>(json),
                      static_cast<size_t>(jsonLength))) {
      report(F("Could not send Semtech UDP TX_ACK"));
    }
  }

  static bool encodeBase64(const uint8_t *input, size_t inputLength,
                           char *output, size_t outputCapacity) {
    static const char alphabet[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    if (!input || !output) return false;
    const size_t required = ((inputLength + 2) / 3) * 4 + 1;
    if (outputCapacity < required) return false;
    size_t out = 0;
    for (size_t in = 0; in < inputLength; in += 3) {
      const size_t remaining = inputLength - in;
      const uint32_t value = (static_cast<uint32_t>(input[in]) << 16) |
        (remaining > 1 ? static_cast<uint32_t>(input[in + 1]) << 8 : 0) |
        (remaining > 2 ? static_cast<uint32_t>(input[in + 2]) : 0);
      output[out++] = alphabet[(value >> 18) & 0x3F];
      output[out++] = alphabet[(value >> 12) & 0x3F];
      output[out++] = remaining > 1 ? alphabet[(value >> 6) & 0x3F] : '=';
      output[out++] = remaining > 2 ? alphabet[value & 0x3F] : '=';
    }
    output[out] = '\0';
    return true;
  }

  static void reportVerbose(const __FlashStringHelper *message) {
#if TTN_NETWORK_VERBOSE
    report(message);
#else
    (void)message;
#endif
  }

  static void report(const __FlashStringHelper *message) {
#if USB_LOGGING
    eventLog.println(message);
#else
    (void)message;
#endif
  }
};

#endif
