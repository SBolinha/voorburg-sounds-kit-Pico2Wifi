# Voorburg Sounds Kit — Pico 2 W Wi-Fi

This publication is the Pico 2 W development, based on the [Amsterdam Sounds Kit](https://gitlab.waag.org/lodewijk/amsterdam-sounds-kit). It uses a **Raspberry Pi Pico 2 W** with an **SPH0645 I2S microphone**. The Pico 2 W was selected for Wi-Fi backhaul, local networking, and software gateway flexibility; it has no physical LoRaWAN transceiver. For a battery-oriented device with a real EU868 LoRaWAN radio and Things Indoor gateway, see the companion `voorburg-sounds-kit-FeatherLoraWan` publication.

The dual-core division is explicit: **core 1 owns the PIO/DMA microphone driver, audio buffers, FFT, and five-minute measurement windows; core 0 owns Wi-Fi/NTP, Semtech UDP gateway transport, LoRaWAN frame/MAC/session state, flash journals, and USB logging**. A lock-free mailbox transfers only the newest complete measurement. Flash writes use a generation-matched audio pause/acknowledge handshake. Heartbeats, watchdog supervision, retained reset diagnostics, and bounded network operations improve stability compared with the Amsterdam base.

Compared with the Amsterdam project, this development replaces the physical LoRa radio path with Wi-Fi plus Semtech UDP, adds LoRaWAN frame construction and MAC handling for the software gateway route, UTC window-end timestamps, persistent frame/session state, custom PIO/DMA capture, and recovery diagnostics. The result is a multi-day-capable networked Pico implementation while keeping the Amsterdam acoustic calculation basis.

The published settings contain placeholders only. Create the Pico as its own ABP end device in TTN, open **Overview → Activation information / Session information**, and copy its DevAddr, NwkSKey, and AppSKey into the private local `LoraSettings.h`. Create a gateway in TTN and copy its gateway EUI, server address, and credentials as required by the selected transport. Never commit Wi-Fi passwords, gateway API keys, or TTN session keys. The Feather must use a different ABP identity and frame-counter stream.

A Raspberry Pi Pico 2 W sound-level sensor using an SPH0645 I2S microphone. It reuses the Amsterdam project's A-weighting response and five measurement values, adding a UTC window-end timestamp, then wraps the payload in an ABP LoRaWAN 1.0.x frame.

## Validated RMIC baseline — 9 October 2026

The TTN export `messages.json` contains 30 consecutive timestamped uplinks from
14:19:26 to 16:44:26 UTC, with exactly 300 seconds between measurement timestamps
and consecutive frame counters 65536–65565. No decoded-payload errors were
reported in the export. This establishes 2 hours 25 minutes of publication
continuity; it does not establish long-term power/Wi-Fi outage recovery.

The cleanup build is identified by `dual-core 1; cleanup 3; UDP only; essential logs` in the startup banner.
It preserves the audio, frame encoding, MAC handling and UDP retry logic. The unused alternative transport and replay tooling have been removed.
Routine socket setup and repeated route-step logs are quiet by default; set
`TTN_NETWORK_VERBOSE` to `1` to restore them. Error and publication logs remain
visible. The saved uplink formatter now outputs the five measurements plus `at`
as a UTC ISO timestamp, matching the formatter used in the Console.

Compile and flash the cleanup build with the existing private `LoraSettings.h`.
No key, gateway registration or payload layout changes are required. Retain the
working build/UF2 as the baseline until the cleanup build has completed a longer
run. UDP is unconditional; old transport-switch and key declarations in a private header are ignored.

## TTN Semtech UDP transport

Semtech UDP GWMP v2 is the sole transport. The sketch forwards its ABP LoRaWAN
PHYPayload in `PUSH_DATA/rxpk`, opens the route with `PULL_DATA`, handles
`PUSH_ACK`, `PULL_ACK` and `PULL_RESP`, and answers scheduled downlinks with
`TX_ACK`. Configure your own gateway identity and server in the private settings
file; the publication contains no live gateway credentials or device identity.

The route remains active through receive windows and downlink airtime; its
keepalive then stops until the next publication. Device-side authentication and
MAC processing remain separate from the gateway acknowledgement. Reception
metadata uses configured RSSI and link-margin values. Semtech UDP has no API-key
or TLS-certificate authentication.

## Virtual radio behavior and limits

The local software radio delivers scheduled EU868 LoRa downlinks after their
calculated airtime. The end-device handler authenticates DevAddr/MIC/counters,
decrypts the payload and processes MAC commands. Confirmed downlinks queue an
ACK for the next uplink. Unsupported or late scheduling receives a gateway
error response. The Pico has no physical LoRa radio; this path delivers to its
own local endpoint.

### End-device MAC requests

The four request types present in the RMIC captures are implemented in `LoRaWANMacState.h`:

| TTN request | Device action | Uplink answer |
| --- | --- | --- |
| `RXParamSetupReq` (05) | Validate and apply RX1 offset, RX2 frequency and LoRa data rate | `RXParamSetupAns` with the three acceptance bits |
| `NewChannelReq` (07) | Add, update or remove a configurable EU868 channel; protect default channels 0–2 | `NewChannelAns` with frequency and data-rate acceptance bits |
| `RxTimingSetupReq` (08) | Apply RX1 delay in seconds; RX2 follows one second later | `RxTimingSetupAns` |
| `DevStatusReq` (06) | Report configured power/battery status and signed link margin | `DevStatusAns` |

These are LoRaWAN end-device answers carried in the next uplink's FOpts. The Semtech `TX_ACK` is the separate gateway transmission acknowledgement. Answers are packed whole within the 15-byte FOpts limit. The captured request batch requires 16 answer bytes, so it spans two uplinks. RX parameter and timing answers repeat until an authenticated Class A downlink arrives after their submission, as LoRaWAN 1.0.3 specifies. Pending unsent answers take priority over repetitions. Later requests replacing the same queued answer cannot be removed by completion of an older prepared frame.

Accepted channels are used for subsequent uplinks; RX timing and radio parameters are used to admit subsequent downlinks. LoRa data rates DR0–DR6 are supported; DR7 requires FSK and is rejected. MAC answers remain separate from the microphone application payload and do not change its value encoding. Unsupported or truncated MAC command streams are logged and leave requested settings unchanged.

Negotiated settings, pending answers, FCntDown and confirmed-downlink ACK state are kept in a checksummed RAM journal for watchdog recovery and an EEPROM journal for power loss. The RAM journal additionally preserves an unsent measurement frame and its answer batch. These records are bound to the configured ABP session. Flash commits occur on protocol changes, rather than every measurement. The two journals share the existing FCntUp EEPROM buffer. Checksums detect damaged records; abrupt power loss during an EEPROM flash erase can still invalidate records in that sector.

The boot banner includes `[MAC revision 2; recovery revision 5; dual-core 1]` and the compile date/time. USB output reports each processed MAC request, its queued answer, and the exact FOpts bytes included in an uplink. Recompile and flash the sketch to install these changes; no additional keys or private settings are required for the current Semtech UDP setup.

Recovery revision 4 replaces the sketch's Semtech `WiFiUDP` sockets with a small lwIP backend. Receive callbacks copy complete datagrams into four fixed slots per socket, then immediately free the network buffers. Foreground polling and reading use lock-free 32-bit counters and RAM copies; they never traverse lwIP receive chains, acquire a network lock, or trigger background network processing. Oversized, incomplete and excess datagrams are discarded and counted. Allocation, binding and sending still run in the core's synchronized lwIP context, with failure cleanup and retry. The GWMP packet format and downlink/MAC processing remain the same. This removes the foreground polling path implicated in revision 3's watchdog resets; continuous hardware operation still needs confirmation.

Core 1 stops capture after producing each payload, resets its sample/analysis counters and statistics, and starts the next window without waiting for publication. Core 0 requests a bounded audio pause only for EEPROM commits/counter reservations; the producer acknowledges stopped DMA before flash work, then resumes with its accumulated statistics intact. Watchdog markers now distinguish counter reservation, frame authentication, protocol journal saves and microphone operations, and clear completed network work so a later stall cannot be mislabelled as UDP polling. The checksummed retained-RAM record is copied byte-for-byte to preserve its padding representation.

Before each publication, USB output reports the longest completed loop, free heap and available stack at that point. The loop duration includes waits and interrupt work; it is not a CPU utilization percentage. The stack value is a snapshot, not a peak-use measurement. The longest completed loop is also retained for the next watchdog-reset report; a loop that stalls cannot contribute a completed duration.

Recovery revision 5 also pauses audio around protocol journal commits, preserving accumulated measurement statistics. Flash erase/program operations mask interrupts; running chained audio DMA through a commit could exhaust both blocks and leave capture stopped. The microphone backend now owns fixed DMA blocks and a bounded sample ring, with a deadline on DMA abort and explicit restart when both channels have stopped. DMA blocks use software ping-pong, so a delayed interrupt cannot trigger a write at an old end-of-buffer address. Its PIO input clocking follows the core's 32-bit stereo I2S program. It avoids the installed AudioBufferManager's linked lists, channel-table bounds, and per-window allocation. A failed audio pause defers the flash save and schedules recovery. These changes address concrete source defects and lifecycle risks; they do not establish the cause of every observed reset.

Wi-Fi attempts use a shorter association timeout, and NTP DNS resolves one hostname per pass with a 1.5-second limit. The core's Wi-Fi driver still contains waits outside that timeout, so the hardware watchdog remains the fallback for a driver stall. Reset markers now cover Wi-Fi state/join, socket setup, DNS/NTP and audio recovery, and retain an explicit context between foreground operations.

### Two-core operation and measurement deadlines

- **Core 1:** microphone PIO/DMA, FFT/level statistics, and the nine-byte application payload. Windows end on a fixed five-minute monotonic schedule, independent of publication. After producing a payload, the audio core stops capture, resets its sample counter and statistics, and starts the next window independently of delivery.
- **Core 0:** Wi-Fi/NTP, gateway transport, LoRaWAN/MAC/session counters, flash storage, and USB logging. No SLM/audio statistics are read directly from this core. Both cores have separate 8 KiB stacks.
- A fixed mailbox transfers a complete snapshot through a single nonblocking lock attempt. Audio retries a busy handoff on its next loop pass; it never waits for networking. The mailbox and traffic core retain only the newest measurement.
- Delivery retries preserve the frame and its measurement timestamp. At the next window boundary, an unsent older measurement is discarded; its unsubmitted MAC answers and confirmed-downlink ACK remain queued. Expiry is checked again immediately before submission. Wi-Fi loss prevents publication without stopping the independent sequence of audio windows.
- Bytes 5–8 carry the **UTC end of the measurement window**, in unsigned Unix seconds, little-endian. This is separate from TTN's reception time. Core 0 publishes a synchronized UTC/monotonic anchor; core 1 never calls Wi-Fi or the core's shared libc clock. Until UTC is known, a measurement is held within its delivery deadline.
- EEPROM commits require a generation-matched core-1 stop acknowledgement. Failed/stale acknowledgements and one-second handshake timeouts defer the write. Arduino-Pico's existing inter-core flash lockout remains responsible for safe erase/program execution; the sketch does not nest a second lockout.
- Core 0 feeds the hardware watchdog only while core 1's completed-loop heartbeat is less than three seconds old. Audio continues to publish heartbeats while waiting through a flash pause. A driver stall can still require a reset of both cores; hardware continuity must be confirmed after flashing.

### Locating a watchdog stall

A spare timer attempts to capture core 0's interrupted PC/LR before the eight-second watchdog expires. A default HardFault handler captures CPU fault registers. The first capture is retained with a CRC, and boot output reports the core, instruction addresses and operation. Core 1 additionally retains its last audio operation. The timer cannot preempt masked interrupts or an equal-priority interrupt stall; a missing PC does not mean no stall occurred.

Keep the **exact `.elf` from the firmware build that was flashed**. Arduino's verbose compilation output identifies its build directory. After a captured PC is available, resolve it locally:

```sh
python3 Tools/resolve_pico_fault.py /path/to/VoorburgSoundsKit.ino.elf --pc 0xADDRESS --lr 0xADDRESS
```

A newly compiled ELF cannot reliably locate an older firmware's addresses. The decoder prints source locations, not firmware strings or keys. An interrupted instruction identifies where execution stopped; the originating corruption or deadlock may have happened earlier.

USB logs are assembled as complete lines in a bounded RAM queue and drained in small chunks only when the serial transmit buffer has space. On reconnect the current partial line is replayed; queue overflow drops complete lines and reports the loss. USB output has its own reset marker. These measures prevent normal buffer backpressure from making each log statement wait for USB output; they cannot eliminate a fault elsewhere in the USB driver.

Offline host checks:

```sh
c++ -std=c++11 -Wall -Wextra -Werror -pedantic Tools/test_lorawan_mac_state.cpp -o /tmp/voorburg-mac-test
/tmp/voorburg-mac-test
c++ -std=c++11 -Wall -Wextra -Werror -pedantic -ITools/host_stubs Tools/test_lorawan_session_store.cpp -o /tmp/voorburg-session-test
/tmp/voorburg-session-test
python3 Tools/test_udp_receive_lock.py
python3 Tools/test_flash_audio_safety.py
python3 Tools/test_measurement_pipeline.py
python3 Tools/test_audio_worker.py
c++ -std=c++17 -Wall -Wextra -Werror -pedantic -pthread Tools/test_measurement_handoff.cpp -o /tmp/voorburg-handoff-test
/tmp/voorburg-handoff-test
c++ -std=c++20 -Wall -Wextra -Werror -pedantic -ITools/audio_host_stubs -IArduino/VoorburgSoundsKit Tools/test_pico_i2s_input.cpp Arduino/VoorburgSoundsKit/PicoI2SInput.cpp -o /tmp/voorburg-audio-test
/tmp/voorburg-audio-test
c++ -std=c++20 -Wall -Wextra -Werror -pedantic -ITools/audio_host_stubs -IArduino/VoorburgSoundsKit Tools/test_slm_continuity.cpp Arduino/VoorburgSoundsKit/PicoI2SInput.cpp -o /tmp/voorburg-slm-test
/tmp/voorburg-slm-test
node Tools/test_payload_formatter.js
c++ -std=c++11 -Wall -Wextra -Werror -pedantic -ITools/host_log_stubs Tools/test_event_log.cpp -o /tmp/voorburg-log-test
/tmp/voorburg-log-test
```

`Tools/test_lorawan_frames.cpp` additionally checks the sketch's actual frame/CMAC code against independent public-key fixtures using the OpenSSL host adapter in `Tools/host_stubs`. It covers FOpts encoding, decrypted MAC commands, persisted FCntDown, authenticated confirmed retransmissions and counter rollover. That adapter is only for host tests; Arduino continues to use the installed Crypto library.


## Sensor payload

The new LoRaWAN uplink uses FPort 1 and a nine-byte application payload. The first five bytes retain the original values and scaling:

| Byte | Value | Encoding |
| --- | --- | --- |
| 0 | Mean relative level | Unsigned Q7.1, value × 2 |
| 1 | Minimum relative level | Unsigned Q7.1, value × 2 |
| 2 | Maximum relative level | Unsigned Q7.1, value × 2 |
| 3 | Standard deviation | Unsigned Q5.3, value × 8 |
| 4 | LEQ relative level | Unsigned Q7.1, value × 2 |
| 5–8 | UTC measurement window end | Unsigned 32-bit Unix seconds, little-endian |

The levels are relative and uncalibrated; they are not absolute dBA measurements. If you want named fields in TTN application Live Data, paste [`Tools/ttn_uplink_payload_formatter.js`](Tools/ttn_uplink_payload_formatter.js) into the end device's **uplink payload formatter**. The formatter decodes the application bytes only; it does not change the gateway Protobuf.

The decoded fields are `mean`, `min`, `max`, `stddev`, `leq`, `at`. USB output reports only the raw nine application bytes as `PAYLOAD_HEX=...`, before LoRaWAN encryption. Update the TTN formatter from `Tools/ttn_uplink_payload_formatter.js` before flashing this revision. It also accepts legacy five-byte messages. A formatter-test payload is `8F4EA51E92009FC86A`.

## Microphone wiring

Use Pico 2 W **GPIO numbers**, not the board header numbering. The pins match `Arduino/VoorburgSoundsKit/SLMSettings.h`.

| SPH0645 breakout | Raspberry Pi Pico 2 W |
| --- | --- |
| 3V | 3V3(OUT) |
| GND | GND |
| BCLK | GP10 |
| LRCL / WS | GP11 (I2S driver sets LRCLK to BCLK + 1) |
| DOUT | GP12 |
| SEL | 3V3(OUT) for right channel |

The microphone's SEL and power connections can use separate 3V3(OUT) pins. Connect grounds together and do not power the microphone from 5 V.

## Arduino setup

1. Install the Earle Philhower **Raspberry Pi Pico/RP2040/RP2350** board package and select **Raspberry Pi Pico 2 W**.
2. Install **Crypto** by Rhys Weatherley from Library Manager.
3. Open `Arduino/VoorburgSoundsKit/VoorburgSoundsKit.ino`.
4. Fill in your local `Arduino/VoorburgSoundsKit/LoraSettings.h`: Wi-Fi credentials, gateway credentials, and the Pico's separate ABP DevAddr, NwkSKey, AppSKey and initial FCntUp. Keep this private file out of shared copies.
5. Set `TTN_GATEWAY_EUI_HEX` to the EUI registered in the Gateway Console, and set the Gateway Server host. The default UDP port is 1700. UDP has no gateway API key or TLS root CA setting.
6. Keep `TTN_UPLINK_ENABLED` set to `1`. `USB_LOGGING` enables USB diagnostics; `USB_MIC_LOGGING` controls the continuous microphone lines.
7. Select the Pico 2 W's USB port and compile/flash from Arduino IDE. The default measurement period is 300 seconds (`SEND_AFTER`). A local UDP send error closes and rebuilds the port-1700 socket, then retries the same prepared frame with capped backoff. Semtech UDP itself does not retry a datagram if `PUSH_ACK` is lost, as specified by the protocol.

The firmware waits for NTP to establish UTC timestamps. Keep Wi-Fi credentials and LoRaWAN session keys private.

## TTN configuration

- Use a separate UDP gateway registration for your deployment.
- For Semtech UDP, register a Gateway EUI and configure the Gateway Server address and UDP port 1700. The Gateway EUI in TTN and `TTN_GATEWAY_EUI_HEX` must match exactly. Semtech UDP has no API-key authentication; do not enable “Require authenticated connection” for this transport.
- Register the Pico separately as an ABP end device in the application. Use its own DevAddr, NwkSKey, AppSKey and frame-counter stream; never run the Feather and Pico with the same ABP identity at once.
- Use an EU868 frequency plan matching the default uplink settings (868.3 MHz, SF7, BW125, coding rate 4/5). The gateway registration and end-device frequency plan must agree.

## Protocol references

- [The Things Stack gateway setup and EUI](https://www.thethingsindustries.com/docs/hardware/gateways/concepts/adding-gateways/)
- [The Things Stack gateway troubleshooting, including UDP port 1700](https://www.thethingsindustries.com/docs/hardware/gateways/troubleshooting/)
- [Semtech Packet Forwarder GWMP protocol](https://github.com/Lora-net/packet_forwarder/blob/master/PROTOCOL.TXT)
- [LoRaWAN 1.0.3 MAC command specification](https://lora-alliance.org/wp-content/uploads/2020/11/lorawan1.0.3.pdf)
- [The Things Stack v3 messages.proto](https://github.com/TheThingsNetwork/lorawan-stack/blob/v3.36/api/ttn/lorawan/v3/messages.proto)
- [The Things Stack v3 lorawan.proto](https://github.com/TheThingsNetwork/lorawan-stack/blob/v3.36/api/ttn/lorawan/v3/lorawan.proto)

## Sound-level calculation

Audio capture runs at 48 kHz over I2S and analyzes 1024-sample windows at up to 32 frames per second. It applies a Hann window and weights FFT bins with the microphone response and A-weighting table reused from the Amsterdam project. Absolute microphone calibration is not provided.

`USB_LOGGING` reports Wi-Fi, selected transport, TTN gateway acknowledgements and frame counters. Set `USB_MIC_LOGGING` to `1` to include continuous I2S and relative sound-level diagnostics.

### Essential USB logging

Normal operation logs the build/startup identity, Wi-Fi connection changes, UTC
synchronization, and one `PAYLOAD_HEX=` line per prepared measurement. That line
contains the nine unencrypted application bytes (five measurements plus the
four timestamp bytes). Decoded values, MAC answer dumps, success self-checks,
and routine publication/downlink progress are silent. Errors and abnormal
restart diagnostics remain available. `TTN_NETWORK_VERBOSE=1` explicitly
re-enables network tracing when needed. No TTN formatter change is required.

### Calibration comparison to the RP2040 M0 LoRaWAN microphone
the RP2040 Feather preserves the M0’s calculation basis and nominal microphone calibration. I compared our source with the original M0 version and confirmed:
- Same 48 kHz sampling, 1,024-sample FFT, and 32 analyses per second.
- Identical A-weighting and microphone frequency-response correction tables.
- Same Hann normalization, fixed-point scaling, and 120 dB reference offset.
- Same calculations for mean, minimum, maximum, standard deviation, and energy-averaged Leq.
The RP2040 changes concern audio capture, buffering, and efficiency; they introduce no additional gain correction. SLM and CMSIS-DSP perform the acoustic calculations; LMIC carries the results over LoRaWAN.
You can therefore assume the same intended calibration, provided the microphone is the same SPH0645LM4H-B model. Physical accuracy still needs a reference comparison: that microphone has a specified ±3 dB sensitivity tolerance, and mounting can affect its response. Knowles datasheet.
