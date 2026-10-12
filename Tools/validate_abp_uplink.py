#!/usr/bin/env python3
"""Offline LoRaWAN 1.0.x ABP uplink MIC check for a captured PHYPayload.

Reads NwkSKey/AppSKey/DevAddr from the local Arduino LoraSettings.h and never
prints or transmits those credentials. The default frame is the captured
TTN gs.up.receive payload for FCntUp 6400.
"""

from __future__ import annotations

import argparse
import base64
import re
import sys
from pathlib import Path


DEFAULT_FRAME_B64 = "QKAZCyYAABkBEa2RjZwbjzg0"


_AES_SBOX = bytes((
    0x63,0x7C,0x77,0x7B,0xF2,0x6B,0x6F,0xC5,0x30,0x01,0x67,0x2B,0xFE,0xD7,0xAB,0x76,
    0xCA,0x82,0xC9,0x7D,0xFA,0x59,0x47,0xF0,0xAD,0xD4,0xA2,0xAF,0x9C,0xA4,0x72,0xC0,
    0xB7,0xFD,0x93,0x26,0x36,0x3F,0xF7,0xCC,0x34,0xA5,0xE5,0xF1,0x71,0xD8,0x31,0x15,
    0x04,0xC7,0x23,0xC3,0x18,0x96,0x05,0x9A,0x07,0x12,0x80,0xE2,0xEB,0x27,0xB2,0x75,
    0x09,0x83,0x2C,0x1A,0x1B,0x6E,0x5A,0xA0,0x52,0x3B,0xD6,0xB3,0x29,0xE3,0x2F,0x84,
    0x53,0xD1,0x00,0xED,0x20,0xFC,0xB1,0x5B,0x6A,0xCB,0xBE,0x39,0x4A,0x4C,0x58,0xCF,
    0xD0,0xEF,0xAA,0xFB,0x43,0x4D,0x33,0x85,0x45,0xF9,0x02,0x7F,0x50,0x3C,0x9F,0xA8,
    0x51,0xA3,0x40,0x8F,0x92,0x9D,0x38,0xF5,0xBC,0xB6,0xDA,0x21,0x10,0xFF,0xF3,0xD2,
    0xCD,0x0C,0x13,0xEC,0x5F,0x97,0x44,0x17,0xC4,0xA7,0x7E,0x3D,0x64,0x5D,0x19,0x73,
    0x60,0x81,0x4F,0xDC,0x22,0x2A,0x90,0x88,0x46,0xEE,0xB8,0x14,0xDE,0x5E,0x0B,0xDB,
    0xE0,0x32,0x3A,0x0A,0x49,0x06,0x24,0x5C,0xC2,0xD3,0xAC,0x62,0x91,0x95,0xE4,0x79,
    0xE7,0xC8,0x37,0x6D,0x8D,0xD5,0x4E,0xA9,0x6C,0x56,0xF4,0xEA,0x65,0x7A,0xAE,0x08,
    0xBA,0x78,0x25,0x2E,0x1C,0xA6,0xB4,0xC6,0xE8,0xDD,0x74,0x1F,0x4B,0xBD,0x8B,0x8A,
    0x70,0x3E,0xB5,0x66,0x48,0x03,0xF6,0x0E,0x61,0x35,0x57,0xB9,0x86,0xC1,0x1D,0x9E,
    0xE1,0xF8,0x98,0x11,0x69,0xD9,0x8E,0x94,0x9B,0x1E,0x87,0xE9,0xCE,0x55,0x28,0xDF,
    0x8C,0xA1,0x89,0x0D,0xBF,0xE6,0x42,0x68,0x41,0x99,0x2D,0x0F,0xB0,0x54,0xBB,0x16,
))


def _xtime(value: int) -> int:
    return ((value << 1) ^ (0x11B if value & 0x80 else 0)) & 0xFF


def _aes_round_keys(key: bytes) -> list[bytes]:
    if len(key) != 16:
        raise ValueError("AES-128 key must be 16 bytes")
    expanded = list(key)
    rcon = 1
    while len(expanded) < 176:
        temp = expanded[-4:]
        if len(expanded) % 16 == 0:
            temp = temp[1:] + temp[:1]
            temp = [_AES_SBOX[b] for b in temp]
            temp[0] ^= rcon
            rcon = _xtime(rcon)
        for byte in temp:
            expanded.append(expanded[-16] ^ byte)
    return [bytes(expanded[offset : offset + 16]) for offset in range(0, 176, 16)]


def aes_encrypt_block(key: bytes, block: bytes) -> bytes:
    if len(block) != 16:
        raise ValueError("AES block must be 16 bytes")
    round_keys = _aes_round_keys(key)
    state = list(block)

    def add_round_key(round_number: int) -> None:
        key_bytes = round_keys[round_number]
        for i in range(16):
            state[i] ^= key_bytes[i]

    add_round_key(0)
    for round_number in range(1, 11):
        state[:] = [_AES_SBOX[b] for b in state]
        old = state[:]
        state[:] = [
            old[0],old[5],old[10],old[15], old[4],old[9],old[14],old[3],
            old[8],old[13],old[2],old[7], old[12],old[1],old[6],old[11],
        ]
        if round_number != 10:
            for col in range(4):
                i = 4 * col
                a0, a1, a2, a3 = state[i : i + 4]
                total = a0 ^ a1 ^ a2 ^ a3
                state[i]     ^= total ^ _xtime(a0 ^ a1)
                state[i + 1] ^= total ^ _xtime(a1 ^ a2)
                state[i + 2] ^= total ^ _xtime(a2 ^ a3)
                state[i + 3] ^= total ^ _xtime(a3 ^ a0)
        add_round_key(round_number)
    return bytes(state)


def aes_cmac(key: bytes, message: bytes) -> bytes:
    zero_block = bytes(16)
    l_value = aes_encrypt_block(key, zero_block)

    def double(block: bytes) -> bytes:
        value = int.from_bytes(block, "big")
        carry = value >> 127
        value = (value << 1) & ((1 << 128) - 1)
        if carry:
            value ^= 0x87
        return value.to_bytes(16, "big")

    k1 = double(l_value)
    k2 = double(k1)
    complete = bool(message) and len(message) % 16 == 0
    block_count = max(1, (len(message) + 15) // 16)
    prefix = message[: (block_count - 1) * 16]
    last = message[(block_count - 1) * 16 :]
    if complete:
        last_block = bytes(a ^ b for a, b in zip(last, k1))
    else:
        padded = last + b"\x80" + bytes(15 - len(last))
        last_block = bytes(a ^ b for a, b in zip(padded, k2))

    state = zero_block
    for offset in range(0, len(prefix), 16):
        block = prefix[offset : offset + 16]
        state = aes_encrypt_block(key, bytes(a ^ b for a, b in zip(state, block)))
    return aes_encrypt_block(key, bytes(a ^ b for a, b in zip(state, last_block)))


def load_crypto():
    # Run standard AES and RFC 4493 CMAC vectors so a coding error fails closed.
    aes_key = bytes.fromhex("000102030405060708090A0B0C0D0E0F")
    aes_plain = bytes.fromhex("00112233445566778899AABBCCDDEEFF")
    aes_expected = bytes.fromhex("69C4E0D86A7B0430D8CDB78070B4C55A")
    cmac_key = bytes.fromhex("2B7E151628AED2A6ABF7158809CF4F3C")
    cmac_expected = bytes.fromhex("BB1D6929E95937287FA37D129B756746")
    if aes_encrypt_block(aes_key, aes_plain) != aes_expected:
        raise RuntimeError("Built-in AES self-check failed; refusing to validate frame")
    if aes_cmac(cmac_key, b"") != cmac_expected:
        raise RuntimeError("Built-in CMAC self-check failed; refusing to validate frame")
    return aes_cmac, aes_encrypt_block


def setting(header: str, name: str) -> str:
    # Join C preprocessor continuations before parsing a local settings file.
    header = re.sub(r"\\\r?\n[ \t]*", " ", header)
    identifier = re.escape(name)
    patterns = (
        # #define NAME "..."
        rf"^\s*#\s*define\s+{identifier}\s+(\"[^\"]*\"|'[^']*'|[^\s/]+)",
        # static const char NAME[] = "..." (also accepts pointer declarations)
        rf"^\s*(?:static\s+)?const\s+char\s+(?:\*\s*)?{identifier}"
        rf"\s*(?:\[[^\]]*\])?\s*=\s*(\"[^\"]*\"|'[^']*')",
    )
    match = next(
        (found for pattern in patterns
         if (found := re.search(pattern, header, re.MULTILINE))),
        None,
    )
    if not match:
        raise ValueError(f"Could not find a string definition for {name}")
    value = match.group(1).strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
        value = value[1:-1]
    return value.strip()


def decrypt_frm_payload(key: bytes, dev_addr_wire: bytes, fcnt: int, payload: bytes,
                        aes_encrypt_block) -> bytes:
    result = bytearray()
    for offset in range(0, len(payload), 16):
        block_number = offset // 16 + 1
        a = bytearray(16)
        a[0] = 0x01
        a[5] = 0x00  # uplink direction
        a[6:10] = dev_addr_wire
        a[10:14] = fcnt.to_bytes(4, "little")
        a[15] = block_number
        stream = aes_encrypt_block(key, bytes(a))
        chunk = payload[offset : offset + 16]
        result.extend(a ^ b for a, b in zip(chunk, stream))
    return bytes(result)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--settings",
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / "Arduino/VoorburgSoundsKit/LoraSettings.h",
        help="local settings header (default: this project's Arduino/LoraSettings.h)",
    )
    parser.add_argument(
        "--frame-base64", default=DEFAULT_FRAME_B64,
        help="captured PHYPayload in base64 (default: supplied TTN receive event)",
    )
    parser.add_argument(
        "--fcnt", type=lambda s: int(s, 0), default=6400,
        help="full 32-bit FCntUp from the TTN event (default: 6400)",
    )
    args = parser.parse_args()

    try:
        header = args.settings.read_text(encoding="utf-8")
        try:
            nwk_key = bytes.fromhex(setting(header, "LORAWAN_NWKSKEY_HEX"))
            app_key = bytes.fromhex(setting(header, "LORAWAN_APPSKEY_HEX"))
            configured_addr = bytes.fromhex(setting(header, "LORAWAN_DEVADDR_HEX"))
        except ValueError as exc:
            defined_names = re.findall(
                r"^\s*#\s*define\s+([A-Za-z_][A-Za-z0-9_]*)", header, re.MULTILINE
            )
            defined_names.extend(re.findall(
                r"^\s*(?:static\s+)?const\s+char\s+(?:\*\s*)?([A-Za-z_][A-Za-z0-9_]*)",
                header,
                re.MULTILINE,
            ))
            relevant_names = [
                name for name in defined_names
                if any(token in name.upper() for token in ("NWKSKEY", "APPSKEY", "DEVADDR"))
            ]
            found = ", ".join(relevant_names) if relevant_names else "none"
            raise ValueError(
                f"{exc} in {args.settings}. Related macro names found (values hidden): {found}. "
                "Pass --settings with the exact local header used to build the flashed sketch."
            ) from exc
        frame = base64.b64decode(args.frame_base64, validate=True)
        aes_cmac, aes_encrypt_block = load_crypto()

        if len(nwk_key) != 16 or len(app_key) != 16 or len(configured_addr) != 4:
            raise ValueError("Settings must contain 16-byte keys and a 4-byte DevAddr")
        if len(frame) < 12:
            raise ValueError(f"Frame is too short: {len(frame)} bytes")
        if frame[0] != 0x40:
            raise ValueError(f"Expected unconfirmed data-up MHDR 0x40, got 0x{frame[0]:02X}")
        if frame[1:5] != configured_addr[::-1]:
            raise ValueError(
                "Frame DevAddr does not match configured LORAWAN_DEVADDR_HEX "
                f"(frame {frame[1:5][::-1].hex().upper()}, settings {configured_addr.hex().upper()})"
            )
        if args.fcnt < 0 or args.fcnt > 0xFFFFFFFF:
            raise ValueError("FCntUp must fit in 32 bits")

        wire_fcnt = frame[6] | (frame[7] << 8)
        if (args.fcnt & 0xFFFF) != wire_fcnt:
            raise ValueError(
                f"Supplied full FCntUp {args.fcnt} disagrees with frame low 16 bits {wire_fcnt}"
            )

        mic_offset = len(frame) - 4
        fopts_len = frame[5] & 0x0F
        fport_offset = 8 + fopts_len
        if fport_offset >= mic_offset:
            raise ValueError("Frame has no FPort before its MIC")
        fport = frame[fport_offset]
        ciphertext = frame[fport_offset + 1 : mic_offset]
        if fport == 0:
            raise ValueError("FPort 0 carries MAC commands; this checker expects application data")

        b0 = bytearray(16)
        b0[0] = 0x49
        b0[5] = 0x00  # uplink direction
        b0[6:10] = frame[1:5]
        b0[10:14] = args.fcnt.to_bytes(4, "little")
        b0[15] = mic_offset
        expected_mic = aes_cmac(nwk_key, bytes(b0) + frame[:mic_offset])[:4]
        actual_mic = frame[mic_offset:]

        print(f"Frame bytes: {len(frame)}")
        print(f"DevAddr: {frame[1:5][::-1].hex().upper()}")
        print(f"FCntUp: {args.fcnt}")
        print(f"FPort: {fport}; encrypted application bytes: {len(ciphertext)}")
        print(f"MIC in frame:      {actual_mic.hex().upper()}")
        print(f"MIC calculated:    {expected_mic.hex().upper()}")
        if actual_mic != expected_mic:
            print("RESULT: MIC INVALID for the NwkSKey in this settings header")
            return 1

        plain = decrypt_frm_payload(app_key, frame[1:5], args.fcnt, ciphertext,
                                    aes_encrypt_block)
        print("RESULT: MIC VALID for the configured ABP session")
        print(f"Decrypted application payload: {plain.hex().upper()}")
        return 0
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"Validation could not be completed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
