#!/usr/bin/env python3
"""CLI helpers to build and decode DDSM115 protocol frames.

Usage examples:
  # Build a drive command for motor 1 at 100 RPM
  ./ddsm115_protocol_tool.py build-drive --id 1 --rpm 100

  # Build a set-id frame for new id 5
  ./ddsm115_protocol_tool.py build-setid --id 5

  # Decode a 10-byte response (hex bytes)
  ./ddsm115_protocol_tool.py decode "01 64 00 64 10 00 00 80 00 3A"

This script implements Maxim CRC-8 to match the driver code.
"""

import argparse
import sys
from typing import List


def maxim_crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        inbyte = b
        for _ in range(8):
            mix = (crc ^ inbyte) & 0x01
            crc >>= 1
            if mix:
                crc ^= 0x8C
            inbyte >>= 1
    return crc


def build_drive_cmd(motor_id: int, rpm: int) -> bytes:
    rpm_val = rpm & 0xFFFF
    cmd = bytearray([motor_id & 0xFF, 0x64, (rpm_val >> 8) & 0xFF, rpm_val & 0xFF,
                     0, 0, 0, 0, 0])
    cmd.append(maxim_crc8(bytes(cmd)))
    return bytes(cmd)


def build_setid_cmd(new_id: int) -> bytes:
    cmd = bytearray([0xAA, 0x55, 0x53, new_id & 0xFF, 0, 0, 0, 0, 0])
    cmd.append(maxim_crc8(bytes(cmd)))
    return bytes(cmd)


def parse_hex_string(s: str) -> bytes:
    parts = s.strip().split()
    return bytes(int(p, 16) for p in parts)


def decode_response(resp: bytes, expected_id: int = None) -> dict:
    if len(resp) < 10:
        raise ValueError("response too short; expected 10 bytes")
    if expected_id is not None and resp[0] != expected_id:
        raise ValueError(f"id mismatch: got {resp[0]}, expected {expected_id}")
    crc_ok = resp[9] == maxim_crc8(resp[:9])

    drive_current = (resp[2] << 8) | resp[3]
    if drive_current & 0x8000:
        drive_current -= 0x10000

    drive_velocity = (resp[5] << 8) | resp[4]
    if drive_velocity & 0x8000:
        drive_velocity -= 0x10000

    drive_position = (resp[6] << 8) | resp[7]

    current_A = drive_current * (8.0 / 32767.0)
    velocity_rpm = float(drive_velocity)
    velocity_rad_s = velocity_rpm / 60.0 * 2.0 * 3.141592653589793
    position_deg = drive_position * (360.0 / 32767.0)

    return {
        "id": resp[0],
        "cmd": resp[1],
        "raw_current": drive_current,
        "current_A": current_A,
        "raw_velocity": drive_velocity,
        "velocity_rpm": velocity_rpm,
        "velocity_rad_s": velocity_rad_s,
        "raw_position": drive_position,
        "position_deg": position_deg,
        "crc_ok": crc_ok,
    }


def hexify(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


def main(argv: List[str]):
    parser = argparse.ArgumentParser(description="DDSM115 protocol builder/decoder")
    sub = parser.add_subparsers(dest="cmd")

    p_build_drive = sub.add_parser("build-drive")
    p_build_drive.add_argument("--id", type=int, required=True)
    p_build_drive.add_argument("--rpm", type=int, required=True)

    p_setid = sub.add_parser("build-setid")
    p_setid.add_argument("--id", type=int, required=True)

    p_decode = sub.add_parser("decode")
    p_decode.add_argument("hex", help="space-separated hex bytes, e.g. '01 64 ...'")
    p_decode.add_argument("--expected-id", type=int, help="optionally check ID")

    args = parser.parse_args(argv)
    if args.cmd == "build-drive":
        pkt = build_drive_cmd(args.id, args.rpm)
        print(hexify(pkt))
        return 0

    if args.cmd == "build-setid":
        pkt = build_setid_cmd(args.id)
        print(hexify(pkt))
        return 0

    if args.cmd == "decode":
        resp = parse_hex_string(args.hex)
        info = decode_response(resp, args.expected_id)
        for k, v in info.items():
            print(f"{k}: {v}")
        return 0

    parser.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
