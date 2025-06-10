import argparse
import sys
import time
from typing import List

import serial

POLYNOMIAL = 0xD5               # CRC-8 polynomial
DEVICE_ADDRESS = 0xC8           # 0xC8 = Flight Controller address
FRAME_TYPE_RC = 0x16            # RC_CHANNELS_PACKED
PAYLOAD_LEN = 22                # 16 channels * 11 bits
FRAME_SIZE = 1 + PAYLOAD_LEN + 1  # type + payload + crc (24 bytes)

# --------------------------------------------------------------------------- #
# CRC-8 (polynomial 0xD5, initial 0, input reflected, output non-reflected)
# --------------------------------------------------------------------------- #
def _make_crc_table(poly: int) -> List[int]:
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        table.append(crc)
    return table

_CRC_TABLE = _make_crc_table(POLYNOMIAL)

def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = _CRC_TABLE[crc ^ b]
    return crc

# --------------------------------------------------------------------------- #
# Pack 16 unsigned 11-bit channel values into 22 bytes (little-endian, LSB-first)
# --------------------------------------------------------------------------- #
def pack_channels(ch: List[int]) -> bytes:
    if len(ch) != 16:
        raise ValueError("Exactly 16 channel values required.")
    # Clamp to 11-bit range
    ch = [max(0, min(0x7FF, int(v))) for v in ch]

    payload = bytearray(PAYLOAD_LEN)

    for i, val in enumerate(ch):
        byte_idx = (i * 11) // 8
        bit_off  = (i * 11) % 8

        payload[byte_idx]     |= (val << bit_off) & 0xFF
        payload[byte_idx + 1] |= (val >> (8 - bit_off)) & 0xFF
        if bit_off > 5:  # value spills into three bytes
            payload[byte_idx + 2] |= (val >> (16 - bit_off)) & 0xFF

    return bytes(payload)

# --------------------------------------------------------------------------- #
# Build a complete CRSF frame
# --------------------------------------------------------------------------- #
def build_rc_frame(ch: List[int]) -> bytes:
    payload = pack_channels(ch)
    crc = crc8(bytes([FRAME_TYPE_RC]) + payload)
    return bytes([
        DEVICE_ADDRESS,
        FRAME_SIZE,
        FRAME_TYPE_RC
    ]) + payload + bytes([crc])

# --------------------------------------------------------------------------- #
# CLI glue and main loop
# --------------------------------------------------------------------------- #
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Send CRSF RC_CHANNELS_PACKED frames over UART.")
    p.add_argument("port", help="Serial port, e.g. COM3 or /dev/ttyACM0")
    p.add_argument("baud", nargs="?", type=int, default=921600,
                   help="Baud rate (default 921600)")
    p.add_argument("--ch", metavar="VAL", type=int, nargs="+",
                   help="Up to 16 raw channel values (0-2047). "
                        "Missing channels default to 1024.")
    p.add_argument("--rate", type=float, default=50.0,
                   help="Send rate in Hz (default 50)")
    return p.parse_args()

def main() -> None:
    args = parse_args()

    # Default center values
    channels = [2**11] + [1024] * 15
    if args.ch:
        if len(args.ch) > 16:
            print("More than 16 channels specified; extras ignored.", file=sys.stderr)
        channels[:len(args.ch)] = args.ch[:16]

    frame = build_rc_frame(channels)

    try:
        with serial.Serial(args.port, args.baud, timeout=0) as ser:
            print(f"Opened {ser.port} @ {ser.baudrate} baud; streaming frames.")
            period = 1.0 / args.rate
            while True:
                ser.write(frame)
                ser.write(frame)  # duplicate
                ser.flush()
                time.sleep(period)
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nStopped.")

if __name__ == "__main__":
    main()



