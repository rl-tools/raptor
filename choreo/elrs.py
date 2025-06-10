import argparse
import struct
import sys
import time
from collections import deque
from datetime import datetime
from typing import Deque, List, Tuple

import serial


POLY = 0xD5
SYNC_ADDR = 0xC8                       # also Flight-Controller address
ADDRS_START = {0xC8, 0xEA, 0xEE}       # FC, RadioTX, CRSF TX
FT_RC       = 0x16                     # RC_CHANNELS_PACKED
FT_LINKSTAT = 0x14                     # Link statistics
FT_BATTERY  = 0x08                     # Battery sensor
FT_GPS      = 0x02                     # GPS, etc. (example only)

RC_PAYLOAD_LEN = 22                    # 16 × 11-bit
RC_FRAME_SIZE  = 1 + RC_PAYLOAD_LEN + 1


def _make_table(poly: int) -> List[int]:
    tbl = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFF if (c & 0x80) else (c << 1) & 0xFF
        tbl.append(c)
    return tbl


_CRC_TBL = _make_table(POLY)


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = _CRC_TBL[crc ^ b]
    return crc


def _pack_channels(ch: List[int]) -> bytes:
    if len(ch) != 16:
        raise ValueError("Exactly 16 channels required")
    ch = [max(0, min(0x7FF, int(v))) for v in ch]

    buf = bytearray(RC_PAYLOAD_LEN)
    for i, v in enumerate(ch):
        byte_idx = (i * 11) // 8
        bit_off  = (i * 11) % 8

        buf[byte_idx]     |= (v << bit_off) & 0xFF
        buf[byte_idx + 1] |= (v >> (8 - bit_off)) & 0xFF
        if bit_off > 5:
            buf[byte_idx + 2] |= (v >> (16 - bit_off)) & 0xFF
    return bytes(buf)


def build_rc_frame(channels: List[int]) -> bytes:
    payload = _pack_channels(channels)
    crc     = crc8(bytes([FT_RC]) + payload)
    return bytes([SYNC_ADDR, RC_FRAME_SIZE, FT_RC]) + payload + bytes([crc])


def _parse_linkstats(payload: bytes) -> str:
    if len(payload) != 10:
        return f"LinkStats invalid length {len(payload)}"
    (rssi1_inv, rssi2_inv, lq_up, snr_up, ant, rf_mode,
     txpwr, rssi_d_inv, lq_dn, snr_dn) = struct.unpack('<BBBBBBBbbb', payload)
    return (f"LinkStat:  RSSI1={-rssi1_inv}dBm  RSSI2={-rssi2_inv}dBm  "
            f"LQ={lq_up}%  SNR={snr_up}dB  RFmode={rf_mode}  TxPwrIdx={txpwr}  "
            f"DownRSSI={-rssi_d_inv}dBm  DownLQ={lq_dn}%  DownSNR={snr_dn}dB")


def _parse_battery(payload: bytes) -> str:
    """
    Battery-sensor frame (type 0x08)

    Layout in C++:
        u16 voltage   // big-endian  (mV * 100)
        u16 current   // big-endian  (mA * 100)
        u32 capacity  // little-endian:
                      #   lower 24 bits  = capacity [mAh]
                      #   upper  8 bits  = remaining [%]
    """
    if len(payload) != 8:
        return f"Battery invalid length {len(payload)}"

    voltage_raw, current_raw = struct.unpack(">HH", payload[:4])   # big-endian
    cap_pack,                 = struct.unpack("<I", payload[4:])   # little-endian

    voltage  = voltage_raw  / 10.0            # volts
    current  = current_raw  / 100.0           # amps
    capacity = (cap_pack & 0xFFFFFF) / 1000.0 # amp-hours
    remain   = cap_pack >> 24                 # percent

    return (f"Battery:  {voltage:.2f} V  {current:.2f} A  "
            f"{capacity:.2f} Ah  {remain}%")

# Map frame-type → decoder
_DECODERS = {
    FT_LINKSTAT: _parse_linkstats,
    FT_BATTERY : _parse_battery,
    # add more if you need them …
}


def frames_from_bytes(buf: Deque[int]):
    """
    In-place parser – consumes bytes from *buf* and yields complete frames.
    """
    while True:
        # Need at least addr + size + type
        if len(buf) < 3:
            return

        # Sync: discard bytes until a plausible address shows up
        if buf[0] not in ADDRS_START:
            buf.popleft()
            continue

        if len(buf) < 2:
            return
        size = buf[1]
        frame_total = size + 2
        if len(buf) < frame_total:
            return

        # Validate CRC
        crc_in  = buf[frame_total - 1]
        calc_crc = crc8(bytes(list(buf)[2:frame_total - 1]))
        if crc_in != calc_crc:
            # bad frame – skip first byte and resync
            buf.popleft()
            continue

        # All good – extract
        addr   = buf.popleft()
        size   = buf.popleft()          # discard, we know it
        ftype  = buf.popleft()
        payload = bytes(buf.popleft() for _ in range(frame_total - 3 - 1))
        buf.popleft()  # remove CRC byte
        yield addr, ftype, payload


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Minimal CRSF TX + Telemetry RX")
    p.add_argument("port", help="Serial port (e.g. COM3 or /dev/ttyACM0)")
    p.add_argument("baud", nargs="?", type=int, default=921600,
                   help="Baud rate (default 921600)")
    p.add_argument("--ch", type=int, nargs="+",
                   help="Up to 16 raw channel values (0-2047). Missing → 1024")
    p.add_argument("--rate", type=float, default=50.0,
                   help="Transmit rate in Hz (default 50)")
    return p.parse_args()

# Standard CRSF channel values (100% endpoints)
# Centered at 992, with a range of +/- 819.
# The 11-bit value sent is from 0 to 2047.
# 992 is the "zero" or center-stick value.
# 172 is the -100% value.
# 1811 is the +100% value.
RC_CHANNEL_MIN = 172
RC_CHANNEL_MID = 992
RC_CHANNEL_MAX = 1811


def main() -> None:
    args = _parse_args()


    channels = [RC_CHANNEL_MID] * 16
    if args.ch:
        channels[:len(args.ch)] = [int(v) for v in args.ch[:16]]

    rc_frame = build_rc_frame(channels)
    period   = 1.0 / args.rate

    ring: Deque[int] = deque(maxlen=512)   # crude RX buffer

    try:
        with serial.Serial(args.port, args.baud, timeout=0) as ser:
            print(f"[{datetime.now().strftime('%H:%M:%S')}] "
                  f"opened {ser.port} @ {ser.baudrate} baud")
            next_tx = time.perf_counter()

            while True:
                # ---------- TX ----------
                now = time.perf_counter()
                if now >= next_tx:
                    ser.write(rc_frame)
                    ser.write(rc_frame)     # duplicate like ExpressLRS
                    next_tx += period

                # ---------- RX ----------
                data = ser.read(ser.in_waiting or 1)
                ring.extend(data)

                for addr, ftype, payload in frames_from_bytes(ring):
                    ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                    if ftype in _DECODERS:
                        print(f"[{ts}] {_DECODERS[ftype](payload)}")
                    else:
                        print(f"[{ts}] Frame 0x{ftype:02X} "
                              f"(len={len(payload)}) from 0x{addr:02X}")

                # Keep CPU usage low
                time.sleep(0.001)

    except serial.SerialException as e:
        print(f"Serial error: {e}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()

