"""Week4 streaming protocol; P/I decoder adapted from codingday3/new/member1/pc.

P, I and N have independent sequence spaces. Diagnostic fields remain raw strings so
firmware can extend its diagnostic schema without breaking navigation display.
"""
from __future__ import annotations

from dataclasses import dataclass
import re

U32_MAX = (1 << 32) - 1
I32_MIN, I32_MAX = -(1 << 31), (1 << 31) - 1
_UINT = re.compile(rb"[0-9]{1,10}\Z")
_INT = re.compile(rb"-?[0-9]{1,10}\Z")


@dataclass(frozen=True)
class Point:
    seq: int
    ms: int
    angle_cdeg: int
    distance_mm: int
    quality: int
    new_scan: bool


@dataclass(frozen=True)
class Imu:
    seq: int
    ms: int
    heading_cdeg: int
    gz_cdps: int


def _number(token: bytes, low: int, high: int, signed: bool = False) -> int:
    if not (_INT if signed else _UINT).fullmatch(token):
        raise ValueError("invalid integer")
    value = int(token)
    if not low <= value <= high:
        raise ValueError("integer out of range")
    return value


@dataclass(frozen=True)
class Navigation:
    seq: int
    ms: int
    cmd_id: int
    state: str
    x_mm: int
    y_mm: int
    theta_mrad: int
    target_x_mm: int
    target_y_mm: int
    waypoint: int
    fault: int


@dataclass(frozen=True)
class Ack:
    cmd_id: int
    status: str
    reason: str


@dataclass(frozen=True)
class Diagnostic:
    kind: str
    fields: tuple[str, ...]


def command(text: str) -> bytes:
    """Encode the firmware command grammar; coordinates are metres."""
    import math
    if '\n' in text or '\r' in text:
        raise ValueError('one command at a time')
    parts = text.strip().upper().split(',')
    if parts in (['STOP'], ['RESET']):
        return (parts[0] + '\n').encode('ascii')
    if len(parts) == 3 and parts[0] == 'GOTO':
        x, y = float(parts[1]), float(parts[2])
        if math.isfinite(x) and math.isfinite(y) and abs(x) <= 2 and abs(y) <= 2:
            return f'GOTO,{x:.6f},{y:.6f}\n'.encode('ascii')
    raise ValueError('use GOTO,x,y (metres, +/-2), STOP or RESET')


def parse_line(line: bytes) -> Point | Imu | Navigation | Ack | Diagnostic:
    """Parse one line without LF. Reject whitespace, non-ASCII and bad ranges."""
    parts = line.split(b",")
    if any(byte < 32 or byte > 126 for byte in line):
        raise ValueError('non printable frame')
    if len(parts) == 12 and parts[0] == b'N':
        state = parts[4].decode('ascii')
        if state not in ('IDLE', 'PLANNING', 'TRACKING', 'ARRIVED', 'STOPPED', 'FAULT'):
            raise ValueError('unknown navigation state')
        return Navigation(*[_number(p, 0, U32_MAX) for p in parts[1:4]], state,
                          *[_number(p, I32_MIN, I32_MAX, True) for p in parts[5:10]],
                          _number(parts[10], 0, 65535), _number(parts[11], 0, U32_MAX))
    if len(parts) == 4 and parts[0] == b'A' and parts[2] and parts[3]:
        return Ack(_number(parts[1], 0, U32_MAX), parts[2].decode(), parts[3].decode())
    if len(parts) >= 2 and parts[0] in (b'D', b'T', b'B', b'S', b'Q'):
        return Diagnostic(parts[0].decode(), tuple(p.decode() for p in parts[1:]))
    if len(parts) == 7 and parts[0] == b"P":
        return Point(_number(parts[1], 0, U32_MAX), _number(parts[2], 0, U32_MAX),
                     _number(parts[3], 0, 35999), _number(parts[4], 0, U32_MAX),
                     _number(parts[5], 0, 63), bool(_number(parts[6], 0, 1)))
    if len(parts) == 5 and parts[0] == b"I":
        return Imu(_number(parts[1], 0, U32_MAX), _number(parts[2], 0, U32_MAX),
                   _number(parts[3], -18000, 18000, True),
                   _number(parts[4], I32_MIN, I32_MAX, True))
    raise ValueError("unknown frame or wrong field count")


class StreamDecoder:
    """A bounded incremental decoder. Oversized lines are discarded through LF."""
    def __init__(self, max_line: int = 512):
        if max_line < 1:
            raise ValueError('max_line must be positive')
        self.max_line = max_line
        self.buffer = bytearray()
        self.discarding = False
        self.bad_lines = 0

    def feed(self, data: bytes) -> list[Point | Imu]:
        frames: list[Point | Imu] = []
        for byte in data:
            if self.discarding:
                if byte == 10:
                    self.discarding = False
                continue
            if byte == 10:
                try:
                    frames.append(parse_line(bytes(self.buffer)))
                except ValueError:
                    self.bad_lines += 1
                self.buffer.clear()
            elif len(self.buffer) >= self.max_line:
                self.bad_lines += 1
                self.buffer.clear()
                self.discarding = True
            else:
                self.buffer.append(byte)
        return frames

    def reset_partial(self) -> None:
        self.buffer.clear()
        self.discarding = False


class SequenceTracker:
    """Count forward gaps without mistaking duplicates or reordering for loss."""
    def __init__(self):
        self.last: int | None = None
        self.missing = 0
        self.duplicates = 0
        self.out_of_order = 0

    def observe(self, seq: int) -> bool:
        if self.last is None:
            self.last = seq
            return True
        delta = (seq - self.last) & U32_MAX
        if delta == 0:
            self.duplicates += 1
            return False
        if delta >= (1 << 31):
            self.out_of_order += 1
            return False
        self.missing += delta - 1
        self.last = seq
        return True

    def reset_connection(self) -> None:
        self.last = None


def cartesian(angle_cdeg: int, distance_mm: int) -> tuple[float, float]:
    """Return x and y in metres, 0 degrees = +x, 90 degrees = +y."""
    from math import cos, sin, radians
    angle = radians(angle_cdeg / 100.0)
    distance = distance_mm / 1000.0
    return distance * cos(angle), distance * sin(angle)
