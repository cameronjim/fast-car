"""Writes app-icon.png (a checkered flag on a blue tile), the source for `tauri icon`.

Standard library only. Run from this directory, then:  npx tauri icon app-icon.png
"""

import struct
import zlib

N = 1024
RADIUS = 180


def pixel(x: int, y: int) -> bytes:
    cx = min(max(x, RADIUS), N - 1 - RADIUS)
    cy = min(max(y, RADIUS), N - 1 - RADIUS)
    if (x - cx) ** 2 + (y - cy) ** 2 > RADIUS**2:
        return bytes([0, 0, 0, 0])
    gx, gy = (x - 192) // 128, (y - 256) // 128
    if 192 <= x < 832 and 256 <= y < 768 and 0 <= gx < 5 and 0 <= gy < 4:
        rgb = (255, 255, 255) if (gx + gy) % 2 == 0 else (16, 19, 23)
    else:
        rgb = (10, 108, 255)
    return bytes(rgb) + b"\xff"


def chunk(tag: bytes, data: bytes) -> bytes:
    body = struct.pack(">I", len(data)) + tag + data
    return body + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)


def main() -> None:
    raw = b"".join(b"\x00" + b"".join(pixel(x, y) for x in range(N)) for y in range(N))
    png = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", N, N, 8, 6, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 9))
        + chunk(b"IEND", b"")
    )
    with open("app-icon.png", "wb") as f:
        f.write(png)


if __name__ == "__main__":
    main()
