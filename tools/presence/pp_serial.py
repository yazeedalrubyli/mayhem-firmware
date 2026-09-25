#!/usr/bin/env python3
"""Talk to the PortaPack Mayhem USB serial console.

Usage:
  pp_serial.py info
  pp_serial.py cmd "<console command>"
  pp_serial.py ls /APPS
  pp_serial.py upload <local-file> </SD/PATH>
  pp_serial.py verify <local-file> </SD/PATH>
  pp_serial.py flash </FIRMWARE/x.bin>
  pp_serial.py sync-apps <dir-with-ppma>
  pp_serial.py appstart <callname>
  pp_serial.py screenshot <out.png>
  pp_serial.py button <n>           (1 up 2 down 3 left 4 right 5 select 6 encoder+ 7 encoder-)
  pp_serial.py touch <x> <y>

Requires pyserial and, for screenshot, Pillow. Fails loudly on any unexpected reply.

The device's `crc32` is CRC-32/BZIP2 (poly 0x04C11DB7, init/xor 0xFFFFFFFF, no bit
reflection), not the zlib variant; `verify` computes the same.

Uploads use `fwb` in 4096-byte chunks written as 64-byte USB packets with a 2 ms
pause (the pacing hackrf.app uses on macOS). A single 16 KiB write wedged the
v2.4.0 USB stack on the H4M (control transfers failed with -71 until a hardware
reset), so the port is opened with a write timeout and the tool never blocks.
"""
import os
import re
import sys
import time

import serial

PORT = os.environ.get("PP_PORT", "/dev/ttyACM0")
CHUNK = 4096        # bytes per fwb command
PIECE = 64          # bytes per USB write
PIECE_PAUSE = 0.002  # seconds between USB writes


_CRC_TABLE = []
for _i in range(256):
    _c = _i << 24
    for _ in range(8):
        _c = ((_c << 1) ^ 0x04C11DB7) & 0xFFFFFFFF if _c & 0x80000000 else (_c << 1) & 0xFFFFFFFF
    _CRC_TABLE.append(_c)


def crc32_bzip2(data):
    """CRC-32/BZIP2 as computed by the Mayhem `crc32` console command."""
    crc = 0xFFFFFFFF
    for b in data:
        crc = ((crc << 8) & 0xFFFFFFFF) ^ _CRC_TABLE[((crc >> 24) ^ b) & 0xFF]
    return crc ^ 0xFFFFFFFF


class Console:
    def __init__(self, port=PORT):
        self.s = serial.Serial(port, 115200, timeout=0.2, write_timeout=5)
        self.s.reset_input_buffer()
        self.s.write(b"\r")
        self._read_until_prompt(2.0)

    def close(self):
        self.s.close()

    def _read_until_prompt(self, timeout):
        buf = b""
        t0 = time.time()
        while time.time() - t0 < timeout:
            waiting = self.s.in_waiting
            chunk = self.s.read(waiting if waiting else 1)
            if chunk:
                buf += chunk
                if buf.rstrip().endswith(b"ch>"):
                    break
        return buf

    @staticmethod
    def _clean(raw):
        txt = raw.decode(errors="replace")
        txt = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", txt).replace("\r", "")
        lines = [l for l in txt.split("\n")]
        # drop the echoed command line and the trailing prompt
        if lines and lines[-1].strip() == "ch>":
            lines = lines[:-1]
        if lines:
            lines = lines[1:]
        return "\n".join(lines).strip()

    def cmd(self, text, timeout=5.0, expect_ok=False):
        self.s.reset_input_buffer()
        self.s.write((text + "\r").encode())
        out = self._clean(self._read_until_prompt(timeout))
        if expect_ok and not out.rstrip().endswith("ok"):
            raise SystemExit(f"command failed: {text!r} -> {out!r}")
        return out

    def write_paced(self, data):
        for i in range(0, len(data), PIECE):
            self.s.write(data[i:i + PIECE])
            time.sleep(PIECE_PAUSE)

    def upload(self, local, remote):
        data = open(local, "rb").read()
        self.cmd("fclose")  # harmless if nothing is open
        self.cmd(f"fopen {remote}", expect_ok=True)
        self.cmd("fseek 0", expect_ok=True)
        sent = 0
        try:
            while sent < len(data):
                part = data[sent:sent + CHUNK]
                self.s.reset_input_buffer()
                self.s.write(f"fwb {len(part)}\r".encode())
                head = b""
                t0 = time.time()
                while b"send " not in head and time.time() - t0 < 5:
                    waiting = self.s.in_waiting
                    head += self.s.read(waiting if waiting else 1)
                if b"send " not in head:
                    raise SystemExit(f"fwb handshake failed after {sent} bytes: {head!r}")
                self.write_paced(part)
                tail = self._read_until_prompt(30.0)
                if b"ok" not in tail:
                    raise SystemExit(f"fwb chunk not acknowledged after {sent} bytes: {tail!r}")
                sent += len(part)
                print(f"\r  {remote}: {sent}/{len(data)}", end="", flush=True)
        finally:
            self.cmd("fclose", expect_ok=True)
        print()
        self.verify(local, remote)

    def verify(self, local, remote):
        data = open(local, "rb").read()
        size_out = self.cmd(f"filesize {remote}")
        size = int(size_out.split()[0])
        if size != len(data):
            raise SystemExit(f"size mismatch for {remote}: device {size}, local {len(data)}")
        crc_out = self.cmd(f"crc32 {remote}", timeout=120.0)
        m = re.search(r"0x([0-9A-Fa-f]{8})", crc_out)
        if not m:
            raise SystemExit(f"no crc in reply: {crc_out!r}")
        device = int(m.group(1), 16)
        local_crc = crc32_bzip2(data)
        if device != local_crc:
            raise SystemExit(f"crc mismatch for {remote}: device {device:08x}, local {local_crc:08x}")
        print(f"  verified {remote}: {size} bytes, crc32 {device:08x}")

    def screenshot(self, out_png):
        from PIL import Image
        self.s.reset_input_buffer()
        self.s.write(b"screenframeshort\r")
        raw = self._read_until_prompt(20.0)
        txt = raw.decode(errors="replace").replace("\r", "")
        rows = [l for l in txt.split("\n") if len(l) == 240]
        if len(rows) != 320:
            raise SystemExit(f"screenframeshort returned {len(rows)} rows of 240 chars, expected 320")
        img = Image.new("RGB", (240, 320))
        px = img.load()
        for y, row in enumerate(rows):
            for x, ch in enumerate(row):
                v = ord(ch) - 32
                px[x, y] = (((v >> 4) & 3) * 85, ((v >> 2) & 3) * 85, (v & 3) * 85)
        img.save(out_png)
        print(f"  screenshot saved to {out_png}")


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    op = argv[1]
    c = Console()
    try:
        if op == "info":
            print(c.cmd("info"))
        elif op == "cmd":
            print(c.cmd(" ".join(argv[2:]), timeout=30.0))
        elif op == "ls":
            print(c.cmd(f"ls {argv[2]}"))
        elif op == "upload":
            c.upload(argv[2], argv[3])
        elif op == "verify":
            c.verify(argv[2], argv[3])
        elif op == "flash":
            print(c.cmd(f"flash {argv[2]}", timeout=10.0))
        elif op == "sync-apps":
            listing = c.cmd("ls /APPS")
            for line in listing.split("\n"):
                name = line.strip()
                if name.lower().endswith(".ppma"):
                    c.cmd(f"unlink /APPS/{name}", expect_ok=True)
                    print(f"  removed /APPS/{name}")
            for name in sorted(os.listdir(argv[2])):
                if name.lower().endswith(".ppma"):
                    c.upload(os.path.join(argv[2], name), f"/APPS/{name}")
        elif op == "appstart":
            print(c.cmd(f"appstart {argv[2]}", timeout=10.0))
        elif op == "screenshot":
            c.screenshot(argv[2])
        elif op == "button":
            print(c.cmd(f"button {argv[2]}"))
        elif op == "touch":
            print(c.cmd(f"touch {argv[2]} {argv[3]}"))
        else:
            print(__doc__)
            return 2
    finally:
        c.close()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
