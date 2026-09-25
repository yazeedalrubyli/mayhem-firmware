"""Hermetic tests for pp_serial.py: a scripted transport plays the device's real replies.

Run: python3 tools/presence/test_pp_serial.py   (or: python3 -m pytest tools/presence/test_pp_serial.py)
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pp_serial  # noqa: E402


class FakeSerial:
    """Each write() appends the next scripted reply to the read buffer."""

    def __init__(self, replies):
        self.replies = list(replies)
        self.buf = b""
        self.written = []

    @property
    def in_waiting(self):
        return len(self.buf)

    def read(self, n):
        chunk, self.buf = self.buf[:n], self.buf[n:]
        return chunk

    def write(self, data):
        self.written.append(data)
        if self.replies:
            self.buf += self.replies.pop(0)

    def reset_input_buffer(self):
        self.buf = b""

    def close(self):
        pass


PROMPT = b"ch>"


def console(*replies):
    return pp_serial.Console(transport=FakeSerial([PROMPT, *replies]))


def test_flash_accepts_the_promptless_flashing_started_reply():
    # The device prints "Flashing started" and reboots into the bootloader: no prompt ever follows.
    c = console(b"flash /FIRMWARE/x.bin\r\nFlashing started\r\n")
    assert c.flash("/FIRMWARE/x.bin", timeout=0.2) == "Flashing started"


def test_flash_fails_loudly_when_the_file_is_missing():
    c = console(b"flash /FIRMWARE/typo.bin\r\nfile not found.\r\n" + PROMPT)
    try:
        c.flash("/FIRMWARE/typo.bin", timeout=0.2)
    except pp_serial.ConsoleError as e:
        assert "file not found." in e.reply
    else:
        raise AssertionError("flash of a missing file did not raise")


def test_appstart_fails_loudly_on_error():
    c = console(b"appstart nosuchapp\r\nerror\r\n" + PROMPT)
    try:
        c.appstart("nosuchapp", timeout=0.2)
    except pp_serial.ConsoleError as e:
        assert "error" in e.reply
    else:
        raise AssertionError("appstart of a missing app did not raise")


def test_appstart_returns_ok():
    c = console(b"appstart presence\r\nok\r\n" + PROMPT)
    assert c.appstart("presence", timeout=0.2) == "ok"


if __name__ == "__main__":
    failed = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except Exception as e:  # noqa: BLE001
                failed += 1
                print(f"FAIL {name}: {type(e).__name__}: {e}")
    sys.exit(1 if failed else 0)
