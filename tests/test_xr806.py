"""Offline XR806 BROM tests; no hardware and no external dependencies required."""

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch


class Conn:
    def __init__(self):
        self.port = "FAKE"
        self.baudrate = None
        self.link_baudrate = None
        self.linked = False

    def fill_baudrate(self, value):
        self.baudrate = self.baudrate or value
        self.link_baudrate = self.link_baudrate or value


class Features:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class Progress:
    def __init__(self):
        self.current = 0

    def on_update(self, value):
        self.current += value


class Memory:
    FLASH = "flash"


class FakeSerial:
    def __init__(self, *args, **kwargs):
        self.buffer = bytearray()
        self.closed = False
        self.ops = []
        self.flash = bytes(range(256)) * 8192

    def write(self, packet):
        self.ops.append(bytes(packet))
        if packet == b"\x55":
            self.buffer += b"OK"
        else:
            assert packet[:4] == b"BROM"
            command = packet[12]
            if command == 0x18:
                payload = bytes.fromhex("EF4015")
            elif command == 0x1A:
                sector = int.from_bytes(packet[13:17], "big")
                count = int.from_bytes(packet[17:21], "big")
                payload = self.flash[sector * 512:(sector + count) * 512]
            else:
                raise AssertionError("Unexpected opcode")
            self.buffer += (
                b"BROM" + bytes([0, 4]) + b"\x00\x00"
                + len(payload).to_bytes(4, "big") + payload
            )
        return len(packet)

    def flush(self):
        pass

    def read(self, n):
        result = self.buffer[:n]
        del self.buffer[:n]
        return bytes(result)

    def reset_input_buffer(self):
        self.buffer.clear()

    def close(self):
        self.closed = True


# Import XR806 reader in isolation; the tests need neither pyserial nor
# a fully installed LibreTiny/ltchiptool environment.
for name in (
    "ltchiptool", "ltchiptool.soc", "ltchiptool.soc.interface",
    "ltchiptool.util", "ltchiptool.util.flash", "ltchiptool.util.streams",
    "serial",
):
    sys.modules[name] = types.ModuleType(name)
sys.modules["ltchiptool"].Family = type("Family", (), {})
sys.modules["ltchiptool.soc.interface"].SocInterface = type("SocInterface", (), {})
for key, val in (
    ("FlashConnection", Conn), ("FlashFeatures", Features),
    ("FlashMemoryType", Memory),
):
    setattr(sys.modules["ltchiptool.util.flash"], key, val)
sys.modules["ltchiptool.util.streams"].ProgressCallback = Progress
sys.modules["serial"].Serial = FakeSerial

source = Path(__file__).resolve().parents[1] / "ltchiptool" / "soc" / "xr806.py"
spec = importlib.util.spec_from_file_location("xr806_under_test", str(source))
xr806 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(xr806)


class XR806ReadOnlyTests(unittest.TestCase):
    def new(self):
        reader = xr806.XR806Main(None)
        reader.flash_set_connection(Conn())
        return reader

    def test_flash_id_frame(self):
        self.assertEqual(
            xr806._packet(0x18), bytes.fromhex("42524f4d040060510000000118")
        )

    def test_connect_and_detect_capacity(self):
        reader = self.new()
        reader.flash_connect()
        self.assertEqual(reader.flash_get_size(), 2 * 1024 * 1024)
        self.assertIn(("BROM version", "4"), reader.flash_get_chip_info())
        self.assertFalse(reader.flash_get_features().can_write)

    def test_unaligned_verified_read(self):
        reader = self.new()
        callback = Progress()
        data = b"".join(reader.flash_read_raw(500, 1200, callback=callback))
        self.assertEqual(data, FakeSerial().flash[500:1700])
        self.assertEqual(callback.current, 1200)

    def test_missing_sync_closes_port(self):
        class SilentSerial(FakeSerial):
            def write(self, packet):
                if packet == b"\x55":
                    return 1
                return super().write(packet)
        reader = self.new()
        with patch.object(xr806.serial, "Serial", SilentSerial):
            with self.assertRaises(TimeoutError):
                reader.flash_connect()
            self.assertIsNone(reader._port)

    def test_oversized_brom_response_rejected(self):
        reader = self.new()
        reader.flash_connect()
        reader._port.buffer = bytearray(
            b"BROM\x00\x04\x00\x00\xff\xff\xff\xff"
        )
        with self.assertRaises(ValueError):
            reader._response()

    def test_write_disabled(self):
        with self.assertRaises(NotImplementedError):
            self.new().flash_write_raw()


if __name__ == "__main__":
    unittest.main()
