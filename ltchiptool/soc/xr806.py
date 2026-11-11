"""XR806 UART boot-ROM reader for ltchiptool (no flash writes).

This implementation deliberately supports only the documented XR806 BROM
read path. Writing, erasing, firmware building and UF2 are out of scope.
"""

import logging
import time
from typing import IO, Generator, List, Tuple, Union

import serial

from ltchiptool import Family
from ltchiptool.soc.interface import SocInterface
from ltchiptool.util.flash import FlashConnection, FlashFeatures, FlashMemoryType
from ltchiptool.util.streams import ProgressCallback


_BROM_MAGIC = b"BROM"
_READ_SECTOR = 512
_MAX_SECTORS = 8
_ROM_BAUD = 115200


def _checksum(data: bytes) -> int:
    """XR BROM host checksum: one's-complement sum of LE 16-bit words."""
    total = sum(data[i] | (data[i + 1] << 8) for i in range(0, len(data) - 1, 2))
    if len(data) & 1:
        total += data[-1]
    return (~total) & 0xFFFF


def _packet(command: int, *values: int) -> bytes:
    """Build a BROM request; logical checksum uses LE words, wire uses BE."""
    wire_data = b"".join(int(v).to_bytes(4, "big") for v in values)
    check_data = b"".join(int(v).to_bytes(4, "little") for v in values)
    n = 1 + len(wire_data)
    pre = _BROM_MAGIC + b"\x04\x00\x00\x00" + n.to_bytes(4, "little")
    check = _checksum(pre + bytes([command]) + check_data)
    return (
        _BROM_MAGIC + b"\x04\x00" + check.to_bytes(2, "big")
        + n.to_bytes(4, "big") + bytes([command]) + wire_data
    )


class XR806Main(SocInterface):
    def __init__(self, family: Family):
        self.family = family
        self.conn = None
        self._port = None
        self._version = None
        self._flash_id = None
        self._flash_size = None
        self._link_timeout = 1.5
        self._response_timeout = 5.0

    def hello(self):
        logging.info("XR806 UART BROM (read-only)")

    @property
    def ota_type(self):
        return None

    def detect_file_type(self, file: IO[bytes], length: int):
        # No XR806 firmware layout is approved for automatic writing.
        return None

    def flash_get_features(self) -> FlashFeatures:
        return FlashFeatures(
            can_write=False, can_read=True, can_read_rom=False,
            can_read_efuse=False, can_read_info=True,
        )

    def flash_get_guide(self) -> List[Union[str, list]]:
        return [
            "XR806 download mode: connect 3.3V UART0 TX/RX crosswise and GND.",
            "Hold PB02 to GND while applying power/resetting, then release it.",
            "Use a stable 3.3V supply; never attach a live mains-powered device.",
            "Only 115200 baud and flash READ are supported in this build.",
        ]

    def flash_set_connection(self, connection: FlashConnection) -> None:
        self.flash_disconnect()
        connection.fill_baudrate(_ROM_BAUD)
        if connection.baudrate != _ROM_BAUD or connection.link_baudrate != _ROM_BAUD:
            raise ValueError("XR806 prototype currently supports 115200 baud only")
        self.conn = connection

    def flash_build_protocol(self, force: bool = False) -> None:
        if self._port and not force:
            return
        self.flash_disconnect()
        self._port = serial.Serial(
            port=self.conn.port, baudrate=_ROM_BAUD,
            timeout=0.2, write_timeout=3,
        )

    def flash_change_timeout(self, timeout: float = 0.0, link_timeout: float = 0.0):
        if timeout:
            self._response_timeout = max(1.0, timeout)
        if link_timeout:
            self._link_timeout = max(0.2, link_timeout)

    def flash_disconnect(self) -> None:
        if self._port:
            self._port.close()
        self._port = None
        self._version = None
        self._flash_id = None
        self._flash_size = None
        if self.conn:
            self.conn.linked = False

    def _read_exact(self, count: int, timeout: float) -> bytes:
        limit = time.monotonic() + timeout
        data = bytearray()
        while len(data) < count and time.monotonic() < limit:
            part = self._port.read(count - len(data))
            if part:
                data.extend(part)
        if len(data) != count:
            raise TimeoutError("XR806 BROM: incomplete response ({}/{})".format(len(data), count))
        return bytes(data)

    def _response(self, max_size: int = 4096) -> bytes:
        header = self._read_exact(12, self._response_timeout)
        if header[:4] != _BROM_MAGIC:
            raise ValueError("XR806: invalid BROM response magic")
        version = header[5]
        if version not in (3, 4):
            raise NotImplementedError("Unsupported XR806 BROM version: {}".format(version))
        n = int.from_bytes(header[8:12], "big")
        if n > max_size:
            raise ValueError("XR806: oversized BROM payload: {}".format(n))
        payload = self._read_exact(n, self._response_timeout) if n else b""
        if header[4] & 1:
            raise IOError("XR806: BROM error response")
        self._version = version
        return payload

    def _command(self, command: int, *values: int) -> bytes:
        self._port.write(_packet(command, *values))
        self._port.flush()
        return self._response()

    def flash_connect(self) -> None:
        if self.conn and self.conn.linked:
            return
        self.flash_build_protocol()
        try:
            for _ in range(10):
                self._port.reset_input_buffer()
                self._port.write(b"\x55")
                self._port.flush()
                try:
                    reply = self._read_exact(2, min(1.5, self._link_timeout))
                except TimeoutError:
                    continue
                if reply == b"OK":
                    break
            else:
                raise TimeoutError("XR806: no BROM sync; check PB02 and UART0")

            flash_id = self._command(0x18)
            if len(flash_id) < 3:
                raise IOError("XR806: incomplete JEDEC flash ID")
            capacity = flash_id[2]
            if not (18 <= capacity <= 25):
                raise ValueError("XR806: unknown JEDEC flash capacity byte")
            self._flash_id = flash_id[:3]
            self._flash_size = 1 << capacity
            self.conn.linked = True
        except Exception:
            self.flash_disconnect()
            raise

    def flash_get_chip_info(self) -> List[Tuple[str, str]]:
        self.flash_connect()
        return [
            ("Chip", "XR806"),
            ("BROM version", str(self._version)),
            ("JEDEC Flash ID", self._flash_id.hex().upper()),
            ("Flash capacity", "{} bytes".format(self._flash_size)),
        ]

    def flash_get_chip_info_string(self) -> str:
        self.flash_connect()
        return "XR806 BROM{} / {} KiB flash".format(
            self._version, self._flash_size // 1024
        )

    def flash_get_size(self, memory=FlashMemoryType.FLASH) -> int:
        if memory != FlashMemoryType.FLASH:
            raise NotImplementedError("XR806: requested memory region is not readable")
        self.flash_connect()
        return self._flash_size

    def _read_sectors(self, sector: int, count: int) -> bytes:
        payload = self._command(0x1A, sector, count)
        required = count * _READ_SECTOR
        if len(payload) != required:
            raise IOError("XR806: read length mismatch: {} != {}".format(len(payload), required))
        return payload

    def flash_read_raw(
        self, offset: int, length: int, verify: bool = True,
        memory=FlashMemoryType.FLASH,
        callback: ProgressCallback = ProgressCallback(),
    ) -> Generator[bytes, None, None]:
        size = self.flash_get_size(memory)
        if offset < 0 or length < 0 or offset > size or length > size - offset:
            raise ValueError("XR806: flash read outside chip capacity")
        start_sector = offset // _READ_SECTOR
        end_sector = (offset + length + _READ_SECTOR - 1) // _READ_SECTOR
        current = start_sector
        while current < end_sector:
            count = min(_MAX_SECTORS, end_sector - current)
            chunk = self._read_sectors(current, count)
            if verify and chunk != self._read_sectors(current, count):
                raise IOError("XR806: repeated flash read mismatch at sector {}".format(current))
            base = current * _READ_SECTOR
            first = max(offset - base, 0)
            last = min(offset + length - base, len(chunk))
            piece = chunk[first:last]
            callback.on_update(len(piece))
            yield piece
            current += count

    def flash_write_raw(self, *args, **kwargs) -> None:
        raise NotImplementedError("XR806 write is disabled pending hardware tests")

    def flash_write_uf2(self, *args, **kwargs) -> None:
        raise NotImplementedError("XR806 firmware upload is not supported")
