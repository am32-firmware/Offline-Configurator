#!/usr/bin/env python3
"""AM32 bootloader/FC protocol simulator used by the CI matrix."""

import argparse
import os
import pty
import select
import struct
import sys
import time
import tty

MCU_FLASH_START = 0x08000000
REQ_MARK = 0x2F
RESP_MARK = 0x2E
ACK_OK = 0x00
ACK_INVALID_CMD = 0x02
ACK_INVALID_CHANNEL = 0x08
ACK_GENERAL_ERROR = 0x0F
GOOD_ACK = 0x30
BAD_ACK = 0xC1
BAD_CRC_ACK = 0xC2
DEVINFO_MAGIC1 = 0x5925E3DA
DEVINFO_MAGIC2 = 0x4EB863D9
DEVINFO_FLASH_OFFSET = 0x0D00

FLASH_VARIANTS = {
    32: dict(size_code=0x1F, eeprom_offset=0x7C00, fw_start=0x1000,
             shift=0, pin_code=0x14, page_size=0x400,
             file_name='AM32_CITEST_F051'),
    64: dict(size_code=0x35, eeprom_offset=0xF800, fw_start=0x1000,
             shift=0, pin_code=0x02, page_size=0x800,
             file_name='AM32_CITEST_L431'),
    128: dict(size_code=0x2B, eeprom_offset=0x1F800, fw_start=0x4000,
              shift=2, pin_code=0x14, page_size=0x800,
              file_name='AM32_CITEST_G431'),
}

NXP128_VARIANT = dict(size_code=0x16, eeprom_offset=0x1E000,
                      fw_start=0x4000, shift=2, pin_code=0x14,
                      page_size=0x2000, file_name='AM32_CITEST_NXP128')


def crc16_direct(data):
    crc = 0
    for byte in data:
        for _ in range(8):
            if (byte ^ crc) & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            byte >>= 1
    return crc


def crc16_xmodem(data):
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF \
                if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def default_settings(version):
    data = bytearray(b'\xFF' * 1024)
    values = {
        0: 1, 1: 4, 2: version, 3: 2, 4: 18, 5: 32, 9: 100,
        10: 45, 12: 10, 20: 1, 21: 1, 22: 1, 23: 2, 24: 24,
        25: 100, 26: 55, 27: 14, 30: 5, 40: 5, 41: 10, 42: 10,
        43: 141, 44: 102, 45: 5,
    }
    for offset, value in values.items():
        data[offset] = value
    data[48:48 + 16] = b'CI_TUNE_PRESERVE'
    data[177] = 37  # extended parameter beyond the 48-byte base config
    return bytes(data)


class EscModel:
    def __init__(self, generation, flash_kb, run_seconds=2.0,
                 nxp128=False, fail_eeprom_from=0):
        variant = NXP128_VARIANT if nxp128 else FLASH_VARIANTS[flash_kb]
        self.generation = generation
        self.flash_size = flash_kb * 1024
        self.eeprom_add = MCU_FLASH_START + variant['eeprom_offset']
        self.app_add = MCU_FLASH_START + variant['fw_start']
        self.shift = variant['shift']
        self.page_size = variant['page_size']
        self.pin_code = variant['pin_code']
        self.size_code = variant['size_code']
        self.version = 3 if generation == 'new' else 2
        self.run_seconds = run_seconds
        self.fail_eeprom_from = fail_eeprom_from
        self.eeprom_write_count = 0
        self.flash = bytearray(b'\xFF' * self.flash_size)
        self.store(variant['eeprom_offset'], default_settings(
            19 if generation == 'new' else 17))
        # v3 128k models a DroneCAN layout: filename is near the application,
        # while the legacy magic filename address still maps to EEPROM-32.
        self.filename_add = self.app_add + 0x400 \
            if generation == 'new' and flash_kb == 128 and not nxp128 \
            else self.eeprom_add - 32
        self.store(self.filename_add - MCU_FLASH_START,
                   variant['file_name'].encode().ljust(32, b'\0'))
        if generation == 'new':
            self.store(DEVINFO_FLASH_OFFSET, self.devinfo_block())
        self.address = 0
        self.continue_address = 0
        self.payload = b''
        self.payload_size = 0
        self.running_until = 0.0

    @property
    def running(self):
        return time.time() < self.running_until

    def store(self, offset, data):
        self.flash[offset:offset + len(data)] = data

    def device_info(self):
        return bytes([ord('4'), ord('7'), ord('1'), self.pin_code,
                      self.size_code, 0x06, 0x06, self.version, GOOD_ACK])

    def devinfo_block(self):
        return struct.pack(
            '<II9sBBHHHH', DEVINFO_MAGIC1, DEVINFO_MAGIC2,
            self.device_info(), 27, self.shift,
            ((self.app_add - MCU_FLASH_START) >> self.shift) & 0xFFFF,
            ((self.filename_add - MCU_FLASH_START) >> self.shift) & 0xFFFF,
            ((self.eeprom_add - MCU_FLASH_START) >> self.shift) & 0xFFFF,
            ((self.eeprom_add - MCU_FLASH_START + 48) >> self.shift) & 0xFFFF)

    def run(self):
        app_offset = self.app_add - MCU_FLASH_START
        stack, entry = struct.unpack_from('<II', self.flash, app_offset)
        if self.flash[self.eeprom_add - MCU_FLASH_START] != 1 or not (
                0x20000000 <= stack <= 0x20010000 and
                self.app_add <= entry <= self.app_add + 256 * 1024):
            self.running_until = 0.0
            return False
        self.running_until = time.time() + self.run_seconds
        self.address = 0
        return True

    def set_address(self, address):
        if address == 0x20:
            self.address = self.eeprom_add
        elif address == 0x21:
            self.address = self.eeprom_add - 32
        elif address == 0x22:
            self.address = self.continue_address
        elif address == 0x23 and self.generation == 'new':
            self.address = MCU_FLASH_START + DEVINFO_FLASH_OFFSET
        elif address < 1024:
            return BAD_ACK
        else:
            self.address = MCU_FLASH_START + (address << self.shift)
        return GOOD_ACK

    def read(self, size):
        if not self.address:
            return None
        offset = self.address - MCU_FLASH_START
        data = bytes(self.flash[offset:offset + size]).ljust(size, b'\xFF')
        self.continue_address = self.address + size
        self.address = 0
        return data

    def program(self):
        if self.address < self.app_add:
            return BAD_ACK
        offset = self.address - MCU_FLASH_START
        if offset + len(self.payload) > self.flash_size:
            return BAD_ACK
        data = bytearray(self.payload)
        if self.eeprom_add <= self.address < self.eeprom_add + 1024:
            self.eeprom_write_count += 1
            if (self.fail_eeprom_from and
                    self.eeprom_write_count >= self.fail_eeprom_from):
                return BAD_ACK
        if self.address == self.eeprom_add:
            if len(data) > 2:
                data[2] = 19 if self.generation == 'new' else 17
            # Real save_flash_nolib implementations erase the complete page
            # when programming its aligned first address.
            page_offset = offset - (offset % self.page_size)
            self.flash[page_offset:page_offset + self.page_size] = \
                b'\xFF' * self.page_size
        self.store(offset, data)
        return GOOD_ACK


class PtyEndpoint:
    def __init__(self):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        self.path = os.ttyname(self.slave)

    def read(self, timeout=0.02):
        try:
            ready, _, _ = select.select([self.master], [], [], timeout)
            return os.read(self.master, 4096) if ready else b''
        except (OSError, ValueError):
            return b''

    def write(self, data):
        os.write(self.master, data)


class DirectServer:
    RUN_IDLE = 0.05

    def __init__(self, esc):
        self.esc = esc
        self.ep = PtyEndpoint()
        self.buffer = b''
        self.last_rx = 0.0
        self.expect_payload = False

    def serve(self):
        while True:
            chunk = self.ep.read()
            now = time.time()
            if self.esc.running:
                if chunk:
                    self.ep.write(chunk)
                self.buffer = b''
                self.expect_payload = False
                continue
            if chunk:
                self.ep.write(chunk)
                self.buffer += chunk
                self.last_rx = now
            self.parse(now)

    def frame_length(self, now):
        command = self.buffer[0]
        if command in (0xFF, 0xFE):
            return 6
        if command in (0x01, 0x02, 0x03, 0xFD):
            return 4
        if command == 0:
            if len(self.buffer) >= 21:
                return 21
            probe = b'\0' * 12 + b'\x0dBLHeli\xF4\x7D'
            if self.buffer == probe[:len(self.buffer)]:
                if len(self.buffer) == 4 and now - self.last_rx > self.RUN_IDLE:
                    return 4
                return None
        return 0

    def parse(self, now):
        while self.buffer and not self.esc.running:
            if self.expect_payload:
                needed = self.esc.payload_size + 2
                if len(self.buffer) < needed:
                    return
                frame, self.buffer = self.buffer[:needed], self.buffer[needed:]
                self.expect_payload = False
                data, received = frame[:-2], struct.unpack('<H', frame[-2:])[0]
                if crc16_direct(data) != received:
                    self.ep.write(bytes([BAD_CRC_ACK]))
                else:
                    self.esc.payload = data
                    self.ep.write(bytes([GOOD_ACK]))
                continue
            length = self.frame_length(now)
            if length is None or len(self.buffer) < length:
                return
            if length == 0:
                self.buffer = b''
                self.ep.write(bytes([BAD_ACK]))
                return
            frame, self.buffer = self.buffer[:length], self.buffer[length:]
            self.handle(frame)

    def valid_crc(self, frame, body_size):
        if crc16_direct(frame[:body_size]) == struct.unpack(
                '<H', frame[body_size:body_size + 2])[0]:
            return True
        self.ep.write(bytes([BAD_CRC_ACK]))
        return False

    def handle(self, frame):
        if len(frame) == 21:
            self.ep.write(self.esc.device_info())
            return
        command = frame[0]
        if command == 0:
            self.esc.run()
        elif command == 0xFF and self.valid_crc(frame, 4):
            self.ep.write(bytes([self.esc.set_address(
                (frame[2] << 8) | frame[3])]))
        elif command == 0xFE:
            if not self.valid_crc(frame, 4):
                return
            self.esc.payload_size = 256 if frame[2] == 1 else frame[3]
            self.expect_payload = True
        elif self.valid_crc(frame, 2):
            if command == 0x01:
                self.ep.write(bytes([self.esc.program()]))
            elif command == 0x02:
                self.ep.write(bytes([GOOD_ACK if self.esc.address >=
                                     self.esc.app_add else BAD_ACK]))
            elif command == 0x03:
                data = self.esc.read(frame[1] or 256)
                if data is None:
                    self.ep.write(bytes([BAD_ACK]))
                else:
                    crc = crc16_direct(data)
                    self.ep.write(data + struct.pack('<H', crc) +
                                  bytes([GOOD_ACK]))
            else:
                self.ep.write(bytes([BAD_ACK]))


class FourWayFC:
    def __init__(self, escs):
        self.escs = escs
        self.selected = 0
        self.ep = PtyEndpoint()
        self.fourway = False
        self.buffer = b''

    def serve(self):
        while True:
            chunk = self.ep.read(0.05)
            if not chunk:
                continue
            self.buffer += chunk
            if self.fourway:
                self.parse_fourway()
            else:
                self.parse_msp()

    def msp_reply(self, command, payload=b''):
        body = bytes([len(payload), command]) + payload
        checksum = 0
        for byte in body:
            checksum ^= byte
        self.ep.write(b'$M>' + body + bytes([checksum]))

    def parse_msp(self):
        while True:
            start = self.buffer.find(b'$M<')
            if start < 0:
                self.buffer = b''
                return
            self.buffer = self.buffer[start:]
            if len(self.buffer) < 6:
                return
            size = self.buffer[3]
            if len(self.buffer) < size + 6:
                return
            command = self.buffer[4]
            payload = self.buffer[5:5 + size]
            self.buffer = self.buffer[6 + size:]
            if command == 245:
                self.msp_reply(command, bytes([len(self.escs)]))
                self.fourway = True
            elif command == 104:
                self.msp_reply(command, struct.pack('<8H', *([1000] * 8)))
            elif command == 1:
                self.msp_reply(command, b'\0\x01.\x00')
            elif command == 2:
                self.msp_reply(command, b'BTFL')
            else:
                self.ep.write(b'$M!' + bytes([0, command, command]))
            if self.fourway:
                self.parse_fourway()
                return

    def reply(self, command, address, params, ack):
        body = bytes([RESP_MARK, command, address >> 8, address & 0xFF,
                      len(params) & 0xFF]) + bytes(params) + bytes([ack])
        return body + struct.pack('>H', crc16_xmodem(body))

    def parse_fourway(self):
        while True:
            start = self.buffer.find(bytes([REQ_MARK]))
            if start < 0:
                self.buffer = b''
                return
            self.buffer = self.buffer[start:]
            if len(self.buffer) < 5:
                return
            size = self.buffer[4] or 256
            total = size + 7
            if len(self.buffer) < total:
                return
            frame, self.buffer = self.buffer[:total], self.buffer[total:]
            if struct.unpack('>H', frame[-2:])[0] == crc16_xmodem(frame[:-2]):
                self.ep.write(self.handle(frame))

    def handle(self, frame):
        command = frame[1]
        address = (frame[2] << 8) | frame[3]
        size = frame[4] or 256
        params = frame[5:5 + size]
        esc = self.escs[self.selected]
        if command == 0x37:
            target = params[0] if params else 0
            if target >= len(self.escs):
                return self.reply(command, 0, [0], ACK_INVALID_CHANNEL)
            self.selected = target
            esc = self.escs[target]
            deadline = time.time() + 3
            while esc.running and time.time() < deadline:
                time.sleep(0.05)
            if esc.running:
                return self.reply(command, 0, [0], ACK_GENERAL_ERROR)
            info = esc.device_info()
            return self.reply(command, 0,
                              [info[5], info[4], info[3], info[6]], ACK_OK)
        if command == 0x35:
            esc.run()
            return self.reply(command, address, [0], ACK_OK)
        if command in (0x3A, 0x3D):
            amount = (params[0] or 256) if params else 256
            if esc.running or esc.set_address(address) != GOOD_ACK:
                return self.reply(command, address, [0], ACK_GENERAL_ERROR)
            data = esc.read(amount)
            return self.reply(command, address, data or [0],
                              ACK_OK if data is not None else ACK_GENERAL_ERROR)
        if command in (0x3B, 0x3E):
            if esc.running or not params or esc.set_address(address) != GOOD_ACK:
                return self.reply(command, address, [0], ACK_GENERAL_ERROR)
            esc.payload = bytes(params)
            ack = ACK_OK if esc.program() == GOOD_ACK else ACK_GENERAL_ERROR
            return self.reply(command, address, [0], ack)
        if command == 0x39:
            ack = ACK_OK if esc.set_address(address) == GOOD_ACK and \
                esc.address >= esc.app_add else ACK_GENERAL_ERROR
            return self.reply(command, address, params[:1] or [0], ack)
        if command in (0x30, 0x31, 0x32, 0x33, 0x3C, 0x3F):
            return self.reply(command, address, [0], ACK_OK)
        if command == 0x34:
            self.fourway = False
            return self.reply(command, address, [0], ACK_OK)
        return self.reply(command, address, [0], ACK_INVALID_CMD)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['direct', '4way'], required=True)
    parser.add_argument('--generation', choices=['old', 'new'], required=True)
    parser.add_argument('--flash-size', choices=[32, 64, 128], type=int,
                        required=True)
    parser.add_argument('--esc-count', choices=range(1, 9), type=int, default=4)
    parser.add_argument('--nxp128', action='store_true')
    parser.add_argument('--mixed-escs', action='store_true')
    parser.add_argument('--fail-eeprom-from', type=int, default=0)
    args = parser.parse_args()
    if args.mixed_escs and args.mode != '4way':
        parser.error('--mixed-escs requires --mode 4way')
    if args.mode == 'direct':
        server = DirectServer(EscModel(
            args.generation, args.flash_size, nxp128=args.nxp128,
            fail_eeprom_from=args.fail_eeprom_from))
    elif args.mixed_escs:
        server = FourWayFC([
            EscModel('new', 128),
            EscModel('old', 64),
            EscModel('new', 128, nxp128=True),
            EscModel('old', 32),
        ])
    else:
        server = FourWayFC([EscModel(
            args.generation, args.flash_size, nxp128=args.nxp128,
            fail_eeprom_from=args.fail_eeprom_from)
                            for _ in range(args.esc_count)])
    print(server.ep.path, flush=True)
    server.serve()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
