#!/usr/bin/env python3
"""Run the CLI against every transport/generation/flash-size combination."""

import argparse
import os
import subprocess
import sys
import tempfile
import time
import struct

CI_DIR = os.path.dirname(os.path.abspath(__file__))


def ihex_line(address, record_type, data):
    body = bytes([len(data), address >> 8, address & 0xFF, record_type]) + data
    return ':' + (body + bytes([-sum(body) & 0xFF])).hex().upper()


def make_hex(path, flash_kb):
    start = 0x4000 if flash_kb == 128 else 0x1000
    lines = [ihex_line(0, 4, b'\x08\x00')]
    image = bytearray((13 + index * 7) & 0xFF for index in range(4096))
    struct.pack_into('<II', image, 0, 0x20001000,
                     0x08000000 + start + 0x101)
    if flash_kb == 128:
        # DroneCAN's linker places .file_name 0x400 bytes after the vector
        # region, rather than before EEPROM.
        name = b'AM32_CITEST_G431_FLASHED'.ljust(32, b'\0')
        image[0x400:0x420] = name
    for offset in range(0, len(image), 16):
        lines.append(ihex_line(start + offset, 0, image[offset:offset + 16]))
    lines.append(ihex_line(0, 1, b''))
    with open(path, 'w', encoding='ascii') as output:
        output.write('\n'.join(lines) + '\n')


def run_cell(cli, mode, generation, flash_kb, firmware, simulator_args=None):
    sim_command = [sys.executable, os.path.join(CI_DIR, 'bootloader_sim.py'),
                   '--mode', mode, '--generation', generation,
                   '--flash-size', str(flash_kb)]
    if mode == '4way':
        sim_command += ['--esc-count', '4']
    if simulator_args:
        sim_command += simulator_args
    simulator = subprocess.Popen(sim_command, stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, text=True)
    try:
        tty_path = simulator.stdout.readline().strip()
        if not tty_path.startswith('/'):
            return 3, 'simulator did not provide a tty path'
        suite = 'direct-suite' if mode == 'direct' else 'fourway-suite'
        try:
            result = subprocess.run(
                [cli, suite, firmware, tty_path],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, timeout=120)
            return result.returncode, result.stdout
        except subprocess.TimeoutExpired as error:
            output = error.stdout or ''
            if isinstance(output, bytes):
                output = output.decode(errors='replace')
            return 2, output + '\n[matrix] suite timed out'
    finally:
        simulator.terminate()
        simulator.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cli', default=os.path.join(
        os.path.dirname(CI_DIR), 'SerialPortConnector_CLI'))
    args = parser.parse_args()
    cli = os.path.abspath(args.cli)
    if not os.path.isfile(cli):
        parser.error('CLI executable not found: ' + cli)

    failed = False
    results = []
    with tempfile.TemporaryDirectory(prefix='offline-config-matrix-') as temp:
        firmware = {}
        for flash_kb in (32, 64, 128):
            firmware[flash_kb] = os.path.join(temp, 'fw_%uk.hex' % flash_kb)
            make_hex(firmware[flash_kb], flash_kb)

        for mode in ('direct', '4way'):
            for generation in ('old', 'new'):
                for flash_kb in (32, 64, 128):
                    expected_refusal = generation == 'old' and flash_kb == 128
                    label = '%-6s %-3s %4uk' % (mode, generation, flash_kb)
                    print('=== %s ===' % label, flush=True)
                    started = time.time()
                    rc, output = run_cell(cli, mode, generation, flash_kb,
                                          firmware[flash_kb])
                    elapsed = time.time() - started
                    if expected_refusal:
                        phrase = ('need a v3 bootloader' if mode == 'direct'
                                  else 'require v3 devinfo')
                        ok = rc == 5 and phrase in output
                        detail = 'refused as designed' if ok else 'bad refusal'
                    else:
                        passed = ('DIRECT SUITE PASSED' if mode == 'direct'
                                  else 'FOURWAY SUITE PASSED')
                        found_all = mode != '4way' or 'ESC 4 discovered' in output
                        ok = rc == 0 and passed in output and found_all
                        detail = 'suite passed' if ok else 'exit %d' % rc
                    if not ok:
                        failed = True
                        print(output)
                    print('--- %s: %s (%.1fs)' %
                          (label, 'OK' if ok else 'FAIL', elapsed), flush=True)
                    results.append((label, ok, detail, elapsed))

        # NXP 128k reports the distinct 0x16 flash code while retaining the
        # shifted-address v3 layout. Cover it over both transports.
        for mode in ('direct', '4way'):
            label = '%-6s nxp %4uk' % (mode, 128)
            print('=== %s ===' % label, flush=True)
            started = time.time()
            rc, output = run_cell(cli, mode, 'new', 128, firmware[128],
                                  ['--nxp128'])
            elapsed = time.time() - started
            passed = ('DIRECT SUITE PASSED' if mode == 'direct'
                      else 'FOURWAY SUITE PASSED')
            ok = rc == 0 and passed in output
            if not ok:
                failed = True
                print(output)
            detail = 'suite passed' if ok else 'exit %d' % rc
            print('--- %s: %s (%.1fs)' %
                  (label, 'OK' if ok else 'FAIL', elapsed), flush=True)
            results.append((label, ok, detail, elapsed))

        # Reuse one FourWayIF while switching across heterogeneous ESCs. The
        # old-protocol channels must not inherit v3 metadata from their peers.
        label = '4way  mixed-esc'
        print('=== %s ===' % label, flush=True)
        started = time.time()
        rc, output = run_cell(cli, '4way', 'new', 128, firmware[128],
                              ['--mixed-escs'])
        elapsed = time.time() - started
        ok = (rc == 0 and 'FOURWAY SUITE PASSED' in output and
              'ESC 2 discovered (v3=0 layout_known=1)' in output and
              'ESC 4 discovered (v3=0 layout_known=1)' in output)
        if not ok:
            failed = True
            print(output)
        detail = 'state isolated' if ok else 'stale device state'
        print('--- %s: %s (%.1fs)' %
              (label, 'OK' if ok else 'FAIL', elapsed), flush=True)
        results.append((label, ok, detail, elapsed))

        # The settings-preservation write consumes four 256-byte writes. Force
        # the first pre- and post-flash EEPROM chunks to fail and ensure the
        # CLI returns an error instead of resetting/printing success.
        for fail_from, phrase, stage in (
                (5, 'pre-flash EEPROM safety write failed', 'pre-fail'),
                (9, 'post-flash EEPROM write failed; ESC not reset',
                 'post-fail')):
            label = 'direct %-9s' % stage
            print('=== %s ===' % label, flush=True)
            started = time.time()
            rc, output = run_cell(
                cli, 'direct', 'new', 64, firmware[64],
                ['--fail-eeprom-from', str(fail_from)])
            elapsed = time.time() - started
            ok = rc == 4 and phrase in output and 'FLASH SUCCESS' not in output
            if not ok:
                failed = True
                print(output)
            detail = 'failed closed' if ok else 'bad failure handling'
            print('--- %s: %s (%.1fs)' %
                  (label, 'OK' if ok else 'FAIL', elapsed), flush=True)
            results.append((label, ok, detail, elapsed))

    print('\n%-18s %-6s %s' % ('cell', 'result', 'detail'))
    for label, ok, detail, elapsed in results:
        print('%-18s %-6s %s (%.1fs)' %
              (label, 'OK' if ok else 'FAIL', detail, elapsed))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
