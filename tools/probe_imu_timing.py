#!/usr/bin/env python3
"""Receive-only FDILink timing with an optional isolated estimator consumer.

No ROS node is started. Device/host clock offsets and sensor filtering delay
are not identifiable from these data. Motor ports are never opened in phase 3.
"""
import argparse
import binascii
from collections import Counter
import csv
import fcntl
import json
import math
import mmap
import os
from pathlib import Path
import select
import signal
import struct
import subprocess
import termios
import time
import unittest

from analyze_timing_probe import analyze, stats

LAYOUT = struct.Struct('<QQQqqqqqq10d')  # matches ImuSnapshot in the C++ consumer
FRAME_FIELDS = ['frame', 'type', 'seq', 'device_us', 'first_read_start_ns',
                'first_read_end_ns', 'read_start_ns', 'read_end_ns', 'parsed_ns',
                'first_batch', 'last_batch', 'relative_host_device_us']


def crc8(data):
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8c if value & 1 else 0)
    return value


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.origins = []
        self.frames = []
        self.errors = Counter()

    def drop(self, n):
        del self.buffer[:n]
        del self.origins[:n]
        self.errors['discarded_bytes'] += n

    def feed(self, data, batch, first, last):
        self.buffer.extend(data)
        self.origins.extend([(batch, first, last)] * len(data))
        output = []
        while len(self.buffer) >= 7:
            if self.buffer[0] != 0xfc:
                self.drop(1)
                continue
            if crc8(self.buffer[:4]) != self.buffer[4]:
                self.errors['header_crc8'] += 1
                self.drop(1)
                continue
            size = self.buffer[2] + 8
            if len(self.buffer) < size:
                break
            frame = bytes(self.buffer[:size])
            payload = frame[7:-1]
            if frame[-1] != 0xfd or binascii.crc_hqx(payload, 0) != (frame[5] << 8 | frame[6]):
                self.errors['trailer_or_payload_crc16'] += 1
                self.drop(1)
                continue
            kind = frame[1]
            values = None
            layout = {0x40: '<12fq', 0x41: '<10fq'}.get(kind)
            if layout:
                if len(payload) == struct.calcsize(layout):
                    values = struct.unpack(layout, payload)
                else:
                    self.errors['invalid_known_payload_length'] += 1
            origin, end = self.origins[0], self.origins[size - 1]
            row = dict(zip(FRAME_FIELDS, [len(self.frames) + 1, hex(kind), frame[3],
                values[-1] if values else '', origin[1], origin[2], end[1], end[2],
                time.monotonic_ns(), origin[0], end[0], '']))
            self.frames.append(row)
            output.append((kind, row, values))
            del self.buffer[:size]
            del self.origins[:size]
        return output


def write_csv(path, rows, fields):
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def capture(args):
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if (output / 'summary.json').exists() or (output / 'frames.csv').exists():
        raise RuntimeError('refusing to overwrite existing capture')
    owner = subprocess.run(['fuser', args.port], capture_output=True, text=True)
    if owner.returncode != 1 or owner.stdout.strip() or owner.stderr.strip():
        raise RuntimeError('IMU port occupied or ownership check failed')
    parser = Parser()
    raw = bytearray()
    reads = []
    latest = {}
    fd = None
    original = None
    child = None
    child_log = None
    error = None
    cleanup_errors = []
    snapshot_file = (output / 'snapshot.bin').open('w+b')
    snapshot_file.truncate(LAYOUT.size)
    shared = mmap.mmap(snapshot_file.fileno(), LAYOUT.size)
    snapshot_version = 0
    start = time.monotonic_ns()
    deadline = start + int((args.duration + (40 if args.consumer_binary else 0)) * 1e9)
    try:
        fd = os.open(args.port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        original = termios.tcgetattr(fd)
        config = termios.tcgetattr(fd)
        config[0] = config[1] = config[3] = 0
        config[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        config[4] = config[5] = termios.B921600
        config[6][termios.VMIN] = config[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, config)
        while time.monotonic_ns() < deadline:
            if child is not None and child.poll() is not None:
                break
            if not select.select([fd], [], [], .02)[0]:
                continue
            before = time.monotonic_ns()
            try:
                data = os.read(fd, 8192)
            except BlockingIOError:
                continue
            after = time.monotonic_ns()
            if not data:
                raise RuntimeError('serial EOF')
            raw.extend(data)
            batch = len(reads)
            reads.append({'batch': batch, 'read_start_ns': before,
                          'read_end_ns': after, 'bytes': len(data)})
            for kind, row, values in parser.feed(data, batch, before, after):
                if kind not in (0x40, 0x41) or values is None:
                    continue
                if not all(math.isfinite(x) for x in values):
                    parser.errors['nonfinite_values'] += 1
                    continue
                if kind == 0x41 and not .99 <= math.sqrt(sum(x*x for x in values[6:10])) <= 1.01:
                    parser.errors['invalid_quaternion'] += 1
                    continue
                latest[kind] = (row, values)
                if not all(k in latest for k in (0x40, 0x41)):
                    continue
                # Production fdilink_ahrs publishes only on an IMU frame,
                # pairing it with the latest previously parsed AHRS attitude.
                if kind != 0x40:
                    continue
                imu, iv = latest[0x40]
                ahrs, av = latest[0x41]
                norm = math.sqrt(sum(x*x for x in av[6:10]))
                # Exact device_type_=1 convention: q_r*q_ahrs*q_rr equals
                # (w,x,-y,-z); gyroscope and acceleration negate y/z.
                quat = (av[6]/norm, av[7]/norm, -av[8]/norm, -av[9]/norm)
                snapshot_version += 1
                body = (snapshot_version, imu['frame'], ahrs['frame'],
                        imu['device_us'], ahrs['device_us'], imu['read_end_ns'], ahrs['read_end_ns'],
                        imu['parsed_ns'], ahrs['parsed_ns'], *quat,
                        iv[0], -iv[1], -iv[2], iv[3], -iv[4], -iv[5])
                fcntl.flock(snapshot_file.fileno(), fcntl.LOCK_EX)
                try:
                    LAYOUT.pack_into(shared, 0, *body)
                finally:
                    fcntl.flock(snapshot_file.fileno(), fcntl.LOCK_UN)
                if args.consumer_binary and child is None:
                    child_log = (output / 'consumer_console.log').open('w')
                    root = Path(__file__).resolve().parents[1]
                    child = subprocess.Popen(['taskset', '--cpu-list', args.consumer_cpus,
                        str(args.consumer_binary.resolve()), '--phase', 'imu', '--workspace', str(root),
                        '--snapshot', str(output / 'snapshot.bin'), '--output', str(output / 'consumer'),
                        '--duration', str(args.duration), '--hz', str(args.consumer_hz)],
                        stdout=child_log, stderr=subprocess.STDOUT, start_new_session=True)
        if args.consumer_binary and (child is None or child.poll() is None):
            raise RuntimeError('isolated consumer did not finish within finite capture limit')
    except (Exception, KeyboardInterrupt) as exc:
        error = f'{type(exc).__name__}: {exc}'
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGTERM)
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        if child_log:
            child_log.close()
        if fd is not None:
            try:
                if original is not None:
                    termios.tcsetattr(fd, termios.TCSANOW, original)
            except OSError as exc:
                cleanup_errors.append('restore serial attributes: ' + str(exc))
            try:
                fcntl.ioctl(fd, termios.TIOCNXCL)
            except OSError as exc:
                cleanup_errors.append('release serial exclusivity: ' + str(exc))
            try:
                os.close(fd)
            except OSError as exc:
                cleanup_errors.append('close serial descriptor: ' + str(exc))
        shared.close()
        snapshot_file.close()
    finish = time.monotonic_ns()
    summary = {'port': args.port, 'serial_write_calls': 0, 'clock': 'CLOCK_MONOTONIC',
               'capture_duration_s': (finish-start)/1e9, 'requested_consumer_duration_s': args.duration,
               'error': error, 'cleanup_errors': cleanup_errors, 'consumer_exit_code': child.returncode if child else None,
               'frame_counts': dict(Counter(row['type'] for row in parser.frames)),
               'errors': dict(parser.errors), 'streams': {}}
    for kind, name in [(0x40, 'imu'), (0x41, 'ahrs')]:
        frames = [row for row in parser.frames if row['type'] == hex(kind) and row['device_us'] != '']
        if frames:
            first = frames[0]
            for row in frames:
                row['relative_host_device_us'] = ((row['read_end_ns'] - first['read_end_ns']) / 1000
                                                  - (row['device_us'] - first['device_us']))
        device_dt = [b['device_us'] - a['device_us'] for a, b in zip(frames, frames[1:])]
        host_dt = [(b['read_end_ns'] - a['read_end_ns'])/1e6 for a, b in zip(frames, frames[1:])]
        summary['streams'][name] = {'frames': len(frames), 'device_interval_us': stats(device_dt),
            'nonincreasing_device_timestamps': sum(x <= 0 for x in device_dt),
            'host_retrieval_interval_ms': stats(host_dt),
            'read_to_parse_ms': stats([(r['parsed_ns'] - r['read_end_ns'])/1e6 for r in frames]),
            'relative_host_device_phase_us': stats([r['relative_host_device_us'] for r in frames]),
            'cross_read_frames': sum(r['first_batch'] != r['last_batch'] for r in frames)}
    deltas = [((b['seq'] - a['seq']) & 255) for a, b in zip(parser.frames, parser.frames[1:])]
    summary['sequence'] = {'all_observed_type_mod256_deltas': dict(Counter(deltas)),
                           'interpretation': 'Raw sequence transitions; shared-counter semantics assumed, not independently proven packet loss.'}
    summary['limits'] = [
        'read_start/end describe userspace retrieval; UART, USB and driver buffering before retrieval remain unmeasured.',
        'Device timestamps and host monotonic timestamps are unsynchronized; relative phase removes only the initial offset.',
        'No absolute sensor latency or internal filter delay can be inferred.',
        'Consumer executes the production estimator/NMPC/WBC algorithms in an isolated harness, with synthetic standing joints.',
        'The consumer is not the production ROS controller callback/executor; shared-memory locking/dispatch overhead is included.',
        'IMU and AHRS records retain separate device/retrieval times; latest gyro and orientation need not be acquired simultaneously.',
    ]
    write_csv(output / 'frames.csv', parser.frames, FRAME_FIELDS)
    write_csv(output / 'reads.csv', reads, ['batch', 'read_start_ns', 'read_end_ns', 'bytes'])
    (output / 'raw.bin').write_bytes(raw)
    (output / 'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False) + '\n')
    if (output / 'consumer/metadata.json').exists():
        analyze(output / 'consumer')
    print(json.dumps({'output': str(output), 'frames': summary['frame_counts'],
                      'errors': summary['errors'], 'consumer_exit_code': summary['consumer_exit_code'], 'error': error}))
    protocol_failed = any(count for key, count in parser.errors.items() if key != 'discarded_bytes')
    return 2 if error or cleanup_errors or protocol_failed or child and child.returncode or not all(k in latest for k in (0x40, 0x41)) else 0


class ProtocolTests(unittest.TestCase):
    @staticmethod
    def frame(kind, seq, payload):
        header = bytes([0xfc, kind, len(payload), seq])
        checksum = binascii.crc_hqx(payload, 0)
        return header + bytes([crc8(header), checksum >> 8, checksum & 255]) + payload + b'\xfd'

    def test_crc_known_vectors(self):
        self.assertEqual(crc8(b'123456789'), 0xa1)
        self.assertEqual(binascii.crc_hqx(b'123456789', 0), 0x31c3)

    def test_split_frames_and_recovery(self):
        parser = Parser()
        valid = self.frame(0x40, 255, struct.pack('<12fq', *([0.] * 12), 123456))
        damaged = bytearray(valid)
        damaged[8] ^= 1
        self.assertFalse(parser.feed(b'garbage' + valid[:5], 0, 10, 20))
        parsed = parser.feed(valid[5:] + bytes(damaged) + valid, 1, 30, 40)
        self.assertEqual(len(parsed), 2)
        self.assertEqual(parsed[0][1]['device_us'], 123456)
        self.assertEqual(parsed[0][1]['first_batch'], 0)
        self.assertEqual(parsed[0][1]['last_batch'], 1)
        self.assertEqual(parser.errors['trailer_or_payload_crc16'], 1)

    def test_snapshot_layout(self):
        self.assertEqual(LAYOUT.size, 152)
        values = (1, 2, 3, 10000, 10010, 20000, 20010, 30000, 30010, *([0.] * 10))
        self.assertEqual(LAYOUT.unpack(LAYOUT.pack(*values)), values)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--consumer-hz', type=float, default=1000)
    parser.add_argument('--consumer-cpus', default='4-7')
    parser.add_argument('--consumer-binary', type=Path)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        suite = unittest.defaultTestLoader.loadTestsFromTestCase(ProtocolTests)
        return 0 if unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful() else 1
    if not args.output or not math.isfinite(args.duration) or not 1 <= args.duration <= 120:
        parser.error('--output and a finite --duration in [1,120] are required')
    if not math.isfinite(args.consumer_hz) or not 1 <= args.consumer_hz <= 2000:
        parser.error('--consumer-hz must be in [1,2000]')
    return capture(args)


if __name__ == '__main__':
    raise SystemExit(main())
