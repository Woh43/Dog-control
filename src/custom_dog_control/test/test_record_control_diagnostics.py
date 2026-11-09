#!/usr/bin/env python3
"""Pure recorder tests; construct messages but never start a ROS node."""

import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/record_control_diagnostics.py'
SPEC = importlib.util.spec_from_file_location('record_control_diagnostics', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def message(mode='MPC_STANCE', enabled='false', level=DiagnosticStatus.OK):
    value = DiagnosticArray()
    value.header.stamp.sec = 1794243897
    value.header.stamp.nanosec = 437698100
    value.header.frame_id = 'base'
    status = DiagnosticStatus()
    status.level = level
    status.name = MODULE.CONTROL_STATUS_NAME
    status.message = 'motor communication failure' if mode == 'FAULT' else mode
    status.hardware_id = 'custom_dog_rs485'
    status.values = [KeyValue(key='mode', value=mode),
                     KeyValue(key='trot_enabled', value=enabled),
                     KeyValue(key='mpc_policy_age_s', value='inf'),
                     KeyValue(key='test_nan', value='nan'),
                     KeyValue(key='duplicate', value='first'),
                     KeyValue(key='duplicate', value='second')]
    value.status = [status]
    return value


class DiagnosticConversionTests(unittest.TestCase):
    def test_real_ros_array_preserves_bytes_level_timestamps_and_all_values(self):
        source = message('FAULT', level=DiagnosticStatus.ERROR)
        self.assertIsInstance(source.status[0].level, bytes)
        record = MODULE.diagnostic_record(source, received_wall_time_ns=123456789)
        restored = json.loads(json.dumps(record, allow_nan=False))
        self.assertEqual(restored['header']['stamp'],
                         {'sec': 1794243897, 'nanosec': 437698100})
        self.assertEqual(restored['received_wall_time_ns'], 123456789)
        status = restored['status'][0]
        self.assertEqual(status['level'], 2)
        self.assertEqual(status['hardware_id'], 'custom_dog_rs485')
        self.assertEqual(status['message'], 'motor communication failure')
        self.assertEqual(status['values'],
                         [{'key': item.key, 'value': item.value}
                          for item in source.status[0].values])
        self.assertTrue(MODULE.requires_immediate_flush(restored))

    def test_level_byte_and_integer_representations(self):
        for value in (b'\x02', bytearray(b'\x02'), memoryview(b'\x02'), 2):
            self.assertEqual(MODULE.diagnostic_level(value), 2)
        for value in (b'', b'\x00\x01', -1, 256):
            with self.assertRaises(ValueError):
                MODULE.diagnostic_level(value)

    def test_error_and_fault_are_flushed_without_waiting_for_timer(self):
        self.assertFalse(MODULE.requires_immediate_flush(MODULE.diagnostic_record(message())))
        self.assertTrue(MODULE.requires_immediate_flush(
            MODULE.diagnostic_record(message(level=DiagnosticStatus.ERROR))))
        self.assertTrue(MODULE.requires_immediate_flush(
            MODULE.diagnostic_record(message('FAULT', level=DiagnosticStatus.OK))))

    def test_notices_report_mode_and_trot_changes_once_without_claiming_acceptance(self):
        reporter = MODULE.ModeNotices()
        standing = MODULE.diagnostic_record(message())
        notices = reporter.update(standing)
        self.assertTrue(any('站立控制模式' in text for _, text in notices))
        self.assertEqual(reporter.update(standing), [])
        armed = MODULE.diagnostic_record(message(enabled='true'))
        notices = reporter.update(armed)
        self.assertTrue(any('已使能行走' in text and '需推动摇杆' in text
                            for _, text in notices))
        self.assertEqual(reporter.update(armed), [])
        fault = MODULE.diagnostic_record(message('FAULT', level=DiagnosticStatus.ERROR))
        notices = reporter.update(fault)
        self.assertTrue(any(severity == 'error' and '故障已锁存' in text
                            and 'motor communication failure' in text
                            for severity, text in notices))
        self.assertEqual(reporter.update(fault), [])


class SegmentWriterTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / 'diagnostics.jsonl'

    def test_jsonl_flush_and_close_persist_complete_messages(self):
        writer = MODULE.JsonlSegments(self.path)
        first = MODULE.diagnostic_record(message(), received_wall_time_ns=1)
        second = MODULE.diagnostic_record(message('FAULT'), received_wall_time_ns=2)
        writer.append(first)
        writer.flush()
        self.assertEqual([json.loads(line) for line in self.path.read_text().splitlines()],
                         [first])
        writer.append(second)
        writer.close()
        writer.close()
        self.assertEqual([json.loads(line) for line in self.path.read_text().splitlines()],
                         [first, second])
        with self.assertRaisesRegex(ValueError, 'closed'):
            writer.append(first)

    def test_rotation_preserves_initial_and_latest_three_complete_utf8_segments(self):
        # Each record has the same byte count, including multibyte Chinese.
        sample = {'index': 0, 'text': '故障详情'}
        line = json.dumps(sample, ensure_ascii=False, allow_nan=False,
                          separators=(',', ':')) + '\n'
        limit = len(line.encode('utf-8'))
        writer = MODULE.JsonlSegments(self.path, max_segment_bytes=limit)
        for index in range(9):
            writer.append({'index': index, 'text': '故障详情'}, force_flush=True)
            self.assertLessEqual(len(list(self.path.parent.glob('*.jsonl'))), 4)
            self.assertLessEqual(len(writer.retained_paths), 4)
            for path in writer.retained_paths:
                self.assertLessEqual(path.stat().st_size, limit)
        paths = writer.retained_paths
        writer.close()
        self.assertEqual([json.loads(path.read_text())['index'] for path in paths],
                         [0, 6, 7, 8])
        self.assertEqual(self.path.read_text(), line)

    def test_oversized_record_fails_instead_of_silently_dropping_or_exceeding_bound(self):
        writer = MODULE.JsonlSegments(self.path, max_segment_bytes=32)
        with self.assertRaisesRegex(ValueError, 'exceeds'):
            writer.append({'data': 'x' * 100})
        writer.close()
        self.assertEqual(self.path.stat().st_size, 0)

    def test_existing_log_is_never_truncated(self):
        self.path.write_text('earlier evidence\n')
        with self.assertRaises(FileExistsError):
            MODULE.JsonlSegments(self.path)
        self.assertEqual(self.path.read_text(), 'earlier evidence\n')

    def test_write_and_flush_errors_propagate(self):
        stream = mock.Mock(spec=io.TextIOBase)
        stream.write.side_effect = OSError('disk full')
        with mock.patch.object(MODULE.JsonlSegments, '_open', return_value=stream):
            writer = MODULE.JsonlSegments(self.path)
        with self.assertRaisesRegex(OSError, 'disk full'):
            writer.append({'mode': 'FAULT'})
        stream.write.side_effect = None
        stream.flush.side_effect = OSError('flush failed')
        with self.assertRaisesRegex(OSError, 'flush failed'):
            writer.flush()
        with self.assertRaisesRegex(OSError, 'flush failed'):
            writer.close()
        stream.close.assert_called_once()

    def test_short_write_and_rotation_open_error_propagate(self):
        stream = mock.Mock(spec=io.TextIOBase)
        stream.write.return_value = 1
        with mock.patch.object(MODULE.JsonlSegments, '_open', return_value=stream):
            writer = MODULE.JsonlSegments(self.path)
        with self.assertRaisesRegex(OSError, 'short'):
            writer.append({'mode': 'FAULT'})
        writer.close()
        writer = MODULE.JsonlSegments(self.path, max_segment_bytes=8)
        writer.append({'a': 1})
        with mock.patch.object(writer, '_open', side_effect=OSError('open failed')):
            with self.assertRaisesRegex(OSError, 'open failed'):
                writer.append({'a': 2})
        writer.close()
        self.assertEqual(json.loads(self.path.read_text()), {'a': 1})


if __name__ == '__main__':
    unittest.main()
