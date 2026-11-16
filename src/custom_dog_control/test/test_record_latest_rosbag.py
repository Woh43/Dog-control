#!/usr/bin/env python3
"""Filesystem/process tests only; no ROS node, controller or hardware access."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

SOURCE = Path(__file__).resolve().parents[1] / 'scripts/record_latest_rosbag.py'
SPEC = importlib.util.spec_from_file_location('latest_bag', SOURCE)
bag = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bag)


class LatestBagTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.latest = self.root / 'recording/latest'
        self.addCleanup(self.temp.cleanup)

    def segments(self, directory, count):
        directory.mkdir(parents=True, exist_ok=True)
        for index in range(count):
            (directory / f'bag_{index}.db3').write_bytes(b'closed')

    def test_new_run_replaces_only_owned_latest(self):
        untouched = self.root / 'unrelated.log'
        untouched.write_text('keep')
        with bag.LatestDirectory(self.latest):
            self.segments(self.latest / 'bag', 2)
        with bag.LatestDirectory(self.latest):
            self.assertTrue((self.latest / bag.OWNER).is_file())
            self.assertFalse((self.latest / 'bag').exists())
        self.assertEqual(untouched.read_text(), 'keep')

    def test_unowned_directory_is_never_deleted(self):
        self.latest.mkdir(parents=True)
        valuable = self.latest / 'important.txt'
        valuable.write_text('keep')
        with self.assertRaises(ValueError):
            with bag.LatestDirectory(self.latest):
                pass
        self.assertEqual(valuable.read_text(), 'keep')

    def test_wrong_marker_and_unknown_files_are_never_deleted(self):
        with bag.LatestDirectory(self.latest):
            pass
        marker = self.latest / bag.OWNER
        original = marker.read_text()
        marker.write_text('{}')
        with self.assertRaises(ValueError):
            with bag.LatestDirectory(self.latest):
                pass
        marker.write_text(original)
        (self.latest / 'unexpected.txt').write_text('keep')
        with self.assertRaises(ValueError):
            with bag.LatestDirectory(self.latest):
                pass
        self.assertEqual((self.latest / 'unexpected.txt').read_text(), 'keep')

    def test_symlinks_in_path_or_bag_are_rejected(self):
        outside = self.root / 'outside'
        outside.mkdir()
        (self.root / 'alias').symlink_to(outside, target_is_directory=True)
        with self.assertRaises(ValueError):
            bag.LatestDirectory(self.root / 'alias/latest')
        with bag.LatestDirectory(self.latest):
            (self.latest / 'bag').mkdir()
            (self.latest / 'bag/bag_0.db3').symlink_to(outside / 'safe')
        with self.assertRaises(ValueError):
            with bag.LatestDirectory(self.latest):
                pass
        self.assertTrue(outside.exists())

    def test_owned_metadata_temporary_is_cleaned_but_symlink_is_rejected(self):
        with bag.LatestDirectory(self.latest):
            self.segments(self.latest / 'bag', 1)
            (self.latest / 'bag/metadata.yaml.tmp').write_text('interrupted write')
        with bag.LatestDirectory(self.latest):
            self.assertFalse((self.latest / 'bag').exists())
            (self.latest / 'bag').mkdir()
            (self.latest / 'bag/metadata.yaml.tmp').symlink_to(self.root / 'outside')
        with self.assertRaises(ValueError):
            with bag.LatestDirectory(self.latest):
                pass
        self.assertTrue((self.latest / 'bag/metadata.yaml.tmp').is_symlink())

    def test_invalid_paths_and_symlink_lock_are_rejected(self):
        for value in ('latest', str(self.root), str(self.root / '../latest')):
            with self.assertRaises(ValueError):
                bag.safe_path(value)
        self.latest.parent.mkdir()
        sentinel = self.root / 'sentinel'
        sentinel.write_text('keep')
        (self.latest.parent / '.latest-rosbag.lock').symlink_to(sentinel)
        with self.assertRaises(OSError):
            with bag.LatestDirectory(self.latest):
                pass
        self.assertEqual(sentinel.read_text(), 'keep')

    def test_lock_excludes_second_writer_before_cleanup(self):
        with bag.LatestDirectory(self.latest):
            self.segments(self.latest / 'bag', 1)
            with self.assertRaises(BlockingIOError):
                with bag.LatestDirectory(self.latest):
                    pass
            self.assertTrue((self.latest / 'bag/bag_0.db3').exists())

    def test_inherited_lock_survives_parent_descriptor_close(self):
        process = None
        try:
            with bag.LatestDirectory(self.latest) as owned:
                process = subprocess.Popen(
                    [sys.executable, '-c', 'import sys; sys.stdin.read()'],
                    stdin=subprocess.PIPE, pass_fds=(owned.fd,))
            # The wrapper has closed its own descriptor, but the live native
            # subprocess's inherited descriptor still owns the same flock.
            with self.assertRaises(BlockingIOError):
                with bag.LatestDirectory(self.latest):
                    pass
            process.communicate(timeout=5)
            with bag.LatestDirectory(self.latest):
                pass
        finally:
            if process is not None and process.poll() is None:
                process.kill()
                process.wait(timeout=5)

    def test_rotation_keeps_first_and_latest_seven_and_never_active(self):
        directory = self.root / 'bag'
        self.segments(directory, 12)
        retention = bag.SegmentRetention(directory)
        held_open = directory / 'bag_1.db3'
        self.assertEqual(retention.prune({str(held_open)}), 3)
        self.assertTrue(held_open.exists())
        self.assertEqual(retention.prune(set()), 1)
        self.assertEqual([index for index, _ in bag.segment_paths(directory)],
                         [0, 5, 6, 7, 8, 9, 10, 11])
        self.assertEqual(retention.removed, 4)
        self.assertLessEqual(len(vars(retention)), 3)

    def test_sidecar_blocks_deletion_and_unlink_failure_propagates(self):
        directory = self.root / 'bag'
        self.segments(directory, 9)
        sidecar = directory / 'bag_1.db3-journal'
        sidecar.touch()
        retention = bag.SegmentRetention(directory)
        self.assertEqual(retention.prune(set()), 0)
        sidecar.unlink()
        with patch.object(Path, 'unlink', side_effect=OSError('disk failure')):
            with self.assertRaises(OSError):
                retention.prune(set())
        self.assertTrue((directory / 'bag_1.db3').exists())

    def test_missing_initial_segment_and_symlink_segment_fail(self):
        directory = self.root / 'bag'
        self.segments(directory, 2)
        (directory / 'bag_0.db3').unlink()
        with self.assertRaises(ValueError):
            bag.SegmentRetention(directory).prune(set())
        (directory / 'bag_0.db3').symlink_to(directory / 'bag_1.db3')
        with self.assertRaises(ValueError):
            bag.segment_paths(directory)

    def test_native_command_has_only_whitelisted_topics_and_bounded_cache(self):
        command = bag.recorder_command('/usr/bin/ros2', Path('/tmp/bag'), Path('/tmp/qos'), 131072)
        self.assertNotIn('-a', command)
        self.assertNotIn('/rosout', command)
        self.assertEqual(command[-len(bag.TOPICS):], list(bag.TOPICS))
        self.assertEqual(command[command.index('--max-cache-size') + 1], '4194304')
        self.assertEqual(command[command.index('--compression-mode') + 1], 'none')
        self.assertIn('/nmpc_wbc_controller/control_mode', command)
        self.assertIn('/nmpc_wbc_controller/contact_plan', command)
        self.assertIn('/imu', command)
        self.assertIn('/joint_states', command)

    def test_proc_reader_finds_an_actual_owned_open_file(self):
        file = self.root / 'sample'
        with file.open('w'):
            self.assertIn(str(file), bag.open_files(os.getpid()))

    def test_stop_signals_only_owned_group_and_waits_for_close(self):
        process = Mock(pid=987654)
        process.poll.return_value = None
        with patch.object(os, 'killpg') as kill:
            self.assertFalse(bag.stop_child(process))
            kill.assert_called_once_with(987654, signal.SIGINT)
            process.wait.assert_called_once_with(timeout=20)
        process.wait.side_effect = [subprocess.TimeoutExpired('owned', 20), 0]
        with patch.object(os, 'killpg') as kill:
            self.assertTrue(bag.stop_child(process))
            self.assertEqual([call.args for call in kill.call_args_list],
                             [(987654, signal.SIGINT), (987654, signal.SIGTERM)])

    def metadata_fixture(self):
        import yaml
        directory = self.root / 'bag'
        directory.mkdir()
        files = []
        for index, timestamps in ((0, (100, 150)), (9, (900, 950))):
            path = directory / f'bag_{index}.db3'
            with sqlite3.connect(path) as db:
                db.execute('CREATE TABLE topics(id INTEGER,name TEXT,type TEXT,serialization_format TEXT)')
                db.execute('CREATE TABLE messages(id INTEGER,topic_id INTEGER,timestamp INTEGER)')
                db.execute("INSERT INTO topics VALUES(1,'/imu','sensor_msgs/msg/Imu','cdr')")
                db.executemany('INSERT INTO messages VALUES(?,1,?)', enumerate(timestamps))
            files.append({'path': path.name, 'message_count': 2,
                          'starting_time': {'nanoseconds_since_epoch': timestamps[0]},
                          'duration': {'nanoseconds': timestamps[1] - timestamps[0]}})
        metadata = {'relative_file_paths': [f['path'] for f in files], 'files': files,
                    'message_count': 4, 'starting_time': {'nanoseconds_since_epoch': 100},
                    'duration': {'nanoseconds': 850},
                    'topics_with_message_count': [{'topic_metadata': {
                        'name': '/imu', 'type': 'sensor_msgs/msg/Imu', 'serialization_format': 'cdr'},
                        'message_count': 4}]}
        (directory / 'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': metadata}))
        return directory, metadata

    def test_metadata_is_verified_against_closed_sqlite(self):
        import yaml
        directory, metadata = self.metadata_fixture()
        self.assertEqual(bag.verify_metadata(directory), (4, 2))
        metadata['message_count'] = 5
        (directory / 'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': metadata}))
        with self.assertRaises(ValueError):
            bag.verify_metadata(directory)

    def test_humble_reindex_files_and_gap_duration_are_completed(self):
        import yaml
        directory, metadata = self.metadata_fixture()
        metadata['files'] = []
        metadata['duration']['nanoseconds'] = 100  # Sum of file durations loses the gap.
        (directory / 'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': metadata}))
        bag.complete_reindexed_metadata(directory)
        self.assertEqual(bag.verify_metadata(directory), (4, 2))
        saved = yaml.safe_load((directory / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
        self.assertEqual(saved['duration']['nanoseconds'], 850)
        self.assertEqual([item['path'] for item in saved['files']], ['bag_0.db3', 'bag_9.db3'])
        self.assertFalse((directory / 'metadata.yaml.tmp').exists())

    def test_reindex_failure_removes_stale_missing_file_metadata(self):
        directory, _ = self.metadata_fixture()
        retention = bag.SegmentRetention(directory)
        retention.removed = 1
        reindexer = Mock()
        reindexer.reindex.side_effect = RuntimeError('write failed')
        module = types.SimpleNamespace(Reindexer=lambda: reindexer, StorageOptions=lambda **k: k)
        with patch.dict('sys.modules', rosbag2_py=module):
            with self.assertRaises(RuntimeError):
                bag.finalize(directory, retention)
        self.assertFalse((directory / 'metadata.yaml').exists())
        self.assertTrue((directory / 'bag_0.db3').exists())

    def test_bad_metadata_is_not_left_playable(self):
        import yaml
        directory, metadata = self.metadata_fixture()
        metadata['relative_file_paths'].append('missing.db3')
        (directory / 'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': metadata}))
        with self.assertRaises(ValueError):
            bag.finalize(directory, bag.SegmentRetention(directory))
        self.assertFalse((directory / 'metadata.yaml').exists())


if __name__ == '__main__':
    unittest.main()
