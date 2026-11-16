#!/usr/bin/env python3
"""Decode bounded cycle traces offline; never imports ROS or accesses hardware.

Schema mirrors diagnostics/CycleTraceTypes.hpp. All quantities below are host
records/software requests; encoded packets do not establish motor execution time
or measured physical torque. NumPy is the only non-standard dependency.
"""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import numpy as np

HEADER = struct.Struct('<8sHHIIIQQQIIII')
FOOTER = struct.Struct('<8sIIQQQQQQ')
NAMES = [f'{leg}_{joint}_joint' for leg in ('FR', 'FL', 'RR', 'RL') for joint in ('hip', 'thigh', 'calf')]
MODES = {0: 'PASSIVE', 1: 'CALIBRATION', 2: 'STAND_UP', 3: 'MPC_STANCE', 4: 'MPC_TROT', 5: 'FAULT'}
COMMAND = np.dtype([(n, '<f8', (12,)) for n in ('position', 'velocity', 'effort', 'kp', 'kd')], align=True)
FEEDBACK = np.dtype([(n, '<f8', (12,)) for n in ('position', 'velocity', 'effort', 'valid')], align=True)
MOTOR = np.dtype([(n, '<u8') for n in ('sdk_start_ns', 'sdk_return_ns', 'parsed_ns', 'flags')] +
                 [(n, '<i8') for n in ('received_id', 'received_mode', 'motor_error')] +
                 [(n, '<f8') for n in ('position', 'velocity', 'effort', 'kp', 'kd', 'gear', 'direction', 'calibration_offset')] +
                 [('tx', 'u1', (17,)), ('rx', 'u1', (16,)), ('reserved', 'u1', (7,))], align=True)
ROUND = np.dtype([(n, '<u8') for n in ('kind', 'start_ns', 'end_ns')] + [('motors', MOTOR, (12,))], align=True)
SDK = np.dtype([('round_count', '<u8'), ('omitted_rounds', '<u8'), ('rounds', ROUND, (2,))], align=True)
CONTROLLER_DTYPE = np.dtype(
    [(n, '<u8') for n in ('cycle', 'manager_time_ns', 'start_ns', 'end_ns', 'policy_sequence', 'mode', 'policy_mode', 'flags')] +
    [(n, '<f8') for n in ('period_s', 'query_time_s', 'policy_age_s', 'source_observation_s', 'policy_first_time_s', 'policy_last_time_s', 'node_left_time_s', 'node_right_time_s', 'handoff_alpha', 'imu_stamp_s', 'imu_use_ros_s', 'wbc_solve_ms', 'equality_residual', 'inequality_violation', 'motor_pd_horizon_s', 'reserved')] +
    [('feedback', FEEDBACK), ('wbc_command', COMMAND), ('output_command', COMMAND)] +
    [(n, '<f8', (12,)) for n in ('policy_position', 'policy_velocity', 'nominal_position', 'nominal_velocity', 'previous_nominal_position', 'previous_nominal_velocity', 'node_left_position', 'node_right_position', 'node_left_velocity', 'node_right_velocity', 'qp_torque', 'qp_pd_nominal', 'qp_pd_lower', 'qp_pd_upper', 'output_p', 'output_d', 'output_total')] +
    [(n, '<f8', (3,)) for n in ('base_position', 'base_velocity', 'base_euler_zyx')], align=True)
HARDWARE_DTYPE = np.dtype(
    [(n, '<u8') for n in ('read_sequence', 'manager_time_ns', 'source_write_sequence', 'source_write_time_ns', 'source_write_steady_ns', 'read_start_ns', 'read_end_ns', 'flags')] +
    [(n, '<f8') for n in ('calibration_request', 'emergency_stop_request', 'motion_request', 'motion_enabled', 'calibrated_before', 'calibrated_after', 'alignment_before', 'alignment_after')] +
    [(n, COMMAND) for n in ('requested_command', 'staged_command', 'sdk_command')] +
    [(n, FEEDBACK) for n in ('before_feedback', 'after_feedback')] +
    [(n, '<f8', (12,)) for n in ('staged_mode', 'sdk_mode')] + [('sdk', SDK)], align=True)
assert (HEADER.size, FOOTER.size, COMMAND.itemsize, FEEDBACK.itemsize, MOTOR.itemsize, ROUND.itemsize,
        CONTROLLER_DTYPE.itemsize, HARDWARE_DTYPE.itemsize) == (64, 64, 480, 384, 160, 1944, 3240, 6432)
DTYPES = {1: CONTROLLER_DTYPE, 2: HARDWARE_DTYPE}


def finite_json(obj):
    if isinstance(obj, dict): return {str(k): finite_json(v) for k, v in obj.items()}
    if isinstance(obj, (tuple, list)): return [finite_json(v) for v in obj]
    if isinstance(obj, np.ndarray): return finite_json(obj.tolist())
    if isinstance(obj, np.generic): return finite_json(obj.item())
    if isinstance(obj, float) and not math.isfinite(obj): return None
    return obj


def stats(values):
    a = np.asarray(values, dtype=float).reshape(-1)
    good = np.sort(a[np.isfinite(a)])
    if not len(good): return {'samples': 0, 'nonfinite_excluded': int(len(a))}
    return {'samples': len(good), 'nonfinite_excluded': int(len(a)-len(good)), 'mean': float(np.mean(good)),
            'rms': float(np.sqrt(np.mean(good*good))), 'min': float(good[0]), 'max': float(good[-1]),
            **{key: float(good[max(0, math.ceil(p*len(good))-1)]) for key, p in [('p50', .5), ('p95', .95), ('p99', .99), ('p999', .999)]}}


def per_joint(values, valid=None):
    a = np.asarray(values, dtype=float).reshape((-1, 12))
    if valid is not None: a = np.where(valid, a, np.nan)
    return {name: stats(a[:, j]) for j, name in enumerate(NAMES)}


def nonfinite_fields(records, prefix=''):
    out = {}
    for field in records.dtype.names or ():
        a = records[field]
        name = prefix + field
        if a.dtype.names: out.update(nonfinite_fields(a, name + '.'))
        elif a.dtype.kind == 'f':
            n = int(np.count_nonzero(~np.isfinite(a)))
            if n: out[name] = n
    return out


def parse_file(path):
    """Return (read-only structured records, quality); reject unsupported schemas."""
    path = Path(path)
    before = path.stat()
    with path.open('rb') as stream:
        raw = stream.read(64)
        if len(raw) != 64: raise ValueError(f'{path}: truncated header')
        fields = HEADER.unpack(raw)
        keys = ('magic', 'version', 'header_size', 'record_size', 'type_id', 'endian', 'wall_start_ns',
                'monotonic_start_ns', 'max_bytes', 'ring_capacity', 'alignment', 'clock_id', 'flags')
        header = dict(zip(keys, fields))
        if header['magic'] != b'DGCYCL01': raise ValueError(f'{path}: bad header magic')
        if header['version'] != 1 or header['header_size'] != 64: raise ValueError(f'{path}: unsupported version/header size')
        if header['type_id'] not in DTYPES: raise ValueError(f'{path}: unsupported type_id')
        dtype = DTYPES[header['type_id']]
        if header['record_size'] != dtype.itemsize: raise ValueError(f'{path}: record size/schema mismatch')
        if header['endian'] != 0x01020304 or header['alignment'] != 8 or header['clock_id'] != 1:
            raise ValueError(f'{path}: unsupported endian/alignment/clock')
        footer = None
        if before.st_size >= 128:
            stream.seek(-64, 2)
            raw = stream.read(64)
            if raw[:8] == b'DGCEND01':
                f = FOOTER.unpack(raw)
                if f[1] != 1 or f[2] not in (1, 2, 3): raise ValueError(f'{path}: invalid footer schema/reason')
                footer = dict(zip(('magic', 'version', 'reason', 'accepted', 'written', 'dropped_full', 'discarded', 'bytes_written', 'io_errors'), f))
        payload_bytes = before.st_size - 64 - (64 if footer else 0)
        count, partial = divmod(payload_bytes, dtype.itemsize)
        digest = hashlib.sha256()
        stream.seek(0)
        while chunk := stream.read(1024*1024): digest.update(chunk)
    records = np.memmap(path, dtype=dtype, mode='r', offset=64, shape=(count,)) if count else np.empty(0, dtype=dtype)
    issues = []
    if footer is None: issues.append('missing_footer_unclean')
    if partial: issues.append('trailing_bytes_excluded')
    if footer:
        if footer['bytes_written'] != before.st_size: issues.append('footer_byte_count_mismatch')
        if not (header['flags'] & 4) and footer['written'] != count: issues.append('footer_record_count_mismatch')
        if footer['accepted'] < footer['written']: issues.append('footer_counter_inconsistent')
        if footer['io_errors'] or footer['reason'] == 3: issues.append('writer_io_failure')
    after = path.stat()
    if (before.st_ino, before.st_size, before.st_mtime_ns) != (after.st_ino, after.st_size, after.st_mtime_ns): issues.append('source_changed_during_read')
    return records, {'path': str(path), 'size_bytes': before.st_size, 'sha256': digest.hexdigest(),
                     'header': {k: v for k, v in header.items() if k != 'magic'},
                     'footer': {k: v for k, v in footer.items() if k != 'magic'} if footer else None,
                     'complete_records': count, 'trailing_bytes_excluded': partial, 'issues': issues,
                     'cleanly_closed': not issues, 'nonfinite_fields': nonfinite_fields(records)}


_CRC_TABLE = []
for _i in range(256):
    _c = _i
    for _ in range(8): _c = (_c >> 1) ^ (0x8408 if _c & 1 else 0)
    _CRC_TABLE.append(_c)


def crc16(raw):
    c = 0
    for b in raw: c = (c >> 8) ^ _CRC_TABLE[(c ^ b) & 255]
    return c


def decode_tx(raw):
    """Exact local GO-M8010-6 SDK scales; q/dq use 6.2832, not exact 2*pi."""
    raw = bytes(raw)
    if len(raw) != 17: raise ValueError('TX must contain 17 bytes')
    tau, speed, position, kp, kd = struct.unpack_from('<hhiHH', raw, 3)
    return {'header_ok': raw[:2] == b'\xfe\xee', 'crc_ok': crc16(raw[:15]) == struct.unpack_from('<H', raw, 15)[0],
            'id': raw[2] & 15, 'mode': (raw[2] >> 4) & 7,
            'effort': tau/256., 'velocity': speed*6.2832/256., 'position': position*6.2832/32768.,
            'kp': kp/1280., 'kd': kd/1280., 'raw_values': (tau, speed, position, kp, kd)}


def decode_rx(raw):
    raw = bytes(raw)
    if len(raw) != 16: raise ValueError('RX must contain 16 bytes')
    tau, speed, position, temperature = struct.unpack_from('<hhib', raw, 3)
    return {'header_ok': raw[:2] == b'\xfd\xee', 'crc_ok': crc16(raw[:14]) == struct.unpack_from('<H', raw, 14)[0],
            'id': raw[2] & 15, 'mode': (raw[2] >> 4) & 7, 'motor_error': raw[12] & 7,
            'temperature_c': temperature, 'effort': tau/256., 'velocity': speed*6.2832/256.,
            'position': position*6.2832/32768.}


def controller_analysis(c, deadline_ms=4.):
    modes = {}
    for mode in np.unique(c['mode']):
        a = c[c['mode'] == mode]
        fb = a['feedback']
        valid = (fb['valid'] >= .5) & np.isfinite(fb['position']) & np.isfinite(fb['velocity'])
        output = a['output_command']
        p = output['kp']*(output['position']-fb['position'])
        d = output['kd']*(output['velocity']-fb['velocity'])
        w = a['wbc_command']
        wpd = w['kp']*(w['position']-fb['position']) + w['kd']*(w['velocity']-fb['velocity'])
        modes[MODES.get(int(mode), str(int(mode)))] = {
            'records': len(a), 'output_active_records': int(np.count_nonzero(a['flags'] & 32)),
            'invalid_joint_samples': int(np.count_nonzero(~valid)), 'FF_request_nm': per_joint(output['effort'], valid),
            'P_request_nm': per_joint(p, valid), 'D_request_nm': per_joint(d, valid),
            'D_reference_Kd_dqdes_nm': per_joint(output['kd']*output['velocity'], valid),
            'D_damping_minus_Kd_dq_nm': per_joint(-output['kd']*fb['velocity'], valid),
            'same_sample_total_request_nm': per_joint(output['effort']+p+d, valid),
            'recorded_total_minus_recomputed_nm': per_joint(a['output_total']-(output['effort']+p+d), valid),
            'wbc_same_sample_PD_minus_QP_model_bias_nm': per_joint(wpd-a['qp_pd_nominal'], valid & ((a['flags'] & 16) != 0)[:, None]),
            'controller_period_ms': stats(a['period_s']*1000), 'controller_period_over_deadline_count': int(np.count_nonzero(a['period_s']*1000 > deadline_ms)),
            'deadline_ms': deadline_ms, 'compute_before_diagnostics_ms': stats((a['end_ns'].astype(float)-a['start_ns'])/1e6),
            'policy_age_ms': stats(a['policy_age_s']*1000), 'wbc_solve_ms': stats(a['wbc_solve_ms'])}
    jumps, velocities, excluded = [], [], Counter()
    for prev, cur in zip(c[:-1], c[1:]):
        seq_change = cur['policy_sequence'] != prev['policy_sequence']
        if seq_change and int(prev['policy_sequence']) > 0 and int(cur['policy_sequence']) > 0 and int(cur['flags']) & 1024 and int(cur['flags']) & 512:
            jumps.append(cur['nominal_position']-cur['previous_nominal_position'])
        flags = int(cur['flags']); prevflags = int(prev['flags'])
        if int(cur['cycle']) != int(prev['cycle'])+1: excluded['nonconsecutive_cycle'] += 1; continue
        if seq_change: excluded['new_policy'] += 1; continue
        if cur['mode'] != prev['mode'] or cur['policy_mode'] != prev['policy_mode']: excluded['mode_event'] += 1; continue
        if flags & 64 or prevflags & 64: excluded['handoff'] += 1; continue
        required = 1 | 4 | 8 | 16 | 32 | 512
        if flags & required != required or prevflags & required != required: excluded['invalid_or_inactive'] += 1; continue
        dt = float(cur['query_time_s']-prev['query_time_s'])
        if not (0 < dt < .1 and 0 < float(cur['period_s']) < .1 and 0 < float(prev['period_s']) < .1): excluded['query_time_gap'] += 1; continue
        if (flags ^ prevflags) & (128 | 256): excluded['command_model_event'] += 1; continue
        valid = (cur['feedback']['valid'] >= .5) & (prev['feedback']['valid'] >= .5)
        residual = (cur['nominal_position']-prev['nominal_position'])/dt - .5*(cur['nominal_velocity']+prev['nominal_velocity'])
        velocities.append(np.where(valid, residual, np.nan))
    return {'by_mode': modes, 'same_query_new_minus_old_nominal_jump_rad': per_joint(jumps),
            'same_policy_no_event_qdot_minus_mean_dq_rad_s': per_joint(velocities),
            'qdot_comparison_pairs': len(velocities), 'qdot_excluded_pairs': dict(excluded)}


def join_records(c, h):
    """Join exact ROS stamps first, then a proven monotonic causal interval.

    The controller-manager can query now() independently at read/update/write.
    Fallback therefore requires end_i <= source_write_steady < start_(i+1),
    consecutive captured cycles and exact finite equality of all five requests.
    The last controller record has no bounded fallback interval.
    """
    by_time = defaultdict(list)
    for i, row in enumerate(c): by_time[int(row['manager_time_ns'])].append(i)
    duplicates = {str(t): len(v) for t, v in by_time.items() if len(v) > 1}
    pairs, counts, methods, source_counts = [], Counter(), Counter(), Counter()
    requested_differences = defaultdict(list); staged_sdk_differences = defaultdict(list)
    pending_read, ros_deltas, write_after_end, seq_offsets = [], [], [], []
    starts = c['start_ns'].astype(np.int64); ends = c['end_ns'].astype(np.int64)
    lower, upper = ends[:-1], starts[1:]
    consecutive = c['cycle'][1:] == c['cycle'][:-1]+1
    well_formed = (starts[:-1] <= ends[:-1]) & (ends[:-1] <= starts[1:]) & (starts[1:] <= ends[1:])
    monotonic = bool(len(c) < 2 or (np.all(np.diff(starts) > 0) and np.all(np.diff(ends) > 0) and np.all(well_formed)))

    def causal_candidates(stamp):
        if len(c) < 2 or stamp <= 0: return []
        if monotonic:
            i = int(np.searchsorted(ends, stamp, side='right'))-1
            return [i] if 0 <= i < len(c)-1 and int(lower[i]) <= stamp < int(upper[i]) else []
        return np.flatnonzero((lower <= stamp) & (stamp < upper)).tolist()

    def fields_match(i, row):
        return all(np.isfinite(row['requested_command'][field]).all() and
                   np.isfinite(c[i]['output_command'][field]).all() and
                   np.array_equal(row['requested_command'][field], c[i]['output_command'][field])
                   for field in COMMAND.names)

    for j, row in enumerate(h):
        flags = int(row['flags']); seq = int(row['source_write_sequence']); stamp = int(row['source_write_time_ns'])
        if not seq or not flags & 2: counts['no_source_write'] += 1; continue
        source_counts[seq] += 1
        if not flags & 4: counts['source_write_failed'] += 1; continue
        if not flags & 16: counts['source_write_not_fresh'] += 1; continue
        if flags & (512 | 1024 | 2048): counts['cleared_or_estop'] += 1; continue
        exact = by_time.get(stamp, [])
        steady = int(row['source_write_steady_ns']); causal = causal_candidates(steady)
        if len(exact) == 1:
            i = exact[0]
            if not fields_match(i, row): counts['exact_ros_field_mismatch_or_nonfinite'] += 1; continue
            method = 'exact_ros_stamp_and_finite_request'
            if steady and causal != [i]: counts['exact_ros_not_confirmed_by_bounded_causal_interval'] += 1
        else:
            if len(causal) > 1: counts['causal_ambiguous_overlap'] += 1; continue
            if not causal:
                counts['unmatched'] += 1
                if len(exact) > 1: counts['ambiguous_duplicate_controller_time'] += 1
                elif not steady: counts['source_write_monotonic_time_unavailable'] += 1
                elif len(c) and steady < int(ends[0]): counts['source_write_before_controller_coverage'] += 1
                elif len(c) and steady >= int(starts[-1]): counts['last_interval_or_after_controller_coverage'] += 1
                else: counts['causal_gap'] += 1
                continue
            i = causal[0]
            if not well_formed[i]: counts['causal_invalid_record_time_order'] += 1; continue
            if not consecutive[i]: counts['causal_missing_controller_cycle'] += 1; continue
            if not fields_match(i, row): counts['causal_field_mismatch_or_nonfinite'] += 1; continue
            method = 'bounded_monotonic_interval_and_finite_request'
        pairs.append((i, j)); counts['matched'] += 1; methods[method] += 1
        ros_deltas.append((stamp-int(c[i]['manager_time_ns']))/1e6)
        if steady >= int(c[i]['end_ns']): write_after_end.append((steady-int(c[i]['end_ns']))/1e6)
        seq_offsets.append(seq-int(c[i]['cycle']))
        for field in COMMAND.names:
            requested_differences[field].append(row['requested_command'][field]-c[i]['output_command'][field])
            staged_sdk_differences[field].append(row['sdk_command'][field]-row['staged_command'][field])
        if int(row['read_start_ns']) >= steady > 0: pending_read.append((int(row['read_start_ns'])-steady)/1e6)
    offsets = Counter(seq_offsets)
    return {'counts': dict(counts), 'methods': dict(methods), 'duplicate_controller_manager_times': duplicates,
            'reused_source_write_sequences': {str(k): v for k, v in source_counts.items() if v > 1},
            'matched_source_write_minus_controller_cycle_offsets': {str(k): v for k, v in offsets.items()},
            'source_write_ros_minus_controller_manager_time_ms': stats(ros_deltas),
            'source_write_monotonic_after_controller_end_ms': stats(write_after_end),
            'causal_interval_quality': {'intervals': len(lower), 'missing_cycle_intervals': int(np.count_nonzero(~consecutive)),
                                        'invalid_time_order_intervals': int(np.count_nonzero(~well_formed)),
                                        'strictly_monotonic_nonoverlapping': monotonic,
                                        'last_controller_record_has_no_fallback_upper_bound': bool(len(c))},
            'matched_controller_records': len(set(i for i, _ in pairs)), 'controller_records_without_match': len(c)-len(set(i for i, _ in pairs)),
            'requested_minus_controller_output_by_field': {k: per_joint(v) for k, v in requested_differences.items()},
            'sdk_caller_minus_staged_by_field': {k: per_joint(v) for k, v in staged_sdk_differences.items()},
            'source_write_to_next_read_start_ms': stats(pending_read),
            'requested_minus_controller_output_max_abs': 0.0 if pairs else None}, pairs


def packet_analysis(h, deadline_ms):
    counts = Counter(); per_motor = {name: defaultdict(list) for name in NAMES}; last_feedback = [None]*12
    packet_rows = []; io = []; pending = []; last_read_sequence = None; fields = ('position', 'velocity', 'effort', 'kp', 'kd')
    for record in h:
        read_sequence = int(record['read_sequence'])
        if last_read_sequence is not None and read_sequence != last_read_sequence + 1:
            last_feedback = [None]*12; counts['trace_read_sequence_discontinuities'] += 1
        last_read_sequence = read_sequence
        io.append((int(record['read_end_ns'])-int(record['read_start_ns']))/1e6)
        counts['omitted_sdk_rounds'] += int(record['sdk']['omitted_rounds'])
        counts['reported_rounds_over_capture_capacity'] += int(record['sdk']['round_count'] > 2)
        flags = int(record['flags'])
        stage_valid = flags & (2 | 4 | 16) == (2 | 4 | 16) and not flags & (512 | 1024 | 2048)
        caller_matches_staged = all(np.array_equal(record['sdk_command'][key], record['staged_command'][key]) for key in COMMAND.names)
        for round_index in range(min(2, int(record['sdk']['round_count']))):
            round = record['sdk']['rounds'][round_index]
            for j, m in enumerate(round['motors']):
                mf = int(m['flags'])
                if not mf & 1: continue
                counts['attempted_packets'] += 1
                counts['round_kind_' + str(int(round['kind'])) + '_packet_attempts'] += 1
                start, returned, parsed = (int(m[k]) for k in ('sdk_start_ns', 'sdk_return_ns', 'parsed_ns'))
                entry = per_motor[NAMES[j]]
                if returned >= start > 0: entry['sdk_call_ms'].append((returned-start)/1e6)
                tx = decode_tx(m['tx'])
                counts['tx_bad_crc'] += not tx['crc_ok']; counts['tx_bad_header'] += not tx['header_ok']
                counts['tx_id_mismatch'] += tx['id'] != j % 3
                counts['tx_reserved_mode_bit_set'] += bool(int(m['tx'][2]) & 128)
                if int(round['kind']) == 1 and stage_valid and float(record['calibrated_before']) >= .5:
                    counts['tx_mode_differs_from_sdk_caller'] += tx['mode'] != int(record['sdk_mode'][j])
                expected = [math.trunc(float(m['effort'])*256), math.trunc(float(m['velocity'])/6.2832*256),
                            math.trunc(float(m['position'])/6.2832*32768), math.trunc(float(m['kp'])*1280), math.trunc(float(m['kd'])*1280)] if all(np.isfinite(m[k]) for k in fields) else None
                if expected is None: counts['nonfinite_sdk_command_records'] += 1
                else: counts['tx_fixed_point_mismatch'] += tuple(expected) != tx['raw_values']
                for k in fields:
                    if np.isfinite(m[k]): entry[f'tx_decode_minus_sdk_{k}'].append(tx[k]-float(m[k]))
                gear, direction = float(m['gear']), float(m['direction'])
                packet_matches_caller = False
                if gear > 0 and math.isfinite(gear) and direction in (-1., 1.) and int(round['kind']) == 1 and stage_valid:
                    s = record['sdk_command']; expected_rotor = {
                        'position': (float(s['position'][j])+float(m['calibration_offset']))*gear*direction if s['kp'][j] != 0 else 0.,
                        'velocity': float(s['velocity'][j])*gear*direction if s['kd'][j] != 0 else 0.,
                        'effort': float(s['effort'][j])/gear*direction, 'kp': float(s['kp'][j])/gear**2, 'kd': float(s['kd'][j])/gear**2}
                    for k in fields: entry[f'sdk_field_minus_staged_rotor_{k}'].append(float(m[k])-expected_rotor[k])
                    packet_matches_caller = (tx['crc_ok'] and tx['header_ok'] and tx['mode'] == int(record['sdk_mode'][j]) and tx['id'] == j % 3
                                             and all(math.isclose(float(m[k]), expected_rotor[k], rel_tol=2e-6, abs_tol=1e-7) for k in fields))
                normal_pending = stage_valid and caller_matches_staged and packet_matches_caller and int(round['kind']) == 1
                if normal_pending and start >= int(record['source_write_steady_ns']) > 0:
                    delay = (start-int(record['source_write_steady_ns']))/1e6
                    entry['pending_write_to_sdk_start_ms'].append(delay); pending.append(delay)
                elif int(round['kind']) == 1:
                    counts['ordinary_packet_not_eligible_for_normal_pending'] += 1
                rx = None
                if mf & 2:
                    rx = decode_rx(m['rx']); counts['received_packets'] += 1
                    counts['rx_bad_crc'] += not rx['crc_ok']; counts['rx_unexpected_header'] += not rx['header_ok']
                    if rx['crc_ok']:
                        counts['rx_motor_error_nonzero'] += rx['motor_error'] != 0
                        counts['rx_id_mismatch'] += rx['id'] != j % 3
                        counts['rx_mode_mismatch'] += rx['mode'] != tx['mode']
                        physical_valid = rx['id'] == j % 3 and rx['mode'] == tx['mode'] and rx['motor_error'] == 0
                        if mf & 512 and not physical_valid: counts['producer_success_inconsistent_rx_fields'] += 1
                        if mf & 512 and physical_valid:
                            counts['successful_feedback_packets'] += 1
                            if parsed and last_feedback[j] is not None and parsed >= last_feedback[j]: entry['valid_feedback_interval_ms'].append((parsed-last_feedback[j])/1e6)
                            if parsed: last_feedback[j] = parsed
                            entry['temperature_c'].append(rx['temperature_c'])
                    if mf & 256 and not rx['crc_ok']: counts['producer_frame_valid_but_crc_bad'] += 1
                counts['producer_failed_packets'] += not bool(mf & 512)
                packet_rows.append({'read_sequence': int(record['read_sequence']), 'source_write_sequence': int(record['source_write_sequence']),
                                    'source_write_time_ns': int(record['source_write_time_ns']), 'round_index': round_index,
                                    'round_kind': int(round['kind']), 'joint': NAMES[j], 'flags': mf, 'sdk_start_ns': start, 'sdk_return_ns': returned,
                                    'parsed_ns': parsed, 'tx_hex': bytes(m['tx']).hex(), 'rx_hex': bytes(m['rx']).hex(),
                                    'tx_crc_ok': tx['crc_ok'], 'rx_crc_ok': rx['crc_ok'] if rx else None})
    return {'counts': dict(counts), 'whole_read_io_ms': stats(io), 'whole_read_over_deadline_count': sum(x > deadline_ms for x in io),
            'deadline_ms': deadline_ms, 'pending_write_to_sdk_start_ms': stats(pending),
            'motors': {name: {k: stats(v) for k, v in entry.items()} for name, entry in per_motor.items()}}, packet_rows


def analyze(trace_dir, csv_dir=None, deadline_ms=4.):
    files = sorted(Path(trace_dir).glob('*.bin'))
    if not files: raise ValueError('no binary trace files found')
    groups = defaultdict(list); qualities = []
    for path in files:
        records, quality = parse_file(path); qualities.append(quality); groups[quality['header']['type_id']].append(records)
    c = np.concatenate(groups[1]) if groups[1] else np.empty(0, dtype=CONTROLLER_DTYPE)
    h = np.concatenate(groups[2]) if groups[2] else np.empty(0, dtype=HARDWARE_DTYPE)
    c = c[np.argsort(c['cycle'], kind='stable')]; h = h[np.argsort(h['read_sequence'], kind='stable')]
    retention = {}
    for stem in ('controller', 'hardware'):
        indices = sorted(int(m[1]) for path in files if (m := re.fullmatch(stem+r'\.(\d+)\.bin', path.name)))
        if indices: retention[stem] = {'retained_segment_indices': indices, 'missing_between_retained_count': max(indices)-min(indices)+1-len(indices)}
    join, pairs = join_records(c, h); packet, packet_rows = packet_analysis(h, deadline_ms)
    drop = {}
    for tid, name in ((1, 'controller'), (2, 'hardware')):
        qs = [q for q in qualities if q['header']['type_id'] == tid]
        footers = [q['footer'] for q in qs if q['footer']]
        cumulative = any(q['header']['flags'] & 4 for q in qs)
        final_known = not qs or (qs[-1]['footer'] is not None and qs[-1]['footer']['reason'] == 1 and qs[-1]['cleanly_closed'])
        drop[name] = {'final_counters_known': final_known, 'stream_present': bool(qs), 'counter_scope': 'cumulative_recorder' if cumulative else 'per_file',
                      **{k: (max((f[k] for f in footers), default=0) if cumulative else sum(f[k] for f in footers)) for k in ('accepted', 'written', 'dropped_full', 'discarded', 'io_errors')}}
    result = {'schema_version': 1, 'joint_order': NAMES, 'sources': qualities, 'retention': retention, 'writer_counters': drop,
              'records': {'controller': len(c), 'hardware': len(h)},
              'duplicate_sequence_counts': {'controller_cycle': len(c)-len(np.unique(c['cycle'])), 'hardware_read_sequence': len(h)-len(np.unique(h['read_sequence']))},
              'sampling_quality': {'all_retained_files_cleanly_closed': all(q['cleanly_closed'] for q in qualities),
                                   'final_writer_counters_complete': all(v['final_counters_known'] for v in drop.values()),
                                   'ring_dropped_or_discarded': (True if any(v['dropped_full'] or v['discarded'] for v in drop.values()) else False if all(v['final_counters_known'] for v in drop.values()) else None),
                                   'retained_window_has_segment_gaps': any(v['missing_between_retained_count'] for v in retention.values())},
              'join': join, 'controller': controller_analysis(c, deadline_ms), 'hardware': packet,
              'limits': ['Exact ROS stamp join preferred; otherwise a bounded monotonic controller-end to next-controller-start interval plus exact finite five-field request equality proves the join. Missing cycles, ambiguous intervals and the final unbounded interval are rejected; row numbers and nearest ROS timestamps are never alignment.',
                         'Invalid joint feedback and nonfinite values are excluded from software PD/physical motion summaries; NaNs remain counted in source quality.',
                         'Same-policy qdot checks exclude gaps, handoff, mode changes and inactive/invalid records. Same-query old/new jumps require flags512+1024 and a changed policy sequence.',
                         'Controller end_ns is before diagnostic publication and is not the complete manager scheduling cost.',
                         'SDK read trace excludes lifecycle Deactivate packets emitted outside the captured hardware read. SDK packets show host transmission, quantization and received feedback. They do not establish motor execution time, actual torque/current or measured foot forces.',
                         'Pending write to SDK start is a host-side delay. Header/SDK CLOCK_MONOTONIC and controller manager ROS timestamps are different domains; they are not subtracted.',
                         'RX unexpected header is reported separately because the local SDK validates CRC without independently rejecting header bytes.',
                         'Missing footer, truncation, ring loss, discarded records and intentional segment retention gaps limit waveform and worst-case claims.']}
    if csv_dir is not None:
        target = Path(csv_dir); target.mkdir(parents=True, exist_ok=True)
        if packet_rows:
            with (target/'packets.csv').open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(packet_rows[0])); writer.writeheader(); writer.writerows(packet_rows)
        with (target/'joins.csv').open('w', newline='') as stream:
            writer = csv.writer(stream); writer.writerow(['controller_cycle', 'controller_manager_time_ns', 'hardware_read_sequence', 'source_write_sequence', 'source_write_time_ns'])
            for i, j in pairs: writer.writerow([int(c[i]['cycle']), int(c[i]['manager_time_ns']), int(h[j]['read_sequence']), int(h[j]['source_write_sequence']), int(h[j]['source_write_time_ns'])])
    return finite_json(result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace-dir', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--csv', action='store_true', help='also write packets.csv and joins.csv beside JSON')
    parser.add_argument('--deadline-ms', type=float, default=4.)
    args = parser.parse_args()
    if args.deadline_ms <= 0 or not math.isfinite(args.deadline_ms): parser.error('deadline must be positive')
    output = args.output if args.output.suffix == '.json' else args.output/'summary.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    try: result = analyze(args.trace_dir, output.parent if args.csv else None, args.deadline_ms)
    except (ValueError, OSError) as error: parser.exit(2, str(error)+'\n')
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(output)

if __name__ == '__main__': main()
