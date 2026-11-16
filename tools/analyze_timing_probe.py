#!/usr/bin/env python3
"""Summarize host timing CSVs; does not access hardware or alter configuration."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics


def stats(values):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return {'samples': 0}
    return {'samples': len(values), 'mean': statistics.fmean(values),
            **{key: values[max(0, math.ceil(len(values) * fraction) - 1)]
               for key, fraction in [('p50', .5), ('p95', .95), ('p99', .99), ('p999', .999)]},
            'min': values[0], 'max': values[-1]}


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def valid_zero_packet(raw, motor_index):
    if len(raw) != 17 or raw[:2] != b'\xfe\xee' or raw[2] & 0x70 or any(raw[3:15]):
        return False
    if raw[2] & 15 != motor_index % 3:
        return False
    crc = 0
    for byte in raw[:15]:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc == (raw[15] | raw[16] << 8)


def analyze(path):
    meta = json.loads((path / 'metadata.json').read_text())
    rows = [{k: float(v) if k in ('policy_age_ms', 'mpc_solve_ms', 'wbc_solve_ms') else int(v)
             for k, v in row.items()} for row in read_csv(path / 'cycles.csv')]
    period = meta['period_ns']
    duration = (meta['finish_ns'] - meta['begin_ns']) / 1e9
    skipped = sum(row['skipped_slots'] for row in rows)
    expected_slots = max(0, math.ceil((meta['finish_ns'] - meta['begin_ns']) / period))
    completion_late = sum(row['end_ns'] > row['scheduled_ns'] + period for row in rows)
    result = {'metadata': meta, 'observed_duration_s': duration, 'executed_cycles': len(rows),
              'executed_hz': len(rows) / duration if duration else 0,
              'skipped_slots': skipped, 'planned_release_slots': expected_slots,
              'executed_completion_deadline_misses': completion_late,
              'executed_completion_miss_fraction': completion_late / len(rows) if rows else None,
              'missed_or_skipped_fraction': (completion_late + skipped) / (len(rows) + skipped) if rows else None,
              'period_ms': stats([(b['start_ns'] - a['start_ns']) / 1e6 for a, b in zip(rows, rows[1:])]),
              'release_lateness_ms': stats([(r['start_ns'] - r['scheduled_ns']) / 1e6 for r in rows]),
              'loop_work_ms': stats([(r['end_ns'] - r['start_ns']) / 1e6 for r in rows]),
              'io_ms': stats([(r['io_end_ns'] - r['io_start_ns']) / 1e6 for r in rows if r['io_start_ns']]),
              'rounds_with_motor_failures': sum(bool(r['failures']) for r in rows),
              'control': {}, 'motors': {}, 'legs': {}}
    for name, first, last in [('estimator_ms', 'estimator_start_ns', 'estimator_end_ns'),
                              ('observation_ms', 'estimator_end_ns', 'observation_end_ns'),
                              ('policy_evaluate_ms', 'observation_end_ns', 'policy_end_ns'),
                              ('wbc_wall_ms', 'policy_end_ns', 'wbc_end_ns'),
                              ('compute_ms', 'control_start_ns', 'control_end_ns')]:
        result['control'][name] = stats([(r[last] - r[first]) / 1e6 for r in rows if r[first] and r[last]])
    for name in ['estimate_valid', 'policy_valid', 'policy_fresh', 'wbc_executed', 'wbc_valid', 'solver_healthy']:
        result['control'][name + '_cycles'] = sum(r[name] for r in rows)
    result['control']['policy_age_ms'] = stats([r['policy_age_ms'] for r in rows if r['policy_age_ms'] >= 0])
    result['control']['mpc_solve_ms_new_policies'] = stats([r['mpc_solve_ms'] for r in rows if r['new_policy']])
    result['control']['policy_age_over_40ms_cycles'] = sum(r['policy_age_ms'] > 40 for r in rows)
    age_limit = meta.get('policy_age_limit_ms', 2000 / meta['mpc_hz'] if meta.get('mpc_hz') else 0)
    updates = [r for r in rows if r['new_policy']]
    result['control']['policy_age_limit_ms'] = age_limit
    result['control']['policy_age_over_limit_cycles'] = sum(r['policy_age_ms'] > age_limit for r in rows)
    result['control']['new_policy_count'] = len(updates)
    result['control']['new_policy_hz'] = len(updates) / duration if duration else 0
    result['control']['adopted_policy_interval_ms'] = stats([
        (b['policy_end_ns'] - a['policy_end_ns']) / 1e6 for a, b in zip(updates, updates[1:])])
    result['control']['isolated_real_imu_consumer'] = {
        'cycles': sum(r['imu_available'] for r in rows),
        'imu_read_to_estimator_ms': stats([(r['estimator_start_ns'] - r['imu_read_ns']) / 1e6
                                          for r in rows if r['imu_available'] and r['estimator_start_ns']]),
        'ahrs_read_to_estimator_ms': stats([(r['estimator_start_ns'] - r['ahrs_read_ns']) / 1e6
                                           for r in rows if r['imu_available'] and r['estimator_start_ns']]),
    }
    wbc_path = path / 'wbc.csv'
    if wbc_path.exists():
        wbc_rows = [{key: float(value) for key, value in row.items()} for row in read_csv(wbc_path)]
        compared = [r for r in wbc_rows if r['compared']]
        result['wbc'] = {'cycles': len(wbc_rows), 'paired_cycles': len(compared),
                         'paired_failures': sum(not r['comparison_ok'] for r in compared),
                         'paired_objective_relative_tolerance': 1e-5,
                         'paired_torque_absolute_tolerance_nm': 1e-3}
        for key in ['model_ms', 'constraints_ms', 'tasks_ms', 'assembly_ms', 'solver_ms',
                    'validation_ms', 'total_ms', 'n_wsr', 'equality_residual', 'inequality_violation']:
            result['wbc'][key] = stats([r[key] for r in wbc_rows])
        for key in ['cold_start', 'hot_start', 'cold_retry', 'hot_failed', 'validation_failed', 'failed']:
            result['wbc'][key + '_cycles'] = sum(bool(r[key]) for r in wbc_rows)
        for key in ['max_solution_difference', 'max_torque_difference', 'objective_relative_difference',
                    'reference_total_ms', 'reference_solver_ms', 'reference_n_wsr']:
            result['wbc'][key] = stats([r[key] for r in compared])
    motor_path = path / 'motors.csv'
    if motor_path.exists():
        motors = read_csv(motor_path)
        per_motor = {i: [] for i in range(12)}
        leg_rounds = {}
        unsafe = 0
        for item in motors:
            raw = bytes.fromhex(item.pop('tx_hex'))
            m = {key: int(value) for key, value in item.items()}
            unsafe += int(not valid_zero_packet(raw, m['motor_index']))
            per_motor[m['motor_index']].append(m)
            leg_rounds.setdefault((m['cycle'], m['motor_index'] // 3), []).append(m)
        shutdown = read_csv(path / 'shutdown.csv')
        for item in shutdown:
            raw = bytes.fromhex(item['tx_hex'])
            unsafe += int(not valid_zero_packet(raw, int(item['motor_index'])))
        result['captured_unsafe_packets'] = unsafe
        result['shutdown_zero_packets'] = len(shutdown)
        for i, samples in per_motor.items():
            successful = [m for m in samples if m['success']]
            result['motors'][str(i)] = {
                'leg': ['FR', 'FL', 'RR', 'RL'][i // 3], 'id': i % 3,
                'attempts': len(samples), 'success': len(successful),
                'no_reply': sum(not m['received'] for m in samples),
                'invalid_frame': sum(bool(m['received']) and not m['frame_valid'] for m in samples),
                'sdk_call_ms': stats([(m['sdk_return_ns'] - m['sdk_start_ns']) / 1e6 for m in samples if m['sdk_start_ns']]),
                'valid_feedback_gap_ms': stats([(b['parsed_or_recovery_end_ns'] - a['parsed_or_recovery_end_ns']) / 1e6
                                                for a, b in zip(successful, successful[1:])]),
                'shadow_write_to_next_sdk_call_ms': stats([(m['sdk_start_ns'] - m['previous_shadow_write_ns']) / 1e6
                                                          for m in samples if m['previous_shadow_write_ns'] and m['sdk_start_ns']]),
            }
        for leg in range(4):
            result['legs'][['FR', 'FL', 'RR', 'RL'][leg]] = stats([
                (max(m['parsed_or_recovery_end_ns'] for m in group) - min(m['sdk_start_ns'] for m in group)) / 1e6
                for (cycle, number), group in leg_rounds.items() if number == leg and len(group) == 3
                and all(m['sdk_start_ns'] and m['parsed_or_recovery_end_ns'] for m in group)])
    result['limits'] = [
        'All measurements use userspace CLOCK_MONOTONIC, not physical UART edges or motor sample/application timestamps.',
        'The finite test uses absolute releases and skips expired slots; it is not the ROS controller_manager scheduler.',
        'Control workload uses synthetic standing joints, real estimator/NMPC/WBC code and always discards numerical motor commands.',
        'Policy age follows production backend reception semantics, not independently measured sensor-to-solve age.',
        'Shadow write-to-send matches host ordering only; identical zero packets have no firmware command sequence echo.',
        'P99.9 is an empirical finite-sample quantile, not a worst-case or long-term real-time guarantee.',
    ]
    (path / 'summary.json').write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    print(json.dumps({key: result[key] for key in ['executed_cycles', 'executed_hz', 'skipped_slots',
                     'executed_completion_miss_fraction', 'io_ms', 'loop_work_ms']}, indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    analyze(parser.parse_args().directory)
