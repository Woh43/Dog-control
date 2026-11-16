#!/usr/bin/env python3
"""Synthetic binary-schema tests; no ROS runtime, SDK loading or hardware."""
import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import unittest
import numpy as np
SCRIPT = Path(__file__).resolve().parents[3]/'tools/analyze_cycle_trace.py'
SPEC = importlib.util.spec_from_file_location('analyze_cycle_trace', SCRIPT)
M = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(M)


def write_trace(path, records, type_id=1, footer=True, flags=1, reason=1, **changes):
    size = records.dtype.itemsize
    values = dict(version=1, header_size=64, record_size=size, type_id=type_id, endian=0x01020304,
                  wall=1, mono=1, cap=64*1024*1024, ring=64, alignment=8, clock=1, flags=flags)
    values.update(changes)
    head = M.HEADER.pack(b'DGCYCL01', *[values[k] for k in ('version','header_size','record_size','type_id','endian','wall','mono','cap','ring','alignment','clock','flags')])
    raw = head+records.tobytes()
    if footer: raw += M.FOOTER.pack(b'DGCEND01', 1, reason, len(records), len(records), 0, 0, len(raw)+64, 0)
    path.write_bytes(raw)


def tx_packet(motor_id=0, mode=1, values=(0,0,0,0,0)):
    raw = b'\xfe\xee'+bytes([motor_id | mode << 4])+struct.pack('<hhiHH', *values)
    return raw+struct.pack('<H', M.crc16(raw))


def controller(n=2):
    a = np.zeros(n, dtype=M.CONTROLLER_DTYPE)
    for i in range(n):
        a[i]['cycle']=i+1; a[i]['manager_time_ns']=100+i*4_000_000
        a[i]['start_ns']=1_000_000+i*4_000_000; a[i]['end_ns']=a[i]['start_ns']+100_000
        a[i]['policy_sequence']=7; a[i]['mode']=3; a[i]['policy_mode']=15
        a[i]['flags']=1|2|4|8|16|32|512; a[i]['period_s']=.004; a[i]['query_time_s']=i*.004
        a[i]['feedback']['valid']=1; a[i]['nominal_velocity']=.5
        a[i]['nominal_position']=i*.002
    return a


class SchemaTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup); self.root=Path(self.temp.name)
    def test_sizes_and_clean_empty_rotation_segment(self):
        self.assertEqual(M.CONTROLLER_DTYPE.itemsize,3240); self.assertEqual(M.HARDWARE_DTYPE.itemsize,6432)
        path=self.root/'controller.00000.bin'; write_trace(path,controller(0),flags=7)
        records,q=M.parse_file(path); self.assertEqual(len(records),0); self.assertTrue(q['cleanly_closed'])
    def test_reject_version_size_type_endian_and_alignment(self):
        for changes in ({'version':2},{'record_size':8},{'type_id':8},{'endian':0x04030201},{'alignment':4}):
            with self.subTest(changes=changes):
                path=self.root/'bad.bin'; write_trace(path,controller(1),**changes)
                with self.assertRaises(ValueError):M.parse_file(path)
    def test_missing_footer_and_partial_record_not_accepted(self):
        path=self.root/'controller.bin'; write_trace(path,controller(2),footer=False)
        path.write_bytes(path.read_bytes()+b'partial')
        a,q=M.parse_file(path); self.assertEqual(len(a),2); self.assertEqual(q['trailing_bytes_excluded'],7)
        self.assertIn('missing_footer_unclean',q['issues']); self.assertFalse(q['cleanly_closed'])
    def test_truncated_footer_does_not_become_a_record(self):
        path=self.root/'controller.bin'; write_trace(path,controller(1)); path.write_bytes(path.read_bytes()[:-11])
        a,q=M.parse_file(path); self.assertEqual(len(a),1); self.assertEqual(q['trailing_bytes_excluded'],53)
        self.assertIn('missing_footer_unclean',q['issues'])
    def test_nonfinite_counted_but_excluded_from_stats(self):
        a=controller(1); a['output_command']['position'][0,5]=np.nan
        path=self.root/'controller.bin'; write_trace(path,a); r,q=M.parse_file(path)
        self.assertEqual(q['nonfinite_fields']['output_command.position'],1)
        result=M.controller_analysis(r); self.assertEqual(result['by_mode']['MPC_STANCE']['P_request_nm'][M.NAMES[5]]['samples'],0)
    def test_next_read_join_by_exact_source_stamp_not_rows(self):
        c=controller(2); h=np.zeros(3,dtype=M.HARDWARE_DTYPE)
        # first read has no preceding write; read3 transmits controller1, not row3.
        h[1]['read_sequence']=2;h[1]['flags']=2|4|16;h[1]['source_write_sequence']=1;h[1]['source_write_time_ns']=c[1]['manager_time_ns']
        h[2]['read_sequence']=3;h[2]['flags']=2|4|16;h[2]['source_write_sequence']=2;h[2]['source_write_time_ns']=c[0]['manager_time_ns']
        result,pairs=M.join_records(c,h);self.assertEqual(pairs,[(1,1),(0,2)]);self.assertEqual(result['counts']['matched'],2)
        # A failed/cleared write is not a match to an old active controller command.
        h[1]['flags']=int(h[1]['flags'])|512; result,pairs=M.join_records(c,h);self.assertEqual(pairs,[(0,2)]);self.assertEqual(result['counts']['cleared_or_estop'],1)
    def test_duplicate_stamps_are_ambiguous_and_zero_time_is_allowed(self):
        c=controller(2);c['manager_time_ns']=0;h=np.zeros(1,dtype=M.HARDWARE_DTYPE);h['flags']=2|4|16;h['source_write_sequence']=1
        result,pairs=M.join_records(c,h);self.assertFalse(pairs);self.assertEqual(result['counts']['ambiguous_duplicate_controller_time'],1)
        result,pairs=M.join_records(c[:1],h);self.assertEqual(pairs,[(0,0)])
    def test_PD_decomposition_and_invalid_feedback(self):
        c=controller(1);w=c['output_command'];w['position']=.2;w['velocity']=.4;w['effort']=3;w['kp']=10;w['kd']=2
        c['feedback']['position']=.1;c['feedback']['velocity']=.1;c['feedback']['valid'][0,5]=0
        s=M.controller_analysis(c)['by_mode']['MPC_STANCE'];name=M.NAMES[0]
        self.assertAlmostEqual(s['P_request_nm'][name]['mean'],1);self.assertAlmostEqual(s['D_request_nm'][name]['mean'],.6)
        self.assertAlmostEqual(s['D_reference_Kd_dqdes_nm'][name]['mean'],.8);self.assertAlmostEqual(s['D_damping_minus_Kd_dq_nm'][name]['mean'],-.2)
        self.assertAlmostEqual(s['same_sample_total_request_nm'][name]['mean'],4.6);self.assertEqual(s['same_sample_total_request_nm'][M.NAMES[5]]['samples'],0)
    def test_same_query_jump_only_changed_policy_and_valid_old_sample(self):
        c=controller(3);c[1]['policy_sequence']=8;c[2]['policy_sequence']=8
        c[1]['flags']=int(c[1]['flags'])|1024;c[2]['flags']=int(c[2]['flags'])|1024;c[1]['nominal_position']=5;c[1]['previous_nominal_position']=4.75
        c[2]['previous_nominal_position']=-99
        s=M.controller_analysis(c);self.assertEqual(s['same_query_new_minus_old_nominal_jump_rad'][M.NAMES[0]]['samples'],1)
        self.assertAlmostEqual(s['same_query_new_minus_old_nominal_jump_rad'][M.NAMES[0]]['mean'],.25)
    def test_qdot_uses_query_time_and_excludes_drop_gap(self):
        c=controller(3);c['manager_time_ns']=[100,900,901]
        s=M.controller_analysis(c);self.assertEqual(s['qdot_comparison_pairs'],2)
        self.assertAlmostEqual(s['same_policy_no_event_qdot_minus_mean_dq_rad_s'][M.NAMES[0]]['max'],0)
        c[2]['cycle']=5;s=M.controller_analysis(c);self.assertEqual(s['qdot_comparison_pairs'],1)
    def test_GO_exact_scales_crc_and_zero_gain_packet(self):
        self.assertEqual(M.crc16(b'123456789'),0x2189)
        raw=tx_packet(2,1,(256,512,32768,1280,0));d=M.decode_tx(raw)
        self.assertEqual(d['id'],2);self.assertEqual(d['mode'],1);self.assertTrue(d['crc_ok'])
        self.assertEqual(d['position'],6.2832);self.assertEqual(d['velocity'],12.5664);self.assertEqual(d['effort'],1);self.assertEqual(d['kp'],1)
        zero=M.decode_tx(tx_packet(0,0));self.assertTrue(zero['crc_ok']);self.assertEqual(zero['position'],0);self.assertEqual(zero['kp'],0)
        self.assertFalse(M.decode_tx(raw[:-1]+bytes([raw[-1]^1]))['crc_ok'])
    def test_invalid_RX_is_not_physical_feedback_and_internal_round_is_not_joined(self):
        h=np.zeros(1,dtype=M.HARDWARE_DTYPE);h['read_sequence']=1;h['flags']=2|4|16;h['source_write_sequence']=1
        h['source_write_steady_ns']=10;h['sdk']['round_count']=1;h['sdk']['rounds'][0,0]['kind']=2
        m=h['sdk']['rounds'][0,0]['motors'][0];m['flags']=1|2|256;m['sdk_start_ns']=20;m['sdk_return_ns']=30;m['parsed_ns']=40;m['tx']=np.frombuffer(tx_packet(0,0),dtype='u1');m['rx'][0]=1
        s,_=M.packet_analysis(h,4);self.assertEqual(s['counts']['rx_bad_crc'],1);self.assertEqual(s['counts']['producer_frame_valid_but_crc_bad'],1)
        self.assertEqual(s['counts'].get('successful_feedback_packets',0),0);self.assertEqual(s['pending_write_to_sdk_start_ms']['samples'],0)
    def test_pending_requires_ordinary_compatible_caller_and_packet(self):
        h=np.zeros(1,dtype=M.HARDWARE_DTYPE);h['read_sequence']=1;h['flags']=2|4|16;h['source_write_sequence']=1
        h['source_write_steady_ns']=100;h['sdk']['round_count']=1;h['sdk']['rounds'][0,0]['kind']=1
        m=h['sdk']['rounds'][0,0]['motors'][0];m['flags']=1;m['sdk_start_ns']=200;m['sdk_return_ns']=300
        m['gear']=6.33;m['direction']=1;m['tx']=np.frombuffer(tx_packet(0,0),dtype='u1')
        result,_=M.packet_analysis(h,4);self.assertEqual(result['pending_write_to_sdk_start_ms']['samples'],1)
        h['sdk_command']['position'][0,0]=1
        result,_=M.packet_analysis(h,4);self.assertEqual(result['pending_write_to_sdk_start_ms']['samples'],0)
        self.assertEqual(result['counts']['ordinary_packet_not_eligible_for_normal_pending'],1)

    def test_success_flag_cannot_make_bad_rx_physical(self):
        h=np.zeros(1,dtype=M.HARDWARE_DTYPE);h['sdk']['round_count']=1
        m=h['sdk']['rounds'][0,0]['motors'][0];m['flags']=1|2|256|512
        m['tx']=np.frombuffer(tx_packet(0,1),dtype='u1')
        raw=b'\xfd\xee'+bytes([0x11])+struct.pack('<hhib',0,0,0,25)+bytes(2)
        raw+=struct.pack('<H',M.crc16(raw));m['rx']=np.frombuffer(raw,dtype='u1')
        result,_=M.packet_analysis(h,4)
        self.assertEqual(result['counts']['producer_success_inconsistent_rx_fields'],1)
        self.assertEqual(result['counts'].get('successful_feedback_packets',0),0)

    def causal_fixture(self, n=3):
        c=controller(n);h=np.zeros(1,dtype=M.HARDWARE_DTYPE)
        h['flags']=2|4|16;h['source_write_sequence']=892;h['source_write_time_ns']=987654321
        h['source_write_steady_ns']=int(c[0]['end_ns'])+50_000
        return c,h

    def test_causal_join_handles_CM_independently_sampled_ROS_stamps(self):
        c,h=self.causal_fixture();result,pairs=M.join_records(c,h)
        self.assertEqual(pairs,[(0,0)])
        self.assertEqual(result['methods']['bounded_monotonic_interval_and_finite_request'],1)
        self.assertEqual(result['matched_source_write_minus_controller_cycle_offsets'],{'891':1})

    def test_causal_requires_all_five_finite_equal_request_fields(self):
        for field in M.COMMAND.names:
            for bad in (1.,float('nan')):
                with self.subTest(field=field,bad=bad):
                    c,h=self.causal_fixture();h['requested_command'][field][0,5]=bad
                    result,pairs=M.join_records(c,h);self.assertFalse(pairs)
                    self.assertEqual(result['counts']['causal_field_mismatch_or_nonfinite'],1)

    def test_causal_never_bridges_missing_controller_cycle(self):
        c,h=self.causal_fixture();c[1]['cycle']=3;c[2]['cycle']=4
        result,pairs=M.join_records(c,h);self.assertFalse(pairs)
        self.assertEqual(result['counts']['causal_missing_controller_cycle'],1)

    def test_causal_rejects_overlap_and_unbounded_last_interval(self):
        c,h=self.causal_fixture();c['start_ns']=[100,300,400];c['end_ns']=[200,250,500]
        h['source_write_steady_ns']=275
        result,pairs=M.join_records(c,h);self.assertFalse(pairs)
        self.assertEqual(result['counts']['causal_ambiguous_overlap'],1)
        c,h=self.causal_fixture();h['source_write_steady_ns']=int(c[-1]['end_ns'])+1
        result,pairs=M.join_records(c,h);self.assertFalse(pairs)
        self.assertEqual(result['counts']['last_interval_or_after_controller_coverage'],1)

    def test_causal_lower_boundary_inclusive_upper_boundary_exclusive(self):
        c,h=self.causal_fixture();h['source_write_steady_ns']=c[0]['end_ns']
        self.assertEqual(M.join_records(c,h)[1],[(0,0)])
        h['source_write_steady_ns']=c[1]['start_ns']
        result,pairs=M.join_records(c,h);self.assertFalse(pairs)
        self.assertEqual(result['counts']['causal_gap'],1)

    def test_exact_ROS_match_also_rejects_nonfinite_or_unequal_fields(self):
        c,h=self.causal_fixture();h['source_write_time_ns']=c[0]['manager_time_ns'];h['requested_command']['kd'][0,0]=float('nan')
        result,pairs=M.join_records(c,h);self.assertFalse(pairs)
        self.assertEqual(result['counts']['exact_ros_field_mismatch_or_nonfinite'],1)

    def test_missing_segments_separate_from_ring_drop_and_JSON_finite(self):
        write_trace(self.root/'controller.00000.bin',controller(1),flags=7,reason=2)
        write_trace(self.root/'controller.00003.bin',controller(1),flags=7)
        s=M.analyze(self.root);self.assertTrue(s['sampling_quality']['retained_window_has_segment_gaps'])
        self.assertFalse(s['sampling_quality']['ring_dropped_or_discarded']);json.dumps(s,allow_nan=False)

if __name__=='__main__':unittest.main()
