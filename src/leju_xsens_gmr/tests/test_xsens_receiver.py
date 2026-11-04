"""Packet-level migration contract tests; runnable with pytest or unittest.

Packets are built independently from the implementation's structs. Rotation
expectations use explicit matrices so inverse/conjugation errors cannot hide
behind matching quaternion helper implementations.
"""
import socket
import struct
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

from leju_xsens_gmr.xsens_receiver import (
    B, C, BODY_MAPPING, ElbowLandmarkCache, FragmentAssembler, PacketError,
    PoseFIFO, XsensFrameDecoder, XsensPacketParser, XsensPoseBuilder,
    XsensReceiver, matrix_quaternion, quaternion_matrix,
)


def packet(kind='02', payload=b'', count=0, counter=0x80, sample=0x12345678,
           char=0, time_code=0x89abcdef):
    return (b'MXTP' + kind.encode('ascii') + struct.pack('>I', sample)
            + bytes((counter, count)) + struct.pack('>I', time_code)
            + bytes((char, 23, 2, 4)) + struct.pack('>HH', 0x5678, len(payload))
            + payload)


def pose_record(sid, pos=None, quat=(1, 0, 0, 0)):
    if pos is None:
        pos = (sid, sid+0.25, sid+0.5)
    return struct.pack('>I7f', sid, *pos, *quat)


def full_pose(char=0, sample=1, overrides=None):
    overrides = overrides or {}
    return packet(payload=b''.join(overrides.get(i, pose_record(i))
                                   for i in range(1, 24)), count=23,
                  char=char, sample=sample)


def string(value):
    value = value.encode('latin1')
    return struct.pack('>i', len(value)) + value


def point(sid, xyz, name=None, pid=3, flags=0):
    if name is None:
        name = {9: 'pRightArmLatEpicondyle', 13: 'pLeftArmLatEpicondyle'}[sid]
    return (struct.pack('>HH', sid, pid) + string(name)
            + struct.pack('>I3f', flags, *xyz))


def scale(segments=(), points=(), counter=0x80, char=0, sample=1):
    payload = struct.pack('>I', len(segments))
    for name, origin in segments:
        payload += string(name) + struct.pack('>3f', *origin)
    payload += struct.pack('>I', len(points)) + b''.join(points)
    # Scale header item counts are not authoritative in C++.
    return packet('13', payload, count=199, counter=counter, char=char, sample=sample)


def calibrate(decoder, char=0, now=0., right=(1, 2, 3), left=(4, 5, 6)):
    decoder.feed(scale(segments=[('Pelvis', (0, 0, 0))], counter=0, char=char), now)
    decoder.feed(scale(points=[point(9, right), point(13, left)], char=char), now+0.001)


class ParserTests(unittest.TestCase):
    def test_header_all_fields_big_endian(self):
        h = XsensPacketParser.parse_header(packet(payload=pose_record(7), count=1,
                                                  counter=0x85, char=201))
        self.assertEqual((h.message_type, h.sample_counter, h.datagram_counter,
                          h.number_of_items, h.time_code, h.character_id,
                          h.body_segments, h.props, h.finger_tracking_segments,
                          h.reserved, h.payload_size),
                         ('02', 0x12345678, 0x85, 1, 0x89abcdef, 201, 23, 2, 4, 0x5678, 32))
        self.assertEqual(h.fragment_index, 5)
        self.assertTrue(h.is_last_fragment)

    def test_bad_header_lengths_magic_and_types(self):
        valid = packet()
        cases = [b'', valid[:23], valid+b'x', valid[:-1],
                 b'XXXX'+valid[4:], b'MXTPa2'+valid[6:],
                 b'MXTP\xff2'+valid[6:]]
        for data in cases:
            with self.subTest(data=data), self.assertRaises(PacketError):
                XsensPacketParser.parse_header(data)
        with self.assertRaises(PacketError):
            XsensPacketParser.parse_packet(packet('99'))

    def test_pose_counts_and_finite_validation(self):
        for data in [packet(payload=pose_record(1), count=0),
                     packet(payload=b'x'*31, count=1),
                     packet(payload=pose_record(1, (float('nan'), 0, 0)), count=1),
                     packet(payload=pose_record(1, quat=(float('inf'), 0, 0, 0)), count=1),
                     packet(payload=pose_record(1, quat=(0, 0, 0, 0)), count=1)]:
            with self.subTest(data=data), self.assertRaises(PacketError):
                XsensPacketParser.parse_packet(data)

    def test_raw_to_bvh_rotation_and_normalization(self):
        # 90 degrees about raw Z becomes 90 degrees about BVH Y.
        pose = XsensPacketParser.parse_packet(packet(
            payload=pose_record(1, (1, 2, 3), (2, 0, 0, 2)), count=1)).pose_quaternion[0]
        np.testing.assert_allclose(pose.position_m, [2, 3, 1])
        np.testing.assert_allclose(quaternion_matrix(pose.orientation),
                                   [[0, 0, 1], [0, 1, 0], [-1, 0, 0]], atol=1e-14)
        self.assertAlmostEqual(np.linalg.norm(pose.orientation), 1.)

    def test_scale_wire_layout_byte_strings_flags(self):
        p = XsensPacketParser.parse_packet(scale(
            segments=[('A\xff\x00', (1, 2, 3))],
            points=[point(0x1234, (4, 5, 6), name='other', pid=0x5678, flags=0x89abcdef)]))
        self.assertEqual(p.scale_segments[0].name, 'A\xff\x00')
        np.testing.assert_equal(p.scale_segments[0].origin_m, [2, 3, 1])
        t = p.scale_points[0]
        self.assertEqual((t.segment_id, t.point_id, t.flags), (0x1234, 0x5678, 0x89abcdef))
        np.testing.assert_equal(t.position_m, [5, 6, 4])

    def test_scale_malformed_counts_strings_trailing_and_nonfinite(self):
        bad = [struct.pack('>I', 0),  # missing mandatory point count
               struct.pack('>I', 0xffffffff),
               struct.pack('>Ii', 1, -1)+b'\0'*16,
               struct.pack('>Ii', 1, 100)+b'\0'*16,
               struct.pack('>II', 0, 0xffffffff),
               struct.pack('>II', 0, 0)+b'x']
        for payload in bad:
            with self.subTest(payload=payload), self.assertRaises(PacketError):
                XsensPacketParser.parse_packet(packet('13', payload))
        for data in [scale(segments=[('x', (0, float('inf'), 0))]),
                     scale(points=[point(9, (float('nan'), 0, 0))])]:
            with self.assertRaises(PacketError):
                XsensPacketParser.parse_packet(data)
        valid = scale(segments=[('x', (1, 2, 3))], points=[point(9, (1, 2, 3))])
        for end in range(24, len(valid)):
            with self.subTest(end=end), self.assertRaises(PacketError):
                XsensPacketParser.parse_packet(packet('13', valid[24:end]))


class FragmentTests(unittest.TestCase):
    def test_out_of_order_duplicate_and_key_isolation(self):
        assembler = FragmentAssembler()
        tail = packet(payload=pose_record(2), count=1, counter=0x81)
        head = packet(payload=pose_record(1), count=1, counter=0)
        self.assertIsNone(assembler.add_datagram(tail, 0))
        self.assertIsNone(assembler.add_datagram(tail, .1))
        self.assertIsNone(assembler.add_datagram(packet(payload=pose_record(7), count=1,
                                                      counter=0, char=2), .2))
        self.assertIsNone(assembler.add_datagram(packet(payload=pose_record(7), count=1,
                                                      counter=0, sample=9), .2))
        assembled = assembler.add_datagram(head, .3)
        p = XsensPacketParser.parse_packet(assembled)
        self.assertEqual([x.segment_id for x in p.pose_quaternion], [1, 2])
        self.assertEqual(p.header.datagram_counter, 0x80)
        self.assertEqual(p.header.number_of_items, 2)
        self.assertEqual(len(assembler.pending), 2)

    def test_scale_datagrams_bypass_ordinary_assembly(self):
        assembler = FragmentAssembler()
        for counter in (0, 1, 0x80, 0x82):
            data = scale(counter=counter)
            self.assertEqual(assembler.add_datagram(data, 0), data)
        self.assertFalse(assembler.pending)

    def test_missing_fragment_timeout_absolute_lifetime_and_key_bound(self):
        assembler = FragmentAssembler(timeout=1, max_pending=2)
        head = packet(counter=0)
        assembler.add_datagram(head, 0)
        assembler.add_datagram(head, .9)
        assembler.cleanup(1.01)  # duplicate did not extend absolute lifetime
        self.assertFalse(assembler.pending)
        for sample in range(3):
            assembler.add_datagram(packet(counter=0, sample=sample), 2)
        self.assertEqual(len(assembler.pending), 2)
        self.assertNotIn((0, '02', 0), assembler.pending)
        assembler.add_datagram(packet(counter=0x82, sample=1), 2.1)
        self.assertEqual(len(assembler.pending), 2)  # missing index 1
        assembler.cleanup(4)
        self.assertFalse(assembler.pending)

    def test_reassembled_limits_and_fragment_127(self):
        assembler = FragmentAssembler()
        assembler.add_datagram(packet(payload=b'x'*40000, counter=0), 0)
        with self.assertRaises(PacketError):
            assembler.add_datagram(packet(payload=b'x'*30000, counter=0x81), 0)
        assembler.add_datagram(packet(count=200, counter=0), 0)
        with self.assertRaises(PacketError):
            assembler.add_datagram(packet(count=100, counter=0x81), 0)
        for index in range(127):
            self.assertIsNone(assembler.add_datagram(packet(counter=index), 0))
        result = assembler.add_datagram(packet(counter=255), 0)
        self.assertEqual(XsensPacketParser.parse_header(result).datagram_counter, 128)
        self.assertFalse(assembler.pending)


class ScaleAndConversionTests(unittest.TestCase):
    def test_scale_required_character_filter_and_cache_isolation(self):
        decoder = XsensFrameDecoder()
        self.assertIsNone(decoder.feed(full_pose(), 0))
        calibrate(decoder, char=1)
        self.assertIsNone(decoder.scale_cache.get_offsets(1))
        calibrate(decoder)
        self.assertIsNone(decoder.feed(full_pose(char=1), .1))
        self.assertEqual(len(decoder.feed(full_pose(), .1)), 20)
        multi = XsensFrameDecoder(-1)
        calibrate(multi, char=1)
        self.assertIsNone(multi.feed(full_pose(), .1))
        self.assertIsNotNone(multi.feed(full_pose(char=1), .1))

    def test_atomic_refresh_rejection_unchanged_and_copy_ownership(self):
        cache = ElbowLandmarkCache()
        def consume(data, now=0):
            return cache.consume(XsensPacketParser.parse_packet(data), now)
        start = scale(segments=[('Pelvis', (0, 0, 0))], counter=0)
        self.assertEqual(consume(scale(points=[point(9, (1, 2, 3))])), 'ignored')
        self.assertEqual(consume(start), 'collecting')
        end = scale(points=[point(9, (1, 2, 3)), point(13, (4, 5, 6))])
        self.assertEqual(consume(end), 'ready')
        old = cache.get_offsets(0)
        consume(start)
        consume(scale(points=[point(9, (9, 9, 9))], counter=1))
        np.testing.assert_equal(cache.get_offsets(0)[9], old[9])
        self.assertEqual(consume(scale(points=[])), 'rejected')
        np.testing.assert_equal(cache.get_offsets(0)[9], old[9])
        consume(start)
        self.assertEqual(consume(end), 'unchanged')
        consume(start)
        self.assertEqual(consume(scale(points=[point(9, (9, 8, 7)),
                                               point(13, (6, 5, 4))])), 'updated')
        snapshot = cache.get_offsets(0)
        snapshot[9][:] = 0
        np.testing.assert_equal(cache.get_offsets(0)[9], [8, 7, 9])

    def test_mismatch_duplicate_incomplete_bad_start_and_timeout(self):
        for points in ([point(9, (1, 2, 3), name='pLeftArmLatEpicondyle'),
                        point(13, (1, 2, 3))],
                       [point(9, (1, 2, 3)), point(9, (2, 2, 3)), point(13, (1, 2, 3))],
                       [point(9, (1, 2, 3))],
                       [point(9, (1, 2, 3), pid=4), point(13, (1, 2, 3))]):
            with self.subTest(points=points):
                decoder = XsensFrameDecoder()
                calibrate(decoder)
                old = decoder.scale_cache.get_offsets(0)
                decoder.feed(scale(segments=[('x', (0, 0, 0))], counter=0), .1)
                decoder.feed(scale(points=points), .2)
                for sid in (9, 13):
                    np.testing.assert_equal(decoder.scale_cache.get_offsets(0)[sid], old[sid])
        cache = ElbowLandmarkCache()
        self.assertEqual(cache.consume(XsensPacketParser.parse_packet(scale(
            segments=[('x', (0, 0, 0))], counter=0x80)), 0), 'rejected')
        cache.consume(XsensPacketParser.parse_packet(scale(
            segments=[('x', (0, 0, 0))], counter=0)), 0)
        self.assertEqual(cache.consume(XsensPacketParser.parse_packet(scale(
            points=[point(9, (1, 2, 3)), point(13, (1, 2, 3))])), 1.1), 'ignored')
        self.assertIsNone(cache.get_offsets(0))

    def test_malformed_scale_cancels_candidate_preserves_committed(self):
        decoder = XsensFrameDecoder()
        calibrate(decoder)
        old = decoder.scale_cache.get_offsets(0)
        for bad in (scale(points=[point(9, (float('nan'), 1, 2))]), scale()+b'x'):
            decoder.feed(scale(segments=[('x', (0, 0, 0))], counter=0), .1)
            with self.assertRaises(PacketError):
                decoder.feed(bad, .2)
            decoder.feed(scale(points=[point(9, (9, 9, 9)), point(13, (9, 9, 9))]), .3)
            np.testing.assert_equal(decoder.scale_cache.get_offsets(0)[9], old[9])

    def test_gmr_exact_names_and_canonical_left_multiplication(self):
        decoder = XsensFrameDecoder()
        calibrate(decoder)
        frame = decoder.feed(full_pose(), .1)
        expected_names = ['Hips', 'Spine', 'Neck', 'Head', 'LeftShoulder', 'LeftArm',
                          'LeftForeArm', 'LeftHand', 'RightShoulder', 'RightArm',
                          'RightForeArm', 'RightHand', 'LeftUpLeg', 'LeftLeg',
                          'LeftFoot', 'LeftToeBase', 'RightUpLeg', 'RightLeg',
                          'RightFoot', 'RightToeBase']
        self.assertEqual(list(frame), expected_names)
        for name, sid, parent in BODY_MAPPING:
            p, q = frame[name]
            self.assertEqual(p.dtype, np.float64)
            self.assertEqual(q.shape, (4,))
            np.testing.assert_allclose(quaternion_matrix(q), C, atol=1e-14)
            if parent is None:
                np.testing.assert_equal(p, [sid, sid+.25, sid+.5])

    def test_elbow_upper_arm_offset_forearm_orientation_and_noncommuting_rotation(self):
        decoder = XsensFrameDecoder()
        calibrate(decoder, right=(1, 2, 3), left=(4, 5, 6))
        data = full_pose(overrides={
            9: pose_record(9, (10, 20, 30), (1, 0, 0, 1)),  # raw Z +90
            13: pose_record(13, (40, 50, 60), (1, 1, 0, 0)),  # raw X +90
            10: pose_record(10, (999, 999, 999), (0, 0, 1, 0)),  # raw Y 180
        })
        frame = decoder.feed(data, .1)
        np.testing.assert_allclose(frame['RightForeArm'][0], [8, 21, 33], atol=1e-12)
        np.testing.assert_allclose(frame['LeftForeArm'][0], [44, 44, 65], atol=1e-12)
        raw_forearm = np.diag([-1., 1., -1.])
        np.testing.assert_allclose(quaternion_matrix(frame['RightForeArm'][1]),
                                   raw_forearm @ C, atol=1e-14)
        self.assertFalse(np.allclose(raw_forearm @ C, C @ raw_forearm))

    def test_missing_bones_pelvis_zero_alias_and_duplicate_last_wins(self):
        decoder = XsensFrameDecoder()
        calibrate(decoder)
        with self.assertRaises(PacketError):
            decoder.feed(packet(payload=pose_record(1), count=1), .1)
        frame = decoder.feed(full_pose(overrides={1: pose_record(0)}), .1)
        np.testing.assert_equal(frame['Hips'][0], [0, .25, .5])
        raw = full_pose()[24:] + pose_record(1, (77, 88, 99))
        frame = decoder.feed(packet(payload=raw, count=24), .1)
        np.testing.assert_equal(frame['Hips'][0], [77, 88, 99])
        with self.assertRaises(PacketError):
            XsensPoseBuilder.build(XsensPacketParser.parse_packet(full_pose()), None)

    def test_scale_update_does_not_modify_already_decoded_frame(self):
        decoder = XsensFrameDecoder()
        calibrate(decoder)
        old = decoder.feed(full_pose(), .1)
        old_pos = old['RightForeArm'][0].copy()
        calibrate(decoder, now=.2, right=(8, 8, 8))
        new = decoder.feed(full_pose(), .3)
        np.testing.assert_equal(old['RightForeArm'][0], old_pos)
        self.assertFalse(np.array_equal(new['RightForeArm'][0], old_pos))

    def test_matrix_quaternion_all_diagonal_branches(self):
        for diagonal in ([1, -1, -1], [-1, 1, -1], [-1, -1, 1]):
            r = np.diag(diagonal)
            np.testing.assert_allclose(quaternion_matrix(matrix_quaternion(r)), r)


class QueueTests(unittest.TestCase):
    def test_fifo_prefill_overflow_hold_and_original_timestamp(self):
        q = PoseFIFO(2, 3, .2)
        q.push({'id': 1}, 1000., 0.)
        self.assertIsNone(q.pop(.01))
        for i in range(2, 5):
            q.push({'id': i}, 1000.+i, .01*i)
        self.assertEqual(len(q.queue), 3)
        self.assertEqual(q.pop(.05).frame_gmr['id'], 2)
        self.assertEqual(q.pop(.06).frame_gmr['id'], 3)
        last = q.pop(.07)
        self.assertEqual(last.frame_gmr['id'], 4)
        self.assertIs(q.pop(.08), last)
        self.assertFalse(q.prefill_ready)
        q.push({'id': 5}, 1005., .09)
        self.assertIs(q.pop(.10), last)  # refilling, preserve original capture
        self.assertEqual(last.capture_time, 1004.)
        self.assertIsNone(q.pop(.25))  # newer arrival cannot refresh old held frame
        q.push({'id': 6}, 1006., .26)
        self.assertEqual(q.pop(.27).frame_gmr['id'], 5)
        self.assertEqual(q.pop(.28).frame_gmr['id'], 6)
        self.assertIsNone(q.pop(.5))

    def test_stale_prefill_expiration_and_refill(self):
        q = PoseFIFO(2, 3, .2)
        q.push({}, 1, 0)
        self.assertIsNone(q.pop(.21))
        self.assertEqual(len(q.queue), 0)
        q.push({}, 2, .3)
        self.assertIsNone(q.pop(.31))
        q.push({}, 3, .32)
        self.assertEqual(q.pop(.33).capture_time, 2)

    def test_invalid_configuration(self):
        for args in ({'publish_hz': 0}, {'publish_hz': float('nan')},
                     {'queue_target_depth': 16}, {'queue_max_size': 0},
                     {'queue_target_depth': 1.5}, {'stale_timeout': -1},
                     {'expected_character_id': 256}, {'expected_character_id': -2},
                     {'port': -1}, {'frame_callback': 123}):
            with self.subTest(args=args), self.assertRaises(ValueError):
                XsensReceiver(**args)


class ReceiverTests(unittest.TestCase):
    def test_loopback_fragments_callback_holds_stale_and_restart(self):
        received = []
        delivered = threading.Event()
        lock = threading.Lock()
        def callback(frame, capture):
            with lock:
                received.append((frame['Hips'][0].copy(), capture, time.monotonic()))
                if len(received) >= 3:
                    delivered.set()
            frame['Hips'][0][:] = -999  # subsequent holds must own separate arrays
        receiver = XsensReceiver('127.0.0.1', 0, callback, publish_hz=100,
                                 queue_target_depth=1, stale_timeout=.12)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            receiver.start()
            receiver.start()
            address = ('127.0.0.1', receiver.port)
            sender.sendto(b'invalid', address)
            sender.sendto(full_pose(char=1), address)
            sender.sendto(scale(segments=[('Pelvis', (0, 0, 0))], counter=0), address)
            sender.sendto(scale(points=[point(9, (1, 2, 3)), point(13, (4, 5, 6))]), address)
            before = time.time()
            payload = full_pose()[24:]
            sender.sendto(packet(payload=payload[32*10:], count=13, counter=0x81), address)
            sender.sendto(packet(payload=payload[:32*10], count=10, counter=0), address)
            self.assertTrue(delivered.wait(2), 'no loopback callbacks')
            self.assertEqual(receiver.total_frames, 1)
            # Wait past expiration without producing any new inputs.
            threading.Event().wait(.18)
            with lock:
                snapshot = list(received)
            threading.Event().wait(.05)
            self.assertEqual(len(received), len(snapshot))
            for pos, capture, _ in snapshot:
                np.testing.assert_equal(pos, [1, 1.25, 1.5])
                self.assertEqual(capture, snapshot[0][1])
                self.assertGreaterEqual(capture, before)
                self.assertLessEqual(capture, time.time())
            self.assertLess(snapshot[-1][2]-snapshot[0][2], .15)
            receiver.stop()
            receiver.stop()
            self.assertTrue(all(not t.is_alive() for t in receiver._threads))
            receiver.start()
            self.assertIsNone(receiver._decoder.scale_cache.get_offsets(0))
            self.assertEqual(receiver.total_frames, 1)
        finally:
            receiver.stop()
            sender.close()

    def test_callback_failure_isolation_and_stop_from_callback(self):
        calls = []
        done = threading.Event()
        def callback(frame, capture):
            calls.append(capture)
            if len(calls) == 1:
                raise RuntimeError('deliberate callback failure')
            receiver.stop()
            done.set()
        receiver = XsensReceiver('127.0.0.1', 0, callback, queue_target_depth=1)
        try:
            with patch('leju_xsens_gmr.xsens_receiver.log.exception') as logged:
                receiver.start()
                # Populate using the pure pipeline, testing publication/lifecycle separately.
                decoder = XsensFrameDecoder()
                calibrate(decoder)
                with receiver._lock:
                    receiver._fifo.push(decoder.feed(full_pose(), .1), 123., time.monotonic())
                self.assertTrue(done.wait(2))
                logged.assert_called_once()
            self.assertEqual(calls, [123., 123.])
        finally:
            receiver.stop()

    def test_bind_failure_is_synchronous_and_stop_before_start(self):
        receiver = XsensReceiver('127.0.0.1', 0)
        receiver.stop()
        blocker = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            blocker.bind(('127.0.0.1', 0))
            receiver = XsensReceiver('127.0.0.1', blocker.getsockname()[1])
            with self.assertRaises(OSError):
                receiver.start()
            self.assertFalse(receiver.running)
        finally:
            blocker.close()
            receiver.stop()


if __name__ == '__main__':
    unittest.main()
