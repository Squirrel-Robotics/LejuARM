"""Cross-process controls use DDS only; body input and output stay independent."""

from types import SimpleNamespace
from dataclasses import fields

import pytest

from leju_gmr_common.dds_io import QuestJoysticks, JoystickSubscriber, DdsPublishers


def joy(stamp=100., **kwargs):
    data = {f.name: False if 'pressed' in f.name or 'touched' in f.name else 0.
            for f in fields(QuestJoysticks)}
    data.update(header_sec=int(stamp), header_nanosec=int((stamp % 1) * 1e9))
    data.update(kwargs)
    return QuestJoysticks(**data)


def test_subscriber_filters_stale_disposed_and_future_controls():
    fresh = joy(100.)
    subscriber = JoystickSubscriber.__new__(JoystickSubscriber)
    subscriber.max_age = .5
    subscriber.reader = SimpleNamespace(take=lambda N: [object(), joy(98.), joy(102.), fresh])
    assert subscriber.take_fresh(now=100.1) == [fresh]


def test_xsens_consumes_controls_without_republishing(monkeypatch):
    from leju_xsens_gmr.main import XsensGmrApp
    from leju_gmr_common.app import MotionGmrApp
    import threading
    app = XsensGmrApp.__new__(XsensGmrApp)
    app._bvh_lock = threading.Lock()
    app._bvh_pending = False
    app.calibrated = True
    app.control_reader = SimpleNamespace(take_fresh=lambda: [joy(left_second_button_pressed=True,
                                                                 right_trigger=1.)])
    app._prev_btn = {'A': False, 'B': False, 'X': False, 'Y': False}
    app.trigger_threshold = .5
    frozen = []
    app._freeze_stream = lambda: frozen.append(True)
    monkeypatch.setattr(MotionGmrApp, '_tick', lambda self: None)
    # No DDS publisher or Pico receiver is needed to interpret control messages.
    app._tick()
    assert frozen == [True]


def test_joystick_process_creates_no_body_reference_writer(monkeypatch):
    from leju_gmr_common import dds_io
    created = []
    monkeypatch.setattr(dds_io, 'DomainParticipant', lambda domain: object())
    monkeypatch.setattr(dds_io, 'Topic', lambda participant, name, cls: name)
    monkeypatch.setattr(dds_io, 'DataWriter', lambda participant, topic: created.append(topic))
    dds_io.DdsPublishers(heft_topic=None, joy_topic='/controls', bone_topic=None, head_topic=None)
    assert created == ['/controls']


def test_pico_joy_only_skips_body_decoding():
    from leju_pico_gmr.pico_receiver import PicoReceiver
    from leju_pico_gmr.protos import body_tracking_extended_pb2 as proto
    receiver = PicoReceiver.__new__(PicoReceiver)
    receiver.receive_body = False
    receiver._parse_full_body_to_matrix = lambda msg: pytest.fail('Body decoded by joystick-only process')
    msg = proto.VRData()
    msg.full_body.full_body.add().rot_qw = 1.
    receiver._handle_packet(msg.SerializeToString())


def test_dds_joystick_writer_and_reader_on_loopback(monkeypatch):
    import os
    import time
    from cyclonedds.domain import DomainParticipant
    monkeypatch.setenv('CYCLONEDDS_URI', '''<CycloneDDS><Domain Id="any">
      <General><Interfaces><NetworkInterface name="lo"/></Interfaces>
      <AllowMulticast>false</AllowMulticast></General>
      <Discovery><Peers><Peer Address="127.0.0.1"/></Peers></Discovery>
    </Domain></CycloneDDS>''')
    domain = 150 + os.getpid() % 40
    topic = '/test/gmr_controls_' + str(os.getpid())
    publisher = DdsPublishers(domain_id=domain, heft_topic=None, joy_topic=topic,
                              bone_topic=None, head_topic=None)
    participant = DomainParticipant(domain)
    subscriber = JoystickSubscriber(participant, topic)
    deadline = time.monotonic() + 2.
    received = []
    while not received and time.monotonic() < deadline:
        publisher.publish_joysticks(joy(right_trigger=1., left_second_button_pressed=True))
        time.sleep(.01)
        received = subscriber.take_fresh()
    assert received, 'DDS control reader did not receive local joystick messages'
    assert received[-1].right_trigger == 1.
    assert received[-1].left_second_button_pressed
