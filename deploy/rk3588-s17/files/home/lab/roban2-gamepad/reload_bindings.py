#!/usr/bin/env python3
"""Notify the existing controller to reload its current teleop config only."""
import os
import time
import threading
from dataclasses import dataclass
os.environ.setdefault('CYCLONEDDS_HOME', '/home/lab/Lejulab_ARM/installed')
from cyclonedds.domain import DomainParticipant
from cyclonedds.core import Listener
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.annotations import final
from cyclonedds.idl.types import int32, uint32

@final
@dataclass
class StringData(IdlStruct, typename='leju::msgs::StringData'):
    header_sec: int32 = 0
    header_nanosec: uint32 = 0
    data: str = ''

dp = DomainParticipant(0)
qos = Qos(Policy.Reliability.Reliable(int(.1e9)),
          Policy.Durability.Volatile, Policy.History.KeepLast(1))
topic = Topic(dp, '/rt/teleop/reload_config', StringData, qos=qos)
matched = threading.Event()
class MatchListener(Listener):
    def on_publication_matched(self, writer, status):
        if status.current_count > 0:
            matched.set()
        else:
            matched.clear()
listener = MatchListener()
writer = DataWriter(Publisher(dp), topic, qos=qos, listener=listener)
if not matched.wait(3):
    raise SystemExit('No live reload subscriber; saved config loads on next controller startup')
for _ in range(3):
    now = time.time_ns()
    writer.write(StringData(now//1_000_000_000, now%1_000_000_000, ''))
    time.sleep(.3)
print('Reload notification sent to the running controller')
