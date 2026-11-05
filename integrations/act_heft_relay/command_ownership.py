"""Distinguish idle SDK endpoints from competing hand command streams."""
from collections import OrderedDict
import math


class HandCommandOwner:
    def __init__(self):
        self.sent = OrderedDict()
        self.publication_handle = None
        self.received = 0
        self.foreign = 0

    def register(self, stamp_ns, positions):
        self.sent[stamp_ns] = list(positions)
        while len(self.sent) > 300:
            self.sent.popitem(last=False)

    def observe(self, publication_handle, stamp_ns, positions):
        self.received += 1
        expected = self.sent.get(stamp_ns)
        if (expected is None or len(positions) != len(expected) or
                any(not math.isfinite(v) for v in positions) or
                any(abs(a-b)>1e-4 for a,b in zip(expected,positions)) or
                (self.publication_handle is not None and publication_handle != self.publication_handle)):
            self.foreign += 1
            raise RuntimeError('OTHER_ACTIVE_HAND_COMMAND_SOURCE: ' + str(publication_handle))
        self.publication_handle = publication_handle
