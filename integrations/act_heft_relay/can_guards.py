"""Live CAN counters and progress, including qualified warnings on all four buses."""

class CanWatch:
    """An unchanged warning enum alone is not an active communication fault."""
    def __init__(self):
        self.baseline = None
        self.previous = {}
        self.progress = {}
        self.samples = 0

    def update(self, rows, now):
        if {r['ifname'] for r in rows} != {'bcan0', 'bcan1', 'bcan2', 'bcan3'}:
            raise RuntimeError('CAN_INTERFACE_MISSING')
        events = {}
        for row in rows:
            name = row['ifname']
            info = row['linkinfo']['info_data']
            # A warning on any bus may pass only with zero current error
            # counters; traffic and new error events are still checked below.
            allowed = ('ERROR-ACTIVE', 'ERROR-WARNING')
            if info['state'] not in allowed or info.get('berr_counter') != {'tx': 0, 'rx': 0}:
                raise RuntimeError('CAN_LIVE_ERROR: ' + name)
            if row.get('operstate') != 'UP' or 'UP' not in row['flags']:
                raise RuntimeError('CAN_LINK_DOWN: ' + name)
            x = row['linkinfo']['info_xstats']
            events[name] = {k: x[k] for k in ('restarts', 'bus_error', 'error_warning', 'error_passive', 'bus_off')}
            counts = []
            for direction in ('rx', 'tx'):
                stats = row['stats64'][direction]
                if any(stats.get(k, 0) for k in ('errors', 'dropped', 'over_errors', 'carrier_errors')):
                    raise RuntimeError('CAN_PACKET_ERRORS: ' + name)
                counts.append(stats['packets'])
            old = self.previous.get(name)
            if old is not None and any(a < b for a, b in zip(counts, old)):
                raise RuntimeError('CAN_COUNTER_RESET: ' + name)
            if old is None or all(a > b for a, b in zip(counts, old)):
                self.progress[name] = now
            if now - self.progress[name] > .5:
                raise RuntimeError('CAN_TRAFFIC_STOPPED: ' + name)
            self.previous[name] = counts
        if self.baseline is None:
            self.baseline = events
        elif events != self.baseline:
            raise RuntimeError('CAN_NEW_ERROR_EVENT')
        self.samples += 1
