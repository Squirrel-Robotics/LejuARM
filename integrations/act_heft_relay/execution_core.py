"""Pure bounded reference execution. No transport or robot imports."""
import math

NAMES = (["waist_yaw_joint"] + [f"leg_{s}{i}_joint" for s in ("l", "r") for i in range(1, 7)]
         + [f"zarm_{s}{i}_joint" for s in ("l", "r") for i in range(1, 5)])
CLOSE = [100., 50., 100., 100., 100., 100.]
PERIOD = .02
# User requested rollback to 60 deg/s on 2026-11-03 after the 133 deg/s run.
# Larger model steps are interpolated; the native preflight and wire packet
# checks share this finite ceiling.
BODY_SPEED_ENVELOPE_DEG_S = 60.0


def feedback_age_limit_ms(config):
    """Finite joint/hand feedback budget, separate from command/heartbeat ages."""
    value = config.get('joint_and_hand_age_limit_ms', 250)
    if type(value) not in (int, float) or not math.isfinite(value) or not 250 <= value <= 1000:
        raise ValueError('INVALID_FEEDBACK_AGE_LIMIT_MS')
    return float(value)


def command_packet_age_limit_ms(config):
    """Bound command lifetime independently of feedback and ACK wait windows."""
    value = config.get('command_packet_age_limit_ms', 250)
    if type(value) not in (int, float) or not math.isfinite(value) or not 250 <= value <= 1000:
        raise ValueError('INVALID_COMMAND_PACKET_AGE_LIMIT_MS')
    return float(value)


def relay_timeouts(config):
    """LAN wait budgets only; sample and command age limits stay independent."""
    ack = config.get('relay_ack_timeout_s', .25)
    idle = config.get('relay_input_timeout_s', .25)
    if (any(type(x) not in (int, float) or not math.isfinite(x) for x in (ack, idle))
            or not .25 <= ack <= 2. or not ack <= idle <= ack + .5):
        raise ValueError('INVALID_RELAY_TIMEOUTS')
    return float(ack), float(idle)


def finite(values, length):
    if not isinstance(values, list) or len(values) != length or any(
            type(x) not in (int, float) or not math.isfinite(x) for x in values):
        raise ValueError("NONFINITE_OR_WRONG_VECTOR")
    return list(values)


def hands(values):
    """Validate outgoing commands against the trained open/close endpoints."""
    values = finite(values, 12)
    if any(not 0 <= x <= CLOSE[i % 6] for i, x in enumerate(values)):
        raise ValueError("HAND_RANGE")
    return values


def hand_feedback(values):
    """Keep measured channels unchanged; command endpoints are not sensor limits."""
    return finite(values, 12)


def hand_command_from_feedback(values):
    """Derive a bounded initial command without modifying measured evidence."""
    return [min(max(value, 0.), CLOSE[i % 6])
            for i, value in enumerate(hand_feedback(values))]


def require_hand_samples(data, header_age_s, max_age_ms=250):
    max_age_ms = feedback_age_limit_ms({'joint_and_hand_age_limit_ms': max_age_ms})
    if not math.isfinite(header_age_s) or not -.05 <= header_age_s <= max_age_ms / 1000:
        raise RuntimeError('HAND_HEADER_STALE')
    for side in ('left', 'right'):
        age = data.get(side+'_sample_age_ms')
        if (data.get(side+'_valid') is not True or type(age) not in (int,float)
                or not math.isfinite(age) or age < 0
                or age + max(0.,header_age_s)*1000 > max_age_ms):
            raise RuntimeError('HAND_SAMPLE_STALE:'+side)


def body(values, limits):
    values = finite(values, 21)
    if any(not limits[n][0] <= q <= limits[n][1] for n, q in zip(NAMES, values)):
        raise ValueError("JOINT_RANGE")
    return values


def speed_caps(config, side):
    arm = float(config['arm_speed_deg_s'])
    waist = float(config['waist_speed_deg_s'])
    if not all(math.isfinite(x) and 0 < x <= BODY_SPEED_ENVELOPE_DEG_S for x in (arm, waist)):
        raise ValueError("SPEED_OUTSIDE_REVIEWED_ENVELOPE")
    active = [0] + list(range(13, 17) if side == 'left' else range(17, 21))
    return [math.radians(waist if i == 0 else arm) if i in active else 0. for i in range(21)]


def step_reference(previous, target, caps, dt=PERIOD):
    """Common interpolation fraction preserves the requested joint-space segment."""
    previous, target = finite(previous, 21), finite(target, 21)
    if not math.isfinite(dt) or not 0 < dt <= PERIOD + 1e-12:
        raise ValueError("INVALID_OR_CATCHUP_TIMESTEP")
    fraction = 1.
    for a, b, cap in zip(previous, target, caps):
        if abs(b-a) > 1e-12:
            if cap <= 0:
                raise ValueError("INACTIVE_JOINT_CHANGED")
            fraction = min(fraction, cap * dt / abs(b-a))
    return [a + fraction * (b-a) for a, b in zip(previous, target)]


def within(a, b, indices, tolerance_deg):
    return max((abs(a[i]-b[i]) for i in indices), default=0) <= math.radians(tolerance_deg)


def feedback_guard_indices(config, active):
    """Only the explicitly requested right R3 feedback exception is supported."""
    monitor = config.get('body_feedback_monitor_only_joints', [])
    if (not isinstance(monitor, list) or any(name != 'zarm_r3_joint' for name in monitor)
            or len(monitor) != len(set(monitor))):
        raise ValueError('INVALID_BODY_FEEDBACK_MONITOR_ONLY_JOINTS')
    return [i for i in active if NAMES[i] not in monitor]


def tracking_diagnostics(measured, requested, active, config):
    measured, requested = finite(measured, 21), finite(requested, 21)
    required = feedback_guard_indices(config, active)
    errors = {NAMES[i]: abs(math.degrees(measured[i]-requested[i])) for i in active}
    monitor = {NAMES[i]: errors[NAMES[i]] for i in active if i not in required}
    return dict(required_body_joint_names=[NAMES[i] for i in required],
                body_error_deg_by_name=errors, monitor_only_body_error_deg=monitor,
                max_required_body_error_deg=max((errors[NAMES[i]] for i in required), default=0.),
                monitor_only_over_tracking_limit=[name for name, error in monitor.items()
                                                 if error > config['tracking_abort_deg']])


def require_body_tracking(measured, requested, active, tolerance_deg, *, config=None):
    """Apply the limit to required joints and retain every measured error."""
    settings = dict(config or {}, tracking_abort_deg=tolerance_deg)
    report = tracking_diagnostics(measured, requested, active, settings)
    if not within(measured, requested, feedback_guard_indices(settings, active), tolerance_deg):
        raise RuntimeError('BODY_TRACKING_ERROR')
    return report


def final_home_feedback(measured, home, active, config):
    required = feedback_guard_indices(config, active)
    errors = tracking_diagnostics(measured, home, active, config)['body_error_deg_by_name']
    return dict(measured_home_reached=within(measured, home, active, config['return_home_tolerance_deg']),
                required_home_reached=within(measured, home, required, config['return_home_tolerance_deg']),
                final_body_home_error_deg_by_name=errors)


class Executor:
    def __init__(self, home, hand_home, side, config):
        if side not in ('left', 'right'):
            raise ValueError('INVALID_SIDE')
        self.config, self.side = config, side
        self.home = body(home, config['joint_limits_rad'])
        self.q = self.home.copy()
        self.hand_home = hands(hand_home)
        self.hand = self.hand_home.copy()
        self.active = [0] + list(range(13, 17) if side == 'left' else range(17, 21))
        self.feedback_required = feedback_guard_indices(config, self.active)
        self.hand_indices = list(range(0, 6) if side == 'left' else range(6, 12))
        self.caps = speed_caps(config, side)
        self.stable_ticks, self.ramp_tick, self.ramp_target = 0, 0, None

    def validate_target(self, q, hand):
        q, hand = body(q, self.config['joint_limits_rad']), hands(hand)
        for i in range(21):
            if i not in self.active and abs(q[i] - self.home[i]) > 1e-9:
                raise ValueError('INACTIVE_BODY_CHANGED')
        for i in range(12):
            if i not in self.hand_indices:
                hand[i] = self.hand_home[i]
        return q, hand

    def tick(self, target, hand_target, measured, *, hand_boundary=False,
             enforce_tracking=True, dt=PERIOD, measured_advanced=True):
        target, hand_target = self.validate_target(target, hand_target)
        measured = finite(measured, 21)
        tracking = tracking_diagnostics(measured, self.q, self.active, self.config)
        if enforce_tracking:
            require_body_tracking(measured, self.q, self.active, self.config['tracking_abort_deg'], config=self.config)
        self.q = step_reference(self.q, target, self.caps, dt)
        reached = within(self.q, target, self.active, .000001)
        settled = reached and within(measured, target, self.feedback_required, self.config['hand_body_tolerance_deg'])
        # Count independent feedback samples, not duplicate 50 Hz API reads.
        if not settled or not hand_boundary:
            self.stable_ticks = 0
            self.ramp_target = None
        elif measured_advanced:
            self.stable_ticks += 1
        allowed = hand_boundary and self.stable_ticks >= self.config['hand_settle_samples']
        if allowed:
            if self.ramp_target != hand_target:
                self.ramp_from, self.ramp_target = self.hand.copy(), hand_target.copy()
                self.ramp_tick = 0
            self.ramp_tick += dt
            u = min(1., self.ramp_tick / self.config['hand_ramp_s'])
            w = 10*u**3 - 15*u**4 + 6*u**5
            self.hand = [a+w*(b-a) for a, b in zip(self.ramp_from, self.ramp_target)]
        hand_done = max(abs(a-b) for a, b in zip(self.hand, hand_target)) < 1e-7
        return dict(q=self.q.copy(), hand=self.hand.copy(), body_reached=reached, body_tracking=tracking,
                    hand_allowed=allowed, hand_done=hand_done,
                    phase='hand_ramp' if allowed and not hand_done else ('settle' if reached else 'body'))


def validate_packet(packet, previous, home, hand_home, config, side, now, last_sequence):
    if packet.get('op') != 'target' or type(packet.get('seq')) is not int or packet['seq'] <= last_sequence:
        raise ValueError('PACKET_SEQUENCE')
    stamp = packet.get('sent_at')
    max_age_s = command_packet_age_limit_ms(config) / 1000.
    if type(stamp) not in (int, float) or not math.isfinite(stamp) or not -.05 <= now-stamp <= max_age_s:
        raise ValueError('PACKET_STALE')
    q, hand = body(packet.get('q'), config['joint_limits_rad']), hands(packet.get('hand'))
    active = [0] + list(range(13, 17) if side == 'left' else range(17, 21))
    hand_indices = list(range(0, 6) if side == 'left' else range(6, 12))
    if any(abs(q[i]-home[i]) > 1e-9 for i in range(21) if i not in active):
        raise ValueError('INACTIVE_BODY_CHANGED')
    if any(abs(hand[i]-hand_home[i]) > 1e-9 for i in range(12) if i not in hand_indices):
        raise ValueError('INACTIVE_HAND_CHANGED')
    caps = speed_caps(config, side)
    if any(abs(q[i]-previous['q'][i]) > caps[i]*PERIOD+1e-8 for i in range(21)):
        raise ValueError('BODY_RATE')
    # 1 s minimum-jerk hand ramp has maximum normalized slope 1.875/s.
    if any(abs(hand[i]-previous['hand'][i]) > CLOSE[i % 6]*1.875*PERIOD+1e-7 for i in range(12)):
        raise ValueError('HAND_RATE')
    return q, hand
