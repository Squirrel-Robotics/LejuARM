"""Small, dependency-free S17 URDF helpers for the HEFT precision test.

``estimate_root_height`` assumes both feet contact a flat floor and uses the
provided world orientation of base_link (level if omitted). It uses hardware
leg angles (indices 0..11), not HEFT waist-first
angles.  The estimate places the lowest collision geometry of each foot on
z=0 and averages the two required base heights.  URDF collision shapes can
differ slightly from the MuJoCo training model; this is a geometric estimate,
not a measured base height.  The caller should reject a large left/right
height difference. This geometric calculation does not verify stability.
"""

from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET


_IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
_ZERO = (0.0, 0.0, 0.0)


def _finite(value: str | float, context: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{context} must be finite")
    return number


def _vector(value: str, context: str) -> tuple[float, float, float]:
    result = tuple(_finite(item, context) for item in value.split())
    if len(result) != 3:
        raise ValueError(f"{context} must contain three numbers")
    return result


def _unit_quaternion(q) -> list[float]:
    if len(q) != 4:
        raise ValueError("quaternion must contain four wxyz components")
    values = [_finite(value, "quaternion component") for value in q]
    scale = max(abs(value) for value in values)
    if scale == 0.0:
        raise ValueError("quaternion must have nonzero norm")
    # Scaling first avoids overflow/underflow for otherwise finite inputs.
    values = [value / scale for value in values]
    norm = math.sqrt(sum(value * value for value in values))
    return [value / norm for value in values]


def quaternion_multiply(left_wxyz, right_wxyz) -> list[float]:
    """Compose world rotations in wxyz order, normalizing both and the result.

    The returned rotation applies ``right_wxyz`` first, then ``left_wxyz``.
    Nonfinite or zero input quaternions are rejected.
    """
    w1, x1, y1, z1 = _unit_quaternion(left_wxyz)
    w2, x2, y2, z2 = _unit_quaternion(right_wxyz)
    return _unit_quaternion([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def root_quat_from_imu(imu_wxyz, waist_hw: float) -> list[float]:
    """Remove S17 torso IMU's hardware waist yaw: imu * Rz(-waist_hw).

    ``waist_hw`` is /rt/joint_state.q[12] in radians, before the HEFT sign
    conversion. This matches HeftRLController::getFreejointAlignedQuat's
    waist compensation, before that controller's world-yaw alignment.
    """
    waist = _finite(waist_hw, "hardware waist angle")
    return quaternion_multiply(imu_wxyz,
                               [math.cos(waist / 2.0), 0.0, 0.0,
                                -math.sin(waist / 2.0)])


def tilt_angle_deg(q) -> float:
    """Angle between the rotated local +Z axis and world +Z, in degrees.

    This uses the rotation itself, with no Euler angles or yaw dependence.
    Passing a relative quaternion measures the relative Z-axis tilt; it does
    not measure heading change.
    """
    _, x, y, _ = _unit_quaternion(q)
    cos_angle = 1.0 - 2.0 * (x*x + y*y)
    return math.degrees(math.acos(max(-1.0, min(1.0, cos_angle))))


def _quaternion_rotation(q) -> tuple:
    w, x, y, z = _unit_quaternion(q)
    return ((1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)),
            (2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)),
            (2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)))


def _matmul(a: tuple, b: tuple) -> tuple:
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3))
                       for j in range(3)) for i in range(3))


def _apply(rotation: tuple, point: tuple) -> tuple:
    return tuple(sum(rotation[i][j] * point[j] for j in range(3))
                 for i in range(3))


def _compose(first: tuple, second: tuple) -> tuple:
    r1, p1 = first
    r2, p2 = second
    shifted = _apply(r1, p2)
    return _matmul(r1, r2), tuple(p1[i] + shifted[i] for i in range(3))


def _axis_rotation(axis: tuple, angle: float) -> tuple:
    norm = math.sqrt(sum(item * item for item in axis))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("joint axis must be a finite nonzero vector")
    x, y, z = (item / norm for item in axis)
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return ((c + x*x*t, x*y*t - z*s, x*z*t + y*s),
            (y*x*t + z*s, c + y*y*t, y*z*t - x*s),
            (z*x*t - y*s, z*y*t + x*s, c + z*z*t))


def _origin(element: ET.Element) -> tuple:
    origin = element.find("origin")
    if origin is None:
        return _IDENTITY, _ZERO
    xyz = _vector(origin.get("xyz", "0 0 0"), "origin xyz")
    roll, pitch, yaw = _vector(origin.get("rpy", "0 0 0"), "origin rpy")
    # URDF fixed-axis roll/pitch/yaw: Rz(yaw) Ry(pitch) Rx(roll).
    rotation = _matmul(_axis_rotation((0, 0, 1), yaw),
                       _matmul(_axis_rotation((0, 1, 0), pitch),
                               _axis_rotation((1, 0, 0), roll)))
    return rotation, xyz


def _positive(value: str | None, context: str) -> float:
    if value is None:
        raise ValueError(f"missing {context}")
    number = _finite(value, context)
    if number <= 0.0:
        raise ValueError(f"{context} must be positive")
    return number


def _collision_min_z(collision: ET.Element, link_pose: tuple) -> float:
    rotation, center = _compose(link_pose, _origin(collision))
    geometry = collision.find("geometry")
    if geometry is None or len(geometry) != 1:
        raise ValueError("foot collision must have exactly one geometry")
    shape = geometry[0]
    if shape.tag == "sphere":
        extent = _positive(shape.get("radius"), "sphere radius")
    elif shape.tag == "box":
        sizes = _vector(shape.get("size", ""), "box size")
        if any(item <= 0.0 for item in sizes):
            raise ValueError("box size must be positive")
        extent = sum(abs(rotation[2][i]) * sizes[i] / 2.0 for i in range(3))
    elif shape.tag == "cylinder":
        radius = _positive(shape.get("radius"), "cylinder radius")
        length = _positive(shape.get("length"), "cylinder length")
        extent = (radius * math.hypot(rotation[2][0], rotation[2][1])
                  + length / 2.0 * abs(rotation[2][2]))
    else:
        # Silently ignoring an unsupported shape can overestimate clearance.
        raise ValueError(f"unsupported foot collision geometry: {shape.tag}")
    return center[2] - extent


def _parse_urdf(urdf: Path) -> ET.Element:
    root = ET.parse(Path(urdf)).getroot()
    if root.tag != "robot":
        raise ValueError("URDF root element must be robot")
    return root


def load_joint_limits(urdf: Path) -> dict[str, tuple[float, float]]:
    """Return finite URDF limits (radians/metres) for bounded movable joints.

    Fixed and continuous joints are omitted.  A movable joint without a
    lower/upper limit is omitted, so callers can require the joints they use.
    Invalid numerical limits are rejected rather than treated as unbounded.
    """
    limits = {}
    for joint in _parse_urdf(urdf).findall("joint"):
        if joint.get("type") not in ("revolute", "prismatic"):
            continue
        limit = joint.find("limit")
        if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
            continue
        name = joint.get("name")
        if not name or name in limits:
            raise ValueError(f"missing or duplicate bounded joint name: {name!r}")
        lower = _finite(limit.get("lower"), f"{name} lower limit")
        upper = _finite(limit.get("upper"), f"{name} upper limit")
        if lower > upper:
            raise ValueError(f"{name} lower limit exceeds upper limit")
        limits[name] = (lower, upper)
    return limits


def estimate_root_height(urdf: Path, motor_q: list[float],
                         root_quat_wxyz=None) -> dict[str, float]:
    """Estimate base_link height from the oriented feet's collision surfaces.

    ``motor_q`` must contain 23 finite hardware angles.  Only the first 12
    enter FK; waist, arms and head do not alter the base_link-to-foot chain.
    ``root_quat_wxyz`` is base_link's world orientation; it is normalized and
    defaults to identity for compatibility. All link offsets and collision
    shapes are rotated into the world before taking their minimum Z.
    The two per-foot estimates let callers detect an asymmetric stance.
    """
    if len(motor_q) != 23:
        raise ValueError(f"expected 23 S17 hardware angles, got {len(motor_q)}")
    angles = [_finite(value, f"motor_q[{i}]") for i, value in enumerate(motor_q)]
    root_rotation = (_IDENTITY if root_quat_wxyz is None
                     else _quaternion_rotation(root_quat_wxyz))
    root = _parse_urdf(urdf)
    links = {}
    for link in root.findall("link"):
        name = link.get("name")
        if not name or name in links:
            raise ValueError(f"missing or duplicate link name: {name!r}")
        links[name] = link
    if "base_link" not in links:
        raise ValueError("URDF has no base_link")
    joints, parent_joints, children = {}, {}, {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        parent, child = joint.find("parent"), joint.find("child")
        if (not name or name in joints or parent is None or child is None
                or parent.get("link") not in links or child.get("link") not in links):
            raise ValueError(f"invalid or duplicate joint: {name!r}")
        parent_name, child_name = parent.get("link"), child.get("link")
        if child_name in parent_joints:
            raise ValueError(f"link has multiple parents: {child_name}")
        joints[name] = joint
        parent_joints[child_name] = joint
        children.setdefault(parent_name, []).append(joint)

    heights = []
    for side, motor_start in (("l", 0), ("r", 6)):
        expected = [f"leg_{side}{i}_joint" for i in range(1, 7)]
        if any(name not in joints for name in expected):
            raise ValueError(f"URDF is missing an S17 {side} leg joint")
        foot_name = joints[expected[-1]].find("child").get("link")
        chain, visited, cursor = [], set(), foot_name
        while cursor != "base_link":
            if cursor in visited or cursor not in parent_joints:
                raise ValueError(f"{foot_name} has no acyclic chain to base_link")
            visited.add(cursor)
            joint = parent_joints[cursor]
            chain.append(joint)
            cursor = joint.find("parent").get("link")
        chain.reverse()
        moving_names = [joint.get("name") for joint in chain
                        if joint.get("type") != "fixed"]
        if moving_names != expected:
            raise ValueError(f"unexpected {side} leg chain: {moving_names}")
        pose = (root_rotation, _ZERO)
        for joint in chain:
            pose = _compose(pose, _origin(joint))
            if joint.get("type") == "fixed":
                continue
            if joint.get("type") not in ("revolute", "continuous"):
                raise ValueError("S17 leg joints must be revolute")
            index = expected.index(joint.get("name")) + motor_start
            axis = joint.find("axis")
            axis_xyz = _vector(axis.get("xyz", "1 0 0") if axis is not None
                               else "1 0 0", "joint axis")
            pose = _compose(pose, (_axis_rotation(axis_xyz, angles[index]), _ZERO))

        # Include the foot roll link and every rigidly attached contact shape.
        pending, visited, lowest = [(foot_name, pose)], set(), []
        while pending:
            link_name, link_pose = pending.pop()
            if link_name in visited:
                raise ValueError("cycle in fixed foot links")
            visited.add(link_name)
            lowest.extend(_collision_min_z(collision, link_pose)
                          for collision in links[link_name].findall("collision"))
            for joint in children.get(link_name, []):
                if joint.get("type") == "fixed":
                    pending.append((joint.find("child").get("link"),
                                    _compose(link_pose, _origin(joint))))
        if not lowest:
            raise ValueError(f"{side} foot has no supported collision geometry")
        height = -min(lowest)
        if not math.isfinite(height):
            raise ValueError("computed root height is not finite")
        heights.append(height)
    return {"height_m": sum(heights) / 2.0,
            "left_height_m": heights[0], "right_height_m": heights[1]}
