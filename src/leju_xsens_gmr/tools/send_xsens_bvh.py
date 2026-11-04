#!/usr/bin/env python3
"""Send a BVH skeleton to the Xsens receiver using only Python's stdlib.

Example (BVH offsets in centimeters, the default):
    python send_xsens_bvh.py actor.bvh --host ROBOT_IP --port 9764 --offset-scale 0.01
Use --offset-scale 1 when the BVH offsets are already in meters.
Only the hierarchy is sent; animation samples are ignored.
"""

import argparse
import json
import math
import re
import socket
import struct
import sys


PROTOCOL = "xsens_bvh_desc_v1"
MAX_FRAME_BYTES = 8 * 1024 * 1024
CHANNEL_NAMES = frozenset(
    axis + kind for axis in "XYZ" for kind in ("position", "rotation")
)


class BvhError(ValueError):
    """Invalid hierarchy or receiver response."""


def _positive_finite(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError("value must be finite and positive")
    return value


class _HierarchyParser:
    def __init__(self, text, scale):
        self.text = text
        self.tokens = iter(re.finditer(r"[{}]|[^\s{}]+", text))
        self.current = next(self.tokens, None)
        self.scale = scale
        self.joints = []
        self.names = set()

    def peek(self):
        return self.current.group() if self.current is not None else None

    def take(self, expected=None):
        value = self.peek()
        if value is None or (expected is not None and value != expected):
            raise BvhError("expected {!r}, got {!r}".format(expected or "token", value))
        self.current = next(self.tokens, None)
        return value

    def offset(self):
        values = []
        for _ in range(3):
            try:
                value = float(self.take()) * self.scale
            except ValueError as exc:
                raise BvhError("OFFSET requires three finite numbers") from exc
            if not math.isfinite(value):
                raise BvhError("OFFSET requires three finite numbers")
            values.append(value)
        return values

    def channels(self):
        count = self.take()
        if not re.fullmatch(r"[0-9]+", count):
            raise BvhError("CHANNELS count must be a nonnegative integer")
        count = int(count)
        if count > len(CHANNEL_NAMES):
            raise BvhError("CHANNELS count exceeds the six supported channels")
        names = [self.take() for _ in range(count)]
        if len(set(names)) != count or any(n not in CHANNEL_NAMES for n in names):
            raise BvhError("CHANNELS contains invalid or duplicate channel names")

    def joint(self, parent_id):
        # A name spans everything between ROOT/JOINT and its opening brace.
        # This preserves names containing spaces, namespaces, and Unicode.
        if self.current is None or self.peek() in ("{", "}"):
            raise BvhError("joint name is missing")
        start = self.current.start()
        while self.peek() != "{":
            if self.peek() in (None, "}", "OFFSET", "CHANNELS", "MOTION"):
                raise BvhError("missing opening brace after joint name")
            self.take()
        name = self.text[start:self.current.start()].strip()
        if name in self.names:
            raise BvhError("duplicate joint name: " + name)
        self.names.add(name)
        self.take("{")
        joint = {"id": len(self.joints) + 1, "name": name, "parent_id": parent_id}
        self.joints.append(joint)
        seen = set()
        children_started = False
        while self.peek() != "}":
            token = self.take()
            if token in ("OFFSET", "CHANNELS"):
                if token in seen or children_started:
                    raise BvhError("duplicate or misplaced " + token)
                seen.add(token)
                if token == "OFFSET":
                    joint["offset"] = self.offset()
                else:
                    self.channels()
            elif token in ("JOINT", "End"):
                children_started = True
                if token == "JOINT":
                    self.joint(joint["id"])
                else:
                    self.take("Site")
                    self.take("{")
                    self.take("OFFSET")
                    self.offset()
                    self.take("}")
            else:
                raise BvhError("unexpected hierarchy token: " + token)
        self.take("}")
        if seen != {"OFFSET", "CHANNELS"}:
            raise BvhError("joint {!r} requires OFFSET and CHANNELS".format(name))


def parse_bvh(text, offset_scale=0.01):
    """Return a receiver payload with continuous preorder IDs and meter offsets."""
    parser = _HierarchyParser(text.lstrip("\ufeff"), _positive_finite(offset_scale))
    parser.take("HIERARCHY")
    parser.take("ROOT")
    parser.joint(0)
    if parser.peek() not in (None, "MOTION"):
        raise BvhError("unexpected data after root: " + parser.peek())
    return {"protocol": PROTOCOL, "unit": "m", "joints": parser.joints}


def load_bvh(path, offset_scale=0.01):
    """Read only hierarchy lines, leaving potentially large motion data unread."""
    lines = []
    with open(path, "r", encoding="utf-8-sig") as source:
        for line in source:
            lines.append(line)
            if re.match(r"\s*MOTION\b", line):
                break
    return parse_bvh("".join(lines), offset_scale)


def recv_exact(sock, length):
    chunks = []
    while length:
        chunk = sock.recv(length)
        if not chunk:
            raise BvhError("connection closed before complete acknowledgment")
        chunks.append(chunk)
        length -= len(chunk)
    return b"".join(chunks)


def send_payload(payload, host, port=9764, timeout=10.0):
    """Send uint32 big-endian length + UTF-8 JSON, then validate the framed ACK."""
    timeout = _positive_finite(timeout)
    data = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode("utf-8")
    if not 0 < len(data) <= MAX_FRAME_BYTES:
        raise BvhError("payload exceeds receiver's default 8 MiB limit")
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.sendall(struct.pack(">I", len(data)) + data)
        size = struct.unpack(">I", recv_exact(sock, 4))[0]
        if not 0 < size <= MAX_FRAME_BYTES:
            raise BvhError("invalid acknowledgment frame length: {}".format(size))
        try:
            ack = json.loads(recv_exact(sock, size).decode("utf-8"))
        except (UnicodeError, ValueError) as exc:
            raise BvhError("acknowledgment is not valid UTF-8 JSON") from exc
        if not isinstance(ack, dict) or ack.get("ok") is not True:
            message = ack.get("message", "missing ok: true") if isinstance(ack, dict) else "expected object"
            raise BvhError("receiver rejected skeleton: {}".format(message))
        return ack


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bvh", help="local BVH file (read-only)")
    parser.add_argument("--host", required=True, help="robot IP address or hostname")
    parser.add_argument("--port", type=int, default=9764)
    parser.add_argument("--offset-scale", type=_positive_finite, default=0.01,
                        help="offset multiplier to meters (default: 0.01, centimeters to meters; use 1 for meters)")
    parser.add_argument("--timeout", type=_positive_finite, default=10.0,
                        help="socket timeout in seconds (default: 10)")
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    try:
        payload = load_bvh(args.bvh, args.offset_scale)
        ack = send_payload(payload, args.host, args.port, args.timeout)
    except (OSError, ValueError, RecursionError) as exc:
        print("Error: {}".format(exc), file=sys.stderr)
        return 1
    print("Sent {} joints: {}".format(len(payload["joints"]), ack.get("message", "accepted")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
