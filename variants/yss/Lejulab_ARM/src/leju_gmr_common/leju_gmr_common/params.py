"""Shared S17 robot model and HEFT joint contract.

Only Roban S17 is registered. ``robans17`` is accepted as an alias of the
canonical name ``roban_s17``.
"""

import pathlib

HERE = pathlib.Path(__file__).parent

# leju_assets is a sibling package inside the same repository checkout.
REPO_SRC = HERE.parent.parent  # .../src
ASSET_ROOT = REPO_SRC / "leju_assets" / "models"

CANONICAL_ROBOT = "roban_s17"
ROBOT_ALIASES = {
    "roban_s17": CANONICAL_ROBOT,
    "robans17": CANONICAL_ROBOT,
    "s17": CANONICAL_ROBOT,
}

ROBOT_XML_DICT = {
    "roban_s17": ASSET_ROOT / "biped_s17" / "xml" / "biped_s17.xml",
}


ROBOT_BASE_DICT = {
    "roban_s17": "base_link",
}

VIEWER_CAM_DISTANCE_DICT = {
    "roban_s17": 3.0,
}

# HEFT policy joint order for Roban S17 (must match
# leju-rl-controller config_heft.yaml joint_names exactly).
HEFT_JOINT_NAMES = [
    "waist_yaw_joint",
    "leg_l1_joint", "leg_l2_joint", "leg_l3_joint",
    "leg_l4_joint", "leg_l5_joint", "leg_l6_joint",
    "leg_r1_joint", "leg_r2_joint", "leg_r3_joint",
    "leg_r4_joint", "leg_r5_joint", "leg_r6_joint",
    "zarm_l1_joint", "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint",
    "zarm_r1_joint", "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint",
]

HEFT_SCHEMA_VERSION = 1


def resolve_robot_name(name: str) -> str:
    """Resolve a robot name or alias to the canonical registry key."""
    key = name.strip().lower()
    if key not in ROBOT_ALIASES:
        raise KeyError(
            f"Unknown robot '{name}'. Supported: {sorted(ROBOT_ALIASES.keys())}"
        )
    return ROBOT_ALIASES[key]
