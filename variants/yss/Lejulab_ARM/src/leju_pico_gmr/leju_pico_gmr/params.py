"""Source-specific IK configuration; robot geometry lives in leju_gmr_common."""
from pathlib import Path
from leju_gmr_common.params import (ROBOT_XML_DICT, ROBOT_BASE_DICT,
    VIEWER_CAM_DISTANCE_DICT, HEFT_JOINT_NAMES, HEFT_SCHEMA_VERSION, resolve_robot_name)

IK_CONFIG_DICT = {
    'bvh_motive_streaming': {"roban_s17": Path(__file__).parent / "ik_configs" / 'pico_to_roban_s17.json'},
}
