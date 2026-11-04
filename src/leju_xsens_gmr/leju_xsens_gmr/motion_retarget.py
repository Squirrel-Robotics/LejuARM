"""Bind the shared solver to this frontend's IK configuration."""
from leju_gmr_common.motion_retarget import MotionRetargeter as BaseRetargeter
from .params import IK_CONFIG_DICT


class MotionRetargeter(BaseRetargeter):
    def __init__(self, src_human, tgt_robot, *args, **kwargs):
        kwargs.setdefault("ik_config_path", IK_CONFIG_DICT[src_human][tgt_robot])
        super().__init__(src_human, tgt_robot, *args, **kwargs)
