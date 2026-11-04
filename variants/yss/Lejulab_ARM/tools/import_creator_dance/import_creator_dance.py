#!/usr/bin/env python3
"""Import Leju Creator exported dance packages into leju-controllers.

New platform (lejulab_platform) path layout:
  src/leju-controllers/leju-rl-controller/config/<ROBOT_VERSION>/
    controllers/
      config_mimic_dance_<controller_name>.yaml  ← 生成
    policy/dance/<controller_name>.onnx          ← 从 zip 解出
    motion_data/<controller_name>.csv            ← 从 zip 解出
    controller_manager.yaml                       ← 追加条目
    teleop_bindings.yaml                          ← 不修改, 桌面端自行管理
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import sys
import unicodedata
import zipfile
from pathlib import Path
from typing import Any

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
CONTROLLER_CONFIG_ROOT = REPO_ROOT / "src/leju-controllers/leju-rl-controller/config"


class CreatorYamlLoader(yaml.SafeLoader):
    """Safe-ish loader for Isaac Lab YAML tags used in creator exports."""


def _construct_python_tuple(loader: yaml.Loader, node: yaml.Node) -> tuple[Any, ...]:
    return tuple(loader.construct_sequence(node))


def _construct_python_slice(loader: yaml.Loader, node: yaml.Node) -> slice:
    values = loader.construct_sequence(node)
    return slice(*values)


CreatorYamlLoader.add_constructor("tag:yaml.org,2002:python/tuple", _construct_python_tuple)
CreatorYamlLoader.add_constructor(
    "tag:yaml.org,2002:python/object/apply:builtins.slice", _construct_python_slice
)


class ImportErrorWithHint(RuntimeError):
    pass


def normalize_stem(stem: str) -> str:
    return unicodedata.normalize("NFC", stem).strip()


def make_controller_name(stem: str) -> str:
    digest = hashlib.sha1(stem.encode("utf-8")).hexdigest()[:10]
    slug = re.sub(r"[^a-z0-9]+", "_", stem.lower()).strip("_")[:24]
    return f"dance_{slug}_{digest}" if slug else f"dance_{digest}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Import a Leju Creator dance zip into leju-controllers.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("zip_path", type=Path, help="Path to the Leju Creator export zip.")
    parser.add_argument(
        "--force", action="store_true",
        help="Overwrite existing controller when dance already imported.",
    )
    parser.add_argument(
        "--json", action="store_true",
        help="Emit a single-line JSON result to stdout (for ws handler).",
    )
    return parser.parse_args()


def fail(message: str) -> None:
    raise ImportErrorWithHint(message)


def classify_error(message: str) -> str:
    msg = message.lower()
    if "joints" in msg and "creator export has" in msg:
        return "E_JOINT_MISMATCH"
    if "controller already exists" in msg or "already exists:" in msg:
        return "E_BINDING_EXISTS"
    if "zip" in msg and ("invalid" in msg or "missing required" in msg
                         or "does not exist" in msg or "multiple" in msg):
        return "E_BAD_ZIP"
    if "trajectory" in msg and "csv" in msg:
        return "E_BAD_ZIP"
    return "E_GENERIC"


# ---------------------------------------------------------------------------
# Zip member helpers
# ---------------------------------------------------------------------------

def _is_zip_member_usable(name: str) -> bool:
    parts = [part for part in name.replace("\\", "/").split("/") if part]
    if not parts or "__MACOSX" in parts:
        return False
    return not any(part.startswith("._") or part.startswith(".") for part in parts)


def _zip_member_basename(name: str) -> str:
    parts = [part for part in name.replace("\\", "/").split("/") if part]
    return parts[-1] if parts else ""


def _find_unique_by_ext(
    zip_file: zipfile.ZipFile, exts: tuple[str, ...], label: str, required: bool = True,
) -> str | None:
    exts = tuple(e.lower() for e in exts)
    matches = [
        info.filename
        for info in zip_file.infolist()
        if not info.is_dir()
        and _is_zip_member_usable(info.filename)
        and _zip_member_basename(info.filename).lower().endswith(exts)
    ]
    if not matches:
        if required:
            fail(f"zip is missing required {label} file (expected one *{exts[0]})")
        return None
    if len(matches) > 1:
        fail(f"zip contains multiple {label} files, expected exactly one: {matches}")
    return matches[0]


def load_creator_env(zip_path: Path) -> tuple[dict[str, Any], dict[str, str]]:
    if not zip_path.is_file():
        fail(f"zip path does not exist: {zip_path}")
    try:
        with zipfile.ZipFile(zip_path) as zf:
            members = {
                "env": _find_unique_by_ext(zf, (".yaml", ".yml"), "env yaml"),
                "model": _find_unique_by_ext(zf, (".onnx",), "model onnx"),
                "trajectory": _find_unique_by_ext(zf, (".csv",), "trajectory csv"),
            }
            env = yaml.load(zf.read(members["env"]), Loader=CreatorYamlLoader)
            if not isinstance(env, dict):
                fail("env yaml did not parse as a YAML mapping")
            return env, {key: value for key, value in members.items() if value}
    except zipfile.BadZipFile as exc:
        fail(f"invalid zip file: {exc}")
    except yaml.YAMLError as exc:
        fail(f"failed to parse env yaml: {exc}")


# ---------------------------------------------------------------------------
# Env.yaml extraction helpers
# ---------------------------------------------------------------------------

def require_mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        fail(f"{label} must be a mapping")
    return value


def require_list(value: Any, label: str) -> list[Any]:
    if not isinstance(value, list):
        fail(f"{label} must be a list")
    return value


def get_nested(data: dict[str, Any], path: str) -> Any:
    cur: Any = data
    for part in path.split("."):
        if not isinstance(cur, dict) or part not in cur:
            fail(f"env.yaml is missing required field: {path}")
        cur = cur[part]
    return cur


def regex_value_for_joint(mapping: dict[str, Any], joint_name: str, field_name: str) -> float:
    if joint_name in mapping:
        return float(mapping[joint_name])
    for pattern, value in mapping.items():
        if re.fullmatch(str(pattern), joint_name):
            return float(value)
    fail(f"{field_name} has no value for joint: {joint_name}")


def _default_torque_limit(joint_name: str) -> float:
    if joint_name == "waist_yaw_joint" or joint_name == "waist_joint":
        return 64.0
    if re.fullmatch(r"leg_[lr][12]_joint", joint_name):
        return 120.0
    if re.fullmatch(r"leg_[lr]3_joint", joint_name):
        return 56.0
    if re.fullmatch(r"leg_[lr]4_joint", joint_name):
        return 150.0
    if re.fullmatch(r"leg_[lr][56]_joint", joint_name):
        return 70.2
    if re.fullmatch(r"zarm_[lr]1_joint", joint_name):
        return 11.28
    if joint_name.startswith("zarm_"):
        return 29.6
    return 100.0


def collect_joint_config(env: dict[str, Any]) -> dict[str, Any]:
    action_cfg = require_mapping(get_nested(env, "actions.joint_pos"), "actions.joint_pos")
    action_joints = [str(item) for item in
                     require_list(action_cfg.get("joint_names"), "actions.joint_pos.joint_names")]
    if not action_joints:
        fail("actions.joint_pos.joint_names is empty")

    policy_joint_names = get_nested(env, "observations.policy.joint_pos.params.asset_cfg.joint_names")
    policy_joints = [str(item) for item in
                     require_list(policy_joint_names, "observations.policy joint_names")]
    if policy_joints != action_joints:
        fail("policy joint_names and action joint_names differ; refusing to guess joint order")

    init_joint_pos = require_mapping(
        get_nested(env, "scene.robot.init_state.joint_pos"), "scene.robot.init_state.joint_pos")
    actuators = require_mapping(
        get_nested(env, "scene.robot.actuators.motor"), "scene.robot.actuators.motor")
    stiffness = require_mapping(actuators.get("stiffness"), "actuators.motor.stiffness")
    damping = require_mapping(actuators.get("damping"), "actuators.motor.damping")
    action_scale = require_mapping(action_cfg.get("scale"), "actions.joint_pos.scale")

    effort_src = actuators.get("effort_limit_sim") or actuators.get("effort_limit")
    if effort_src is not None:
        effort_src = require_mapping(effort_src, "effort_limit")
        torque_limits = [regex_value_for_joint(effort_src, joint, "effort_limit")
                         for joint in action_joints]
    else:
        torque_limits = [_default_torque_limit(joint) for joint in action_joints]

    return {
        "joint_names": action_joints,
        "default_joint_pos": [float(init_joint_pos.get(joint, 0.0)) for joint in action_joints],
        "joint_kp": [regex_value_for_joint(stiffness, joint, "stiffness") for joint in action_joints],
        "joint_kd": [regex_value_for_joint(damping, joint, "damping") for joint in action_joints],
        "torque_limits": torque_limits,
        "action_scale": [regex_value_for_joint(action_scale, joint, "action_scale")
                         for joint in action_joints],
    }


def get_residual_action(env: dict[str, Any]) -> bool:
    action_cfg = require_mapping(get_nested(env, "actions.joint_pos"), "actions.joint_pos")
    class_type = str(action_cfg.get("class_type", ""))
    if "Residual" in class_type:
        return True
    if "residual" in action_cfg:
        return bool(action_cfg["residual"])
    return False


# ---------------------------------------------------------------------------
# Controller config YAML builder (new lejulab format)
# ---------------------------------------------------------------------------

def build_controller_config(
    env: dict[str, Any],
    controller_name: str,
    onnx_name: str,
    csv_name: str,
) -> dict[str, Any]:
    """Build a config_mimic_dance_<name>.yaml dict in leju-controllers format."""
    joint_cfg = collect_joint_config(env)
    joint_names = joint_cfg["joint_names"]
    num_joints = len(joint_names)
    sim_dt = float(get_nested(env, "sim.dt"))
    decimation = int(env.get("decimation", 1))
    policy_dt = sim_dt * decimation

    residual_action = get_residual_action(env)

    # Default joint direction: waist_yaw is -1.0, all others 1.0
    joint_direction = [-1.0 if j in ("waist_yaw_joint", "waist_joint") else 1.0
                       for j in joint_names]

    # Default actuator_control_mode: 0 for all joints
    control_mode = [0] * num_joints

    config = {
        "HumanoidRobotCfg": {
            "inference_engine": "onnxruntime",
            "loop_dt": 0.001,
            "policy_path": f"../policy/dance/{onnx_name}",
            "motion": {
                "default_motion": controller_name,
                "motions": [
                    {
                        "name": controller_name,
                        "file": f"../motion_data/{csv_name}",
                    }
                ],
                # 舞蹈播完自动切回稳定控制器（与预置舞蹈 config_mimic_dance_*.yaml 对齐）
                "auto_switch_back": {
                    "enabled": True,
                    "target_controller": "amp",
                },
            },
            "env": {
                "policy_dt": float(policy_dt),
                "robot": {
                    "residual_action": residual_action,
                    "joint_names": list(joint_names),
                    "joint_direction": joint_direction,
                    "joint_default_pos": [float(v) for v in joint_cfg["default_joint_pos"]],
                    "joint_torque_limit": [float(v) for v in joint_cfg["torque_limits"]],
                    "actuator_kp": [float(v) for v in joint_cfg["joint_kp"]],
                    "actuator_kd": [float(v) for v in joint_cfg["joint_kd"]],
                    "actuator_control_mode": control_mode,
                    "action_scale": [float(v) for v in joint_cfg["action_scale"]],
                },
                "observations": {
                    "history_length": 1,
                    "stack_order": "classic",
                    "terms": {
                        "motion_command": {"scale": 1.0, "clip": [-5000.0, 5000.0]},
                        "motion_target_height": {"scale": 1.0, "clip": [-5000.0, 5000.0]},
                        "motion_anchor_ori_b": {"scale": 1.0, "clip": [-5000.0, 5000.0]},
                        "projected_gravity": {"scale": 1.0, "clip": [-5000.0, 5000.0]},
                        "base_ang_vel": {"scale": 1.0, "clip": [-5000.0, 5000.0]},
                        "joint_pos": {"scale": 1.0, "clip": [-5000.0, 5000.0]},
                        "joint_vel": {"scale": 1.0, "clip": [-5000.0, 5000.0]},
                        "actions": {"scale": 1.0, "clip": [-5000.0, 5000.0]},
                    },
                },
            },
        }
    }
    return config


# ---------------------------------------------------------------------------
# YAML file I/O
# ---------------------------------------------------------------------------

def read_controllers_yaml(path: Path) -> dict[str, Any]:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError as exc:
        fail(f"failed to parse YAML {path}: {exc}")
    if not isinstance(data, dict):
        fail(f"YAML file must contain a mapping: {path}")
    return data


def check_file_conflict(path: Path, force: bool, label: str) -> None:
    if path.exists() and not force:
        fail(f"{label} already exists: {path}; pass --force to overwrite")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    args = parse_args()
    try:
        display_name = normalize_stem(args.zip_path.stem)
        if not display_name:
            fail("zip filename has no stem; cannot derive dance_name")
        controller_name = make_controller_name(display_name)

        robot_version = os.environ.get("ROBOT_VERSION", "").strip()
        if not robot_version:
            fail("ROBOT_VERSION is not set")
        if not re.fullmatch(r"\d+", robot_version):
            fail(f"ROBOT_VERSION must be numeric, got: {robot_version}")

        version_dir = CONTROLLER_CONFIG_ROOT / robot_version
        if not version_dir.is_dir():
            fail(f"target robot config directory does not exist: {version_dir}")

        controllers_yaml = version_dir / "controller_manager.yaml"
        if not controllers_yaml.is_file():
            fail(f"target controller_manager.yaml does not exist: {controllers_yaml}")

        # Check for duplicate
        existing = read_controllers_yaml(controllers_yaml)
        existing_controllers = existing.get("controllers", [])
        if isinstance(existing_controllers, list):
            for item in existing_controllers:
                if isinstance(item, dict) and item.get("name") == controller_name:
                    if not args.force:
                        fail(f"controller already exists: {controller_name}; pass --force to overwrite")
                    break

        env, members = load_creator_env(args.zip_path)

        onnx_name = f"{controller_name}.onnx"
        csv_name = f"{controller_name}.csv"
        config_name = f"config_mimic_dance_{controller_name}.yaml"

        controllers_subdir = version_dir / "controllers"
        policy_dance_dir = version_dir / "policy" / "dance"
        motion_data_dir = version_dir / "motion_data"

        config_path = controllers_subdir / config_name
        onnx_path = policy_dance_dir / onnx_name
        csv_path = motion_data_dir / csv_name

        check_file_conflict(config_path, args.force, "controller config yaml")
        check_file_conflict(onnx_path, args.force, "onnx model")
        check_file_conflict(csv_path, args.force, "trajectory csv")

        # Build controller config
        config_data = build_controller_config(env, controller_name, onnx_name, csv_name)

        # Write files with temp + atomic rename, cleanup on failure
        controllers_subdir.mkdir(parents=True, exist_ok=True)
        policy_dance_dir.mkdir(parents=True, exist_ok=True)
        motion_data_dir.mkdir(parents=True, exist_ok=True)

        config_tmp = config_path.with_suffix(config_path.suffix + ".tmp")
        onnx_tmp = onnx_path.with_suffix(onnx_path.suffix + ".tmp")
        csv_tmp = csv_path.with_suffix(csv_path.suffix + ".tmp")
        yaml_tmp = controllers_yaml.with_suffix(controllers_yaml.suffix + ".tmp")
        written_files = []

        try:
            with open(config_tmp, "w", encoding="utf-8") as f:
                yaml.dump(config_data, f, default_flow_style=False,
                          allow_unicode=True, sort_keys=False)
            written_files.append(config_tmp)

            with zipfile.ZipFile(args.zip_path) as zf:
                with zf.open(members["model"]) as src, onnx_tmp.open("wb") as dst:
                    shutil.copyfileobj(src, dst)
                written_files.append(onnx_tmp)

                with zf.open(members["trajectory"]) as src, csv_tmp.open("wb") as dst:
                    shutil.copyfileobj(src, dst)
                written_files.append(csv_tmp)

            # 读入 yaml → 内存修改 → 写临时文件
            config_rel = f"controllers/{config_name}"
            data = read_controllers_yaml(controllers_yaml)
            controllers_list = data.setdefault("controllers", [])
            if not isinstance(controllers_list, list):
                fail(f"{controllers_yaml} field 'controllers' must be a list")
            if args.force:
                # 覆盖导入时先移除所有同名注册，再追加唯一的新注册，避免每次
                # force 导入都在 controller_manager.yaml 中累积重复项。
                controllers_list[:] = [
                    item for item in controllers_list
                    if not (isinstance(item, dict)
                            and item.get("name") == controller_name)
                ]
            controllers_list.append({
                "name": controller_name,
                "type": "GenericRLController",
                "config": config_rel,
                "enabled": True,
            })
            with open(yaml_tmp, "w", encoding="utf-8") as f:
                yaml.dump(data, f, default_flow_style=False,
                          allow_unicode=True, sort_keys=False)
            written_files.append(yaml_tmp)

            # 全部写入成功 → 原子 rename
            os.replace(config_tmp, config_path)
            os.replace(onnx_tmp, onnx_path)
            os.replace(csv_tmp, csv_path)
            os.replace(yaml_tmp, controllers_yaml)

        except Exception:
            # 清理所有临时文件
            for tmp in written_files:
                try:
                    os.unlink(tmp)
                except OSError:
                    pass
            raise

        if args.json:
            result = {
                "code": 0,
                "robot_version": robot_version,
                "dance_name": display_name,
                "controller": controller_name,
                "config_path": str(config_path),
                "onnx_path": str(onnx_path),
                "csv_path": str(csv_path),
            }
            print(json.dumps(result, ensure_ascii=False))
            return 0

        print("Import complete.")
        print(f"  ROBOT_VERSION: {robot_version}")
        print(f"  Dance name:  {display_name}")
        print(f"  Controller:  {controller_name}")
        print(f"  Config: {config_path}")
        print(f"  ONNX: {onnx_path}")
        print(f"  CSV: {csv_path}")
        return 0

    except ImportErrorWithHint as exc:
        if args.json:
            print(json.dumps(
                {"code": 2, "error": str(exc), "error_kind": classify_error(str(exc))},
                ensure_ascii=False,
            ))
        else:
            print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
