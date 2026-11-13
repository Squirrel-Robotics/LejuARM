#!/usr/bin/env python3
"""Check the archived S17 files and runtime without starting robot processes."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
ACT = ROOT / "integrations/act_heft_relay"
NATIVE_ROOT = Path("/home/lab/Lejulab_ARM")


def verify_hash(path, expected):
    if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
        raise ValueError(f"SHA256 mismatch: {path.relative_to(ROOT)}")


def check_archive(snapshot):
    update = next(x for x in snapshot["code_updates"] if x["date"] == "2026-11-13")
    for filename, sha in update["file_sha256"].items():
        verify_hash(ROOT / filename, sha)
    baseline = next(x for x in snapshot["integrations"] if x["archive_path"] == "integrations/act_heft_relay")
    for filename, sha in baseline["file_sha256"].items():
        verify_hash(ACT / filename, sha)

    versions = [ACT, *(ROOT / entry["archive_path"] for entry in update["act_versions"])]
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    for directory in versions:
        cfg = json.loads((directory / "config.json").read_text())
        for item in (cfg, *cfg.get("sequence_sides", {}).values()):
            if Path(item["native_root"]) != NATIVE_ROOT:
                raise ValueError(f"Unexpected native_root: {directory.name}")
            for name, sha in item["native_source_sha256"].items():
                verify_hash(ROOT / Path(name).relative_to(NATIVE_ROOT), sha)
        subprocess.run(["bash", str(directory / "relay.sh"), "--help"],
                       env=env, check=True, stdout=subprocess.DEVNULL)
    print(f"Archive OK: {len(update['file_sha256'])} updated files, "
          f"{len(versions)} ACT versions including the baseline")


def check_lower():
    if platform.machine() != "aarch64":
        raise ValueError("The --lower checks require an ARM64 lower computer.")
    ros = Path("/opt/ros/humble/lib")
    dds = ROOT / "src/lejusdk/3rd_party/aarch64/cyclonedds-0.10.2/lib"
    lib = ROOT / "installed/lib"
    base_env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    link_env = dict(base_env, LD_LIBRARY_PATH=":".join(map(str, (lib, ros, ros / "aarch64-linux-gnu"))))
    for name in ("run_rl_controller", "leju-hardware", "lejusdk_recorder"):
        path = ROOT / "installed/bin" / name
        if not os.access(path, os.X_OK):
            raise ValueError(f"Not executable: {path.relative_to(ROOT)}")
        result = subprocess.run(["ldd", str(path)], check=True, text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=link_env)
        if "not found" in result.stdout:
            raise ValueError(f"{name} has unresolved dependencies:\n{result.stdout}")
    python_env = dict(base_env, CYCLONEDDS_LIB_DIR=str(dds), LD_LIBRARY_PATH=f"{dds}:{lib}")
    for key in ("PYTHONHOME", "PYTHONPATH", "PYTHONNOUSERSITE"):
        python_env.pop(key, None)
    subprocess.run(["/usr/bin/python3", "-c",
                    "import cyclonedds.domain, cyclonedds.idl, mcap.reader, lz4.frame, zstandard, yaml"],
                   check=True, env=python_env)
    print("ARM64 runtime OK: controller, hardware and recorder libraries; Python DDS and MCAP imports")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lower", action="store_true", help="Also check ARM64 runtime dependencies")
    args = parser.parse_args()
    try:
        check_archive(json.loads((ROOT / "SOURCE_SNAPSHOT.json").read_text()))
        if args.lower:
            check_lower()
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as exc:
        sys.exit(f"Recovery check failed: {exc}")


if __name__ == "__main__":
    main()
