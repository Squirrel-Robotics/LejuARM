#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BINARY="${ROOT_DIR}/run_rl_controller.transport_head_down.aarch64"
TARGET="${ROOT_DIR}/installed/bin/run_rl_controller"
EXPECTED_SHA256="2ab4d6926e5746a329e2071c2f3571c68aa8eb65b3e5e971cc6c45bfbd06f043"
PATCHED_SHA256="bf690f6fc353cd469d66e18326142356e6c36683a5b7ea1e73645016a42444cb"

if [[ ! -f "${BINARY}" ]]; then
  echo "错误：未找到补丁二进制：${BINARY}" >&2
  exit 1
fi
if [[ ! -f "${TARGET}" ]]; then
  echo "错误：未找到开源仓库控制器：${TARGET}" >&2
  exit 1
fi

binary_sha256="$(sha256sum "${BINARY}" | awk '{print $1}')"
if [[ "${binary_sha256}" != "${PATCHED_SHA256}" ]]; then
  echo "错误：补丁二进制校验失败，未替换。" >&2
  exit 1
fi

target_sha256="$(sha256sum "${TARGET}" | awk '{print $1}')"
if [[ "${target_sha256}" == "${PATCHED_SHA256}" ]]; then
  echo "补丁已应用，未修改文件。"
  exit 0
fi
if [[ "${target_sha256}" != "${EXPECTED_SHA256}" ]]; then
  echo "错误：当前 run_rl_controller 不是适配的 OpenLET beta 原版，未替换。" >&2
  echo "当前 SHA256：${target_sha256}" >&2
  exit 1
fi

backup="${TARGET}.bak.$(date +%Y%m%d_%H%M%S)"
cp -p "${TARGET}" "${backup}"
install -m 755 "${BINARY}" "${TARGET}"

installed_sha256="$(sha256sum "${TARGET}" | awk '{print $1}')"
if [[ "${installed_sha256}" != "${PATCHED_SHA256}" ]]; then
  echo "错误：替换后校验失败，原文件备份位于：${backup}" >&2
  exit 1
fi

echo "已替换：${TARGET}"
echo "原文件备份：${backup}"
echo "请按原有方式重启控制器或手柄服务。"
