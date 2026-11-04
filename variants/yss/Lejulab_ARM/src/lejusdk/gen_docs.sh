#!/bin/bash
# Script to generate documentation for LejuSDK
# 用于生成 LejuSDK 的 API 文档

SCRIPT_DIR=$(dirname "$(realpath "$0")")
PROJECT_DIR=$(realpath "$SCRIPT_DIR")
DOCS_DIR="$PROJECT_DIR/docs"
OUTPUT_DIR="$DOCS_DIR/output"
BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
VERSION=$(git describe --tags --always 2>/dev/null || echo "0.0.0")

# Exit on error
set -e

# 颜色定义
RED='\033[31m'
GREEN='\033[32m'
YELLOW='\033[33m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

exit_with_failure() {
    if [ "$1" ]; then
        log_error "$1"
    fi
    post_process
    exit 1
}

check_and_format_version() {
    local branch="$1"
    local -n __version_ref="$2"

    if [ -z "$__version_ref" ]; then
        __version_ref="0.0.0"
    fi

    # 通过git获取(e.g.): 1.1.0-324-g792046c35, 1.2.0 ...
    # Remove the hash part (g followed by alphanumeric characters) from the version
    local version1=$(echo "$__version_ref" | sed 's/-g[0-9a-f]\+//')

    if [ "$branch" == "master" ]; then
        version1=$(echo "$version1" | sed 's/-/.post/g')
    elif [ "$branch" == "beta" ]; then
        version1=$(echo "$version1" | sed 's/-/b/g')
        if [[ ! "$version1" == *"b"* ]]; then
            version1="${version1}b0"
        fi
    else
        version1=$(echo "$version1" | sed 's/-/a/g')
        if [[ ! "$version1" == *"a"* ]]; then
            version1="${version1}a0"
        fi
    fi

    __version_ref="$version1"
}

install_dependencies() {
    log_info "Checking and installing dependencies..."

    # 检查 doxygen
    if ! command -v doxygen &> /dev/null; then
        log_warn "Doxygen not found, installing..."
        sudo apt-get update && sudo apt-get install -y doxygen
    fi

    # 检查并安装 Python 依赖 (版本兼容 Python 3.8)
    pip install --quiet --force-reinstall "sphinx>=4.0,<6.0" "sphinx-rtd-theme<2.0" breathe "sphinx-markdown-builder==0.5.5"
}

generate_doxygen() {
    log_info "Generating Doxygen XML..."

    if [ ! -f "$DOCS_DIR/Doxyfile" ]; then
        exit_with_failure "Doxyfile not found in $DOCS_DIR"
    fi

    cd "$DOCS_DIR"
    doxygen Doxyfile

    log_info "Doxygen XML generated in $OUTPUT_DIR/xml"
}

generate_sphinx() {
    log_info "Generating Sphinx HTML documentation..."

    if [ ! -f "$DOCS_DIR/conf.py" ]; then
        exit_with_failure "conf.py not found in $DOCS_DIR"
    fi

    cd "$DOCS_DIR"
    sphinx-build -b html -D language=zh_CN . "$OUTPUT_DIR/html"

    log_info "HTML documentation generated in $OUTPUT_DIR/html"
}

generate_markdown() {
    log_info "Generating Sphinx Markdown documentation..."

    cd "$DOCS_DIR"
    sphinx-build -b markdown . "$OUTPUT_DIR/markdown"

    log_info "Markdown documentation generated in $OUTPUT_DIR/markdown"
}

pre_process() {
    check_and_format_version "$BRANCH" VERSION
    export LEJUSDK_VERSION="$VERSION"
    log_info "Building documentation for LejuSDK version: $VERSION (branch: $BRANCH)"

    # 清理构建缓存
    log_info "Cleaning build cache..."
    rm -rf "$OUTPUT_DIR"
    rm -rf "$DOCS_DIR/.doctrees"
}

post_process() {
    unset LEJUSDK_VERSION
}

# Main
pre_process
install_dependencies
generate_doxygen
generate_sphinx
generate_markdown
post_process

log_info "Documentation generation completed!"
echo ""
echo "HTML docs: $OUTPUT_DIR/html/index.html"
echo "Markdown docs: $OUTPUT_DIR/markdown/"
