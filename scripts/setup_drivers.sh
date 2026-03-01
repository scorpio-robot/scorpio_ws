#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PROGRAMS_DIR="$HOME/Programs"
JOBS=$(nproc)

# ── 工具函数 ─────────────────────────────────────────────────
ok()  { echo -e "\e[32m✓ $1\e[0m"; echo ""; }
log() { echo "  -> $1"; }
section() { echo "[$1] $2"; }

# 通用 cmake 库安装：clone（可选）+ cmake build + sudo make install
cmake_install() {
    local name="$1"
    local dir="$2"
    local clone_cmd="$3"   # 若为空则跳过 clone

    if [ -n "$clone_cmd" ]; then
        if [ -d "$dir" ]; then
            log "$name 目录已存在,跳过 clone"
        else
            log "Clone $name..."
            mkdir -p "$PROGRAMS_DIR"
            eval "$clone_cmd"
        fi
    fi

    log "编译并安装 $name..."
    cmake -S "$dir" -B "$dir/build"
    cmake --build "$dir/build" -j"$JOBS"
    sudo cmake --install "$dir/build"
}

# ─────────────────────────────────────────────────────────────
echo "================================================================"
echo "Scorpio 依赖自动安装脚本"
echo "================================================================"
echo "本脚本将自动安装:"
echo "  1. Sophus"
echo "  2. nanoflann"
echo "  3. Intel RealSense SDK 2.0 + ROS2 Wrapper"
echo "  4. YDLidar SDK"
echo "  5. YDLidar udev 规则"
echo ""
echo "需要 sudo 权限,请准备输入密码。"
echo "================================================================"
echo ""

# ============================================================
section "1/5" "安装 Sophus..."
cmake_install "Sophus" "$PROGRAMS_DIR/Sophus" \
    "git clone https://github.com/strasdat/Sophus.git -b 1.24.6 --depth=1 \"$PROGRAMS_DIR/Sophus\""
ok "Sophus 安装完成"

# ============================================================
section "2/5" "安装 nanoflann..."
cmake_install "nanoflann" "$PROGRAMS_DIR/nanoflann" \
    "git clone https://github.com/jlblancoc/nanoflann.git -b v1.8.0 --depth=1 \"$PROGRAMS_DIR/nanoflann\""
ok "nanoflann 安装完成"

# ============================================================
section "3/5" "安装 Intel RealSense SDK 2.0..."

log "注册 RealSense GPG 密钥..."
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
    | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

log "添加 RealSense 软件源..."
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update

log "安装 librealsense2 + ROS2 Wrapper..."
sudo apt-get install -y librealsense2-dkms librealsense2-utils ros-$ROS_DISTRO-realsense2-*

ok "RealSense SDK 安装完成"

# ============================================================
section "4/5" "安装 YDLidar SDK..."
cmake_install "YDLidar-SDK" "$PROGRAMS_DIR/YDLidar-SDK" \
    "git clone https://github.com/YDLIDAR/YDLidar-SDK.git \"$PROGRAMS_DIR/YDLidar-SDK\""
ok "YDLidar SDK 安装完成"

# ============================================================
section "5/5" "配置 YDLidar udev 规则..."

declare -A UDEV_RULES=(
    ["/etc/udev/rules.d/99-ydlidar.rules"]='KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"'
    ["/etc/udev/rules.d/99-ydlidar-V2.rules"]='KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"'
    ["/etc/udev/rules.d/99-ydlidar-2303.rules"]='KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"'
)

log "写入 udev 规则..."
for file in "${!UDEV_RULES[@]}"; do
    echo "${UDEV_RULES[$file]}" | sudo tee "$file" > /dev/null
done

log "重载 udev 服务..."
sudo udevadm control --reload-rules && sudo udevadm trigger

log "将用户 $(whoami) 添加到 dialout 组..."
sudo usermod -aG dialout "$(whoami)"

ok "YDLidar udev 规则已设置"

# ============================================================
echo "================================================================"
echo -e "\e[32m✓ 所有依赖安装完成!\e[0m"
echo "================================================================"
echo ""
echo "Tips:"
echo "  1. 请注销并重新登录以使 dialout 组权限生效"
echo "  2. 然后继续执行 README 的 2.3 Build 步骤"
echo ""
