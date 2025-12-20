# scorpio_ws

## 1. Overview

scorpio_workspace 是天蝎座机器人的上位机 ROS 工作空间，包含仿真、底盘串口通信、传感器驱动、导航等模块。

## 2. Quick Start

### 2.1 Setup Environment

Ubuntu 22.04: [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

> [!NOTE]
> 由于使用了 RGLGazeboPlugin 用于仿真 mid360 点云,仿真包仅可运行在带 Nvidia GPU 的电脑

### 2.2 Create Workspace

### 2.2.1 Clone

```sh
pip3 install vcs2l
pip3 install xmacro
```

克隆工作空间仓库并进入目录

```sh
git clone https://github.com/scorpio-robot/scorpio_ws.git
cd scorpio_ws
```

导入依赖仓库并安装系统依赖

```sh
./setup.sh
```

安装传感器驱动和 udev 规则

```sh
./scripts/setup_drivers.sh
```

> [!NOTE]
> `dependencies.repos` 文件已包含所有模块所依赖的仓库地址，无需手动查阅子模块的 README 手动克隆依赖。

> [!TIP]
> 使用命令 `vcs pull ./src` 可更新所有子模块。

#### 2.2.3 Build

```sh
./build.sh
```

### 2.3 Running

如启动仿真环境，请先取消注释 [dependencies.repos](dependencies.repos) 中的 `scorpio_simulator` 和 `rgl_gazebo_plugin` 部分，然后再次运行 `./setup.sh`, `./build.sh`

```sh
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```

> [!NOTE]
> **注意：需要点击 Gazebo 左下角橙红色的 `启动` 按钮**

启动实车

```sh
ros2 launch scorpio_bringup bringup_hardware_launch.py
```

控制机器人移动

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 维护者及开源许可证

Maintainer: Lihan Chen, <lihanchen2004@163.com>

scorpio is provided under Apache License 2.0.
