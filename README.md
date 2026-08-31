# Nero Control Studio

Nero 七轴臂的桌面示教盒：关节点动（步进 / 连续）、软件路点示教、固件拖动示教。经 SocketCAN + [pyAgxArm](https://github.com/agilexrobotics/pyAgxArm) 直连机械臂，**不经过 ROS**。

## CAN 独占

本进程会打开 `can_left` / `can_right`（或 `can0`）。同一条 CAN **同时只能被一个进程占用**。启动前必须关掉：

- `ros2 launch dual_nero_bringup real.launch.py`
- `publish_nero_joint_states.py`
- `test_dual_nero_comm.py` / `nero_enable_disable.py`
- 其它直接打开该接口的程序

双臂通道绑定见 [`CAN_name_binding_for_USB/README.md`](../ros2_ws/dev_process_doc/CAN_name_binding_for_USB/README.md)：

```bash
cd /home/bci/Projects/nero/ros2_ws/dev_process_doc/CAN_name_binding_for_USB
sudo ./bind_can_left_right.sh apply
```

连接失败时，界面会提示接口不存在或已被占用。

## 安装

需要本机已有 pyAgxArm（实验室路径）：

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e /home/bci/Projects/pyAgxArm
pip install -e ".[dev]"
```

## 启动

```bash
source .venv/bin/activate
python -m nero_control_studio --channel can_left
# 或
nero-control-studio --channel can_right
```

可选参数：`--channel`（默认 `can_left`）、`--interface`（默认 `socketcan`）。

## 使用

操作步骤、点动 / 示教 / 急停见 [使用说明.md](使用说明.md)。

关闭窗口时会尽量退出零力拖动、失能并断开 CAN。Nero 无抱闸，界面不提示抱闸。

## 测试（无真机）

本机若已 `source /opt/ros/jazzy/setup.bash`，ROS 会注入 pytest 插件。请关掉自动加载：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest
```
