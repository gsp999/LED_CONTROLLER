# LED_CONTROLLER

ROS 2 Jazzy 工程，用 FT232H 控制 WS2811 灯带，并提供摄像头识别灯组状态的 action。

当前有两个 action：

- `/led_pattern`: 控制灯组颜色、亮度、闪烁频率。
- `/recognize_led_state`: 用摄像头自动识别 3 组灯的颜色和闪烁频率。

## 1. 安装依赖

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-pip python3-opencv
```

硬件控制 FT232H 还需要：

```bash
/usr/bin/python3 -m pip install --user adafruit-blinka adafruit-circuitpython-neopixel-spi
```

如果 pip 提示系统 Python 受保护，用：

```bash
/usr/bin/python3 -m pip install --user --break-system-packages adafruit-blinka adafruit-circuitpython-neopixel-spi
```

## 2. 配置

主要配置文件：

```text
config/led_controller.yaml
```

常用参数：

```yaml
led_count: 60
backend: "simulate"
pixel_order: "BRG"
default_brightness: 0.4
frame_rate: 40.0
```

说明：

- `led_count`: 可控组数量。你的 WS2811 是 6 个物理灯珠为 1 组。
- `backend`: `"simulate"` 是仿真；接 FT232H 硬件时改成 `"hardware"`。
- `pixel_order`: 当前灯带颜色顺序是 `"BRG"`，颜色不对就先改这里。
- `default_brightness`: 默认亮度，范围 `0.0-1.0`。

## 3. 构建

```bash
cd /home/gsp/LED_CONTROLLER
conda deactivate
unset LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
colcon build --packages-select led_controller --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

如果改过 `action/*.action`，用干净重建：

```bash
cd /home/gsp/LED_CONTROLLER
source /opt/ros/jazzy/setup.bash
rm -rf build install log
colcon build --packages-select led_controller --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

## 4. 启动控灯节点

仿真模式：

```bash
cd /home/gsp/LED_CONTROLLER
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch led_controller led_controller.launch.py
```

硬件模式：

先把 `config/led_controller.yaml` 里的 `backend` 改成 `"hardware"`，然后启动：

```bash
cd /home/gsp/LED_CONTROLLER
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export BLINKA_FT232H=1
ros2 launch led_controller led_controller.launch.py
```

## 5. Action 1：控制灯组

Action 名称：

```text
/led_pattern
```

Action 类型：

```text
led_controller/action/LedPattern
```

Goal 字段：

```text
groups          # 要控制的组号，[] 表示全部组
colors          # RGB，按 groups 顺序，每组 3 个数
brightnesses    # 亮度，1 个值表示所有组共用
blink_hz        # 闪烁频率，0.0 表示常亮
duration        # 秒；0.0 表示一直运行，直到取消
```

第 0 组红色常亮 5 秒：

```bash
ros2 action send_goal /led_pattern led_controller/action/LedPattern \
"{groups: [0], colors: [255, 0, 0], brightnesses: [0.5], blink_hz: [0.0], duration: 5.0}" \
--feedback
```

第 0、1、2 组分别红、绿、蓝，频率不同，一直运行：

```bash
ros2 action send_goal /led_pattern led_controller/action/LedPattern \
"{groups: [0, 1, 2], colors: [255, 0, 0, 0, 255, 0, 0, 0, 255], brightnesses: [0.5, 0.5, 0.5], blink_hz: [0.0, 1.0, 3.0], duration: 0.0}" \
--feedback
```

取消当前灯效并关灯：

```bash
ros2 service call /cancel_led_pattern std_srvs/srv/Trigger {}
```

## 6. Action 2：摄像头识别

先启动识别 action server：

```bash
cd /home/gsp/LED_CONTROLLER
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run led_controller led_camera_recognition_action_server.py
```

Action 名称：

```text
/recognize_led_state
```

Action 类型：

```text
led_controller/action/RecognizeLedState
```

发送识别请求：

```bash
ros2 action send_goal /recognize_led_state led_controller/action/RecognizeLedState \
"{camera_index: -1, duration: 5.0, expected_groups: 3, min_value: 55.0, min_saturation: 45.0, show_debug: true, state_map_path: ''}" \
--feedback
```

字段说明：

- `camera_index`: 摄像头编号。`-1` 表示自动扫描。
- `duration`: 识别持续时间，建议至少 `5.0` 秒。
- `expected_groups`: 要识别几组灯，当前一般填 `3`。
- `min_value`: HSV 亮度阈值，背景太亮就调高。
- `min_saturation`: HSV 饱和度阈值，背景彩色干扰多就调高。
- `show_debug`: 是否显示调试画面。
- `state_map_path`: 状态映射 JSON，不需要就填空字符串。

识别结果里重点看：

- `signature`: 识别签名，例如 `g0:red@0|g1:green@1|g2:blue@3`
- `group_colors`: 每组颜色
- `group_frequencies`: 每组闪烁频率
- `group_boxes`: 自动检测到的灯组位置框

状态映射示例：

```text
config/led_state_map.example.json
```

## 7. 硬件接线

- FT232H `SCK` 接 WS2811 `DIN`
- FT232H `GND`、灯带电源 `GND`、外部电源 `GND` 必须共地
- `DIN` 前建议串 330-500 欧姆电阻
- 灯带用外部电源供电，不要从 FT232H 给整条灯带供电

检查 FT232H：

```bash
lsusb
```

正常能看到 `0403:6014 FT232H`。

## 8. 常见问题

颜色不对：

```yaml
pixel_order: "BRG"
```

红绿蓝显示错位时，改 `config/led_controller.yaml` 里的 `pixel_order`。

构建时 conda 报奇怪错误：

```bash
conda deactivate
unset LD_LIBRARY_PATH
```

摄像头打不开：

```bash
ls -l /dev/video*
groups
```

如果用户不在 `video` 组，执行后重新登录：

```bash
sudo usermod -aG video $USER
```
