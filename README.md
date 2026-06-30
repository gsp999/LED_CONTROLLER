# LED_CONTROLLER 启动说明

这是一个 ROS 2 节点，用 action 控制 WS2811 LED 灯带。硬件方案是 FT232H USB 转 SPI，当前默认配置为硬件模式。

## 1. 构建

进入仓库目录：

```bash
cd /home/gsp/LED_CONTROLLER
```

如果当前终端启用了 conda，先退出 conda，并清掉 conda 留在动态库搜索路径里的库：

```bash
conda deactivate
unset LD_LIBRARY_PATH
```

使用系统 ROS 和系统 Python 构建：

```bash
source /opt/ros/jazzy/setup.bash
rm -rf build install log
colcon build --packages-select led_controller --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

普通 ROS 2 环境也可以这样构建：

```bash
colcon build --packages-select led_controller
source install/setup.bash
```

## 2. 硬件模式启动

先安装 FT232H/NeoPixel 相关 Python 库：

```bash
/usr/bin/python3 -m pip install --user adafruit-blinka adafruit-circuitpython-neopixel-spi
```

如果系统提示 Python 环境受保护，可以使用：

```bash
/usr/bin/python3 -m pip install --user --break-system-packages adafruit-blinka adafruit-circuitpython-neopixel-spi
```

安装后检查：

```bash
BLINKA_FT232H=1 /usr/bin/python3 -c "import board; import neopixel_spi; print('ok')"
```

把 `config/led_controller.yaml` 改成：

```yaml
backend: "hardware"
```

如果改过配置或代码，先重新构建一次：

```bash
colcon build --packages-select led_controller --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

启动节点。`BLINKA_FT232H=1` 必须设置在启动节点的这个终端里：

```bash
cd /home/gsp/LED_CONTROLLER
conda deactivate
unset LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export BLINKA_FT232H=1
ros2 launch led_controller led_controller.launch.py
```

看到下面日志，说明硬件后端已经启动：

```text
LED action server ready: action=/led_pattern, backend=hardware, leds=60
```

接线建议：

- FT232H `SCK` 接 WS2811 灯带 `DIN`
- `DIN` 前串一个 330-500 欧姆电阻
- FT232H `GND`、灯带电源 `GND`、外部电源 `GND` 必须共地
- 灯带电源根据你的灯带选择 5V 或 12V
- 不要直接从 FT232H 给整条灯带供电

## 3. 发送灯效 action

另开一个终端，加载同一个 ROS 工作空间：

```bash
cd /home/gsp/LED_CONTROLLER
conda deactivate
unset LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

然后发送 action。

action 名称：

```text
/led_pattern
```

action 类型：

```text
led_controller/action/LedPattern
```

## 4. 摄像头识别状态灯效

下面 7 个状态专门用于摄像头识别。它们不依赖整条灯带的位置或方向，即使摄像头只能看到灯带的一小段，也能通过颜色和时间节奏区分。场馆有环境灯时，不建议用纯常亮状态，所以 7 个状态都带有明确节奏编码。

识别时建议先固定：

- `brightness: 0.5`
- `speed: 1.0`
- `duration: 0.0`

`duration: 0.0` 表示持续显示，直到调用 `/cancel_led_pattern`。

状态 1，绿色单短闪：

```bash
ros2 action send_goal /led_pattern led_controller/action/LedPattern \
"{pattern: state_1_green_single_pulse, color: [0, 0, 0], secondary_color: [0, 0, 0], brightness: 0.5, speed: 1.0, duration: 0.0}" \
--feedback
```

状态 2，蓝色慢呼吸：

```bash
ros2 action send_goal /led_pattern led_controller/action/LedPattern \
"{pattern: state_2_blue_slow_pulse, color: [0, 0, 0], secondary_color: [0, 0, 0], brightness: 0.5, speed: 1.0, duration: 0.0}" \
--feedback
```

状态 3，黄色双闪：

```bash
ros2 action send_goal /led_pattern led_controller/action/LedPattern \
"{pattern: state_3_yellow_slow_blink, color: [0, 0, 0], secondary_color: [0, 0, 0], brightness: 0.5, speed: 1.0, duration: 0.0}" \
--feedback
```

状态 4，红色四连快闪：

```bash
ros2 action send_goal /led_pattern led_controller/action/LedPattern \
"{pattern: state_4_red_fast_blink, color: [0, 0, 0], secondary_color: [0, 0, 0], brightness: 0.5, speed: 1.0, duration: 0.0}" \
--feedback
```

状态 5，紫红短长双闪：

```bash
ros2 action send_goal /led_pattern led_controller/action/LedPattern \
"{pattern: state_5_magenta_double_pulse, color: [0, 0, 0], secondary_color: [0, 0, 0], brightness: 0.5, speed: 1.0, duration: 0.0}" \
--feedback
```

状态 6，橙色五连闪：

```bash
ros2 action send_goal /led_pattern led_controller/action/LedPattern \
"{pattern: state_6_orange_five_pulse, color: [0, 0, 0], secondary_color: [0, 0, 0], brightness: 0.5, speed: 1.0, duration: 0.0}" \
--feedback
```

状态 7，白色心跳三段闪：

```bash
ros2 action send_goal /led_pattern led_controller/action/LedPattern \
"{pattern: state_7_white_heartbeat, color: [0, 0, 0], secondary_color: [0, 0, 0], brightness: 0.45, speed: 1.0, duration: 0.0}" \
--feedback
```

推荐识别方法：

- 先用 HSV 或 RGB 阈值分颜色：绿、蓝、黄、红、紫、橙、白
- 再用 ROI 中的亮度时间序列区分节奏：单短闪、呼吸、双闪、四连快闪、短长双闪、五连闪、心跳三段闪
- 摄像头 ROI 只需要框住能看到的那一小段灯带
- 如果画面过曝，优先降低 `brightness`，尤其是白色状态

取消当前状态并关灯：

```bash
ros2 service call /cancel_led_pattern std_srvs/srv/Trigger {}
```

## 5. 摄像头识别验证工具

安装 OpenCV：

```bash
sudo apt install python3-opencv
```

重新构建后启动识别工具：

```bash
colcon build --packages-select led_controller --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 run led_controller led_state_camera_recognizer --camera 0
```

识别工具默认会尝试把摄像头亮度调低到 `0.1`，并把曝光设置到 `-8`，减少 LED 过曝。如果还太亮，可以继续调低亮度或曝光：

```bash
ros2 run led_controller led_state_camera_recognizer --camera 0 --camera-brightness 0.05 --exposure -10
```

如果你的摄像头不接受曝光参数，可以只调亮度：

```bash
ros2 run led_controller led_state_camera_recognizer --camera 0 --camera-brightness 0.05
```

识别器默认会使用 6 秒亮度历史，并在闪烁熄灭间隙保持上一次可信状态 1.2 秒，避免灯灭的一瞬间跳成其它状态。如果状态切换显示太慢，可以缩短保持时间：

```bash
ros2 run led_controller led_state_camera_recognizer --camera 0 --hold-seconds 0.5
```

窗口按键：

- `s`: 框选摄像头里能看到的灯带 ROI
- `r`: 恢复使用整张画面
- `q`: 退出

如果你已经知道 ROI，可以直接传入：

```bash
ros2 run led_controller led_state_camera_recognizer --camera 0 --roi 320,220,260,80
```

识别工具会在画面左上角显示：

- 当前识别到的状态名
- 颜色判断
- HSV 均值
- 最近 6 秒内的脉冲数量
- 置信度

建议测试流程：

1. 先启动 LED 节点。
2. 再启动 `led_state_camera_recognizer`。
3. 按 `s` 框住能看到的那一小段灯带。
4. 依次发送 7 个 `state_*` action。
5. 观察画面左上角识别出的状态是否稳定。

如果识别不稳定，优先调整：

- 降低 LED action 的 `brightness`
- 让 ROI 只框住灯带，少框背景
- 避免摄像头画面过曝
- 固定摄像头曝光和白平衡

## 6. 普通灯效示例

彩虹灯效：

```bash
ros2 action send_goal /led_pattern led_controller/action/LedPattern \
"{pattern: rainbow, color: [255, 0, 0], secondary_color: [0, 0, 255], brightness: 0.35, speed: 1.0, duration: 8.0}" \
--feedback
```

流星灯效：

```bash
ros2 action send_goal /led_pattern led_controller/action/LedPattern \
"{pattern: comet, color: [0, 180, 255], secondary_color: [0, 0, 0], brightness: 0.6, speed: 1.5, duration: 10.0}" \
--feedback
```

呼吸灯：

```bash
ros2 action send_goal /led_pattern led_controller/action/LedPattern \
"{pattern: breathe, color: [255, 80, 20], secondary_color: [0, 0, 0], brightness: 0.5, speed: 1.0, duration: 6.0}" \
--feedback
```

警示闪烁：

```bash
ros2 action send_goal /led_pattern led_controller/action/LedPattern \
"{pattern: police, color: [255, 0, 0], secondary_color: [0, 0, 255], brightness: 0.7, speed: 1.2, duration: 8.0}" \
--feedback
```

随机闪电爆闪：

```bash
ros2 action send_goal /led_pattern led_controller/action/LedPattern \
"{pattern: lightning, color: [180, 220, 255], secondary_color: [0, 0, 0], brightness: 0.8, speed: 1.0, duration: 10.0}" \
--feedback
```

分段扫描闪光：

```bash
ros2 action send_goal /led_pattern led_controller/action/LedPattern \
"{pattern: scanner_flash, color: [255, 180, 40], secondary_color: [0, 0, 0], brightness: 0.7, speed: 1.3, duration: 10.0}" \
--feedback
```

如果希望某个灯效一直运行，把 `duration` 设置为 `0.0`。持续运行的 action 需要手动 cancel：

```bash
ros2 action send_goal /led_pattern led_controller/action/LedPattern \
"{pattern: lightning, color: [180, 220, 255], secondary_color: [0, 0, 0], brightness: 0.8, speed: 1.0, duration: 0.0}" \
--feedback
```

取消当前灯效并关灯：

```bash
ros2 service call /cancel_led_pattern std_srvs/srv/Trigger {}
```

这个命令适合取消 `duration: 0.0` 的持续闪光。取消后 action 会返回 `CANCELED`。

## 7. 模拟模式排查

如果不接灯带，只想确认 action 通信和灯效逻辑，可以把 `config/led_controller.yaml` 改成：

```yaml
backend: "simulate"
```

重新构建并启动：

```bash
colcon build --packages-select led_controller --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch led_controller led_controller.launch.py
```

看到下面日志，说明 action server 已经以模拟模式启动：

```text
LED action server ready: action=/led_pattern, backend=simulate, leds=60
```

模拟模式不会亮灯，但收到 action 后会在启动终端打印 `sim frame=...`。

## 8. 环境变量说明

这里有两个看起来相反的操作，其实作用完全不同：

- `unset LD_LIBRARY_PATH`: 清掉当前 shell 里残留的动态库搜索路径。之前 conda 的 `/home/gsp/miniconda3/lib` 混进来后，会让系统的 `/usr/bin/cmake` 加载 conda 里的 `libcurl`、`libtinfo`，导致 colcon 误判 CMake 版本或输出很多库版本警告。
- `export BLINKA_FT232H=1`: 设置 Adafruit Blinka 需要的硬件选择开关。它告诉 `board`/`neopixel_spi` 使用 FT232H 作为 GPIO/SPI 后端。这个变量要在启动 `ros2 launch` 的终端里设置，因为 LED 节点进程是从这个终端继承环境变量的。

简单说：`unset LD_LIBRARY_PATH` 是为了避开 conda 污染；`export BLINKA_FT232H=1` 是为了启用 FT232H 硬件。

## 9. 支持的灯效

摄像头识别状态：

- `state_1_green_single_pulse`: 绿色单短闪
- `state_2_blue_slow_pulse`: 蓝色慢呼吸
- `state_3_yellow_slow_blink`: 黄色双闪
- `state_4_red_fast_blink`: 红色四连快闪
- `state_5_magenta_double_pulse`: 紫红短长双闪
- `state_6_orange_five_pulse`: 橙色五连闪
- `state_7_white_heartbeat`: 白色心跳三段闪

普通灯效：

- `solid`: 常亮
- `blink`: 闪烁
- `breathe`: 呼吸灯
- `wipe`: 从头扫亮
- `rainbow`: 彩虹
- `theater_chase`: 剧院追逐灯
- `comet`: 流星拖尾
- `sparkle`: 随机闪烁
- `police`: 红蓝警示
- `lightning`: 随机闪电爆闪
- `scanner_flash`: 分段扫描后全灯闪烁
- `color_cycle`: 多色循环

`duration` 说明：

- `duration > 0`: 运行指定秒数后自动结束
- `duration = 0`: 一直运行，直到取消 action

## 10. 配置参数

在 `config/led_controller.yaml` 中修改：

```yaml
led_count: 60              # 灯珠数量
backend: "hardware"        # hardware 或 simulate
pixel_order: "BRG"         # 当前 WS2811 灯带使用 BRG
default_brightness: 0.4    # 默认亮度，0.0 到 1.0
action_name: "led_pattern" # action 名称
cancel_service_name: "cancel_led_pattern" # 取消当前灯效的 service 名称
frame_rate: 40.0           # 动画刷新率
```

如果 ROS launch 提示不能写入 `~/.ros/log`，可以临时把日志目录放到 `/tmp`：

```bash
mkdir -p /tmp/ros_logs
ROS_LOG_DIR=/tmp/ros_logs ros2 launch led_controller led_controller.launch.py
```
