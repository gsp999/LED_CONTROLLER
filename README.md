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

彩虹灯效：

```bash
ros2 action send_goal /led_pattern led_controller/action/LedPattern \
"{pattern: rainbow, color: [255, 0, 0], secondary_color: [0, 0, 255], brightness: 0.35, speed: 1.0, duration: 8.0, loop: false}" \
--feedback
```

流星灯效：

```bash
ros2 action send_goal /led_pattern led_controller/action/LedPattern \
"{pattern: comet, color: [0, 180, 255], secondary_color: [0, 0, 0], brightness: 0.6, speed: 1.5, duration: 10.0, loop: true}" \
--feedback
```

呼吸灯：

```bash
ros2 action send_goal /led_pattern led_controller/action/LedPattern \
"{pattern: breathe, color: [255, 80, 20], secondary_color: [0, 0, 0], brightness: 0.5, speed: 1.0, duration: 6.0, loop: false}" \
--feedback
```

警示闪烁：

```bash
ros2 action send_goal /led_pattern led_controller/action/LedPattern \
"{pattern: police, color: [255, 0, 0], secondary_color: [0, 0, 255], brightness: 0.7, speed: 1.2, duration: 8.0, loop: false}" \
--feedback
```

## 4. 模拟模式排查

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

## 5. 环境变量说明

这里有两个看起来相反的操作，其实作用完全不同：

- `unset LD_LIBRARY_PATH`: 清掉当前 shell 里残留的动态库搜索路径。之前 conda 的 `/home/gsp/miniconda3/lib` 混进来后，会让系统的 `/usr/bin/cmake` 加载 conda 里的 `libcurl`、`libtinfo`，导致 colcon 误判 CMake 版本或输出很多库版本警告。
- `export BLINKA_FT232H=1`: 设置 Adafruit Blinka 需要的硬件选择开关。它告诉 `board`/`neopixel_spi` 使用 FT232H 作为 GPIO/SPI 后端。这个变量要在启动 `ros2 launch` 的终端里设置，因为 LED 节点进程是从这个终端继承环境变量的。

简单说：`unset LD_LIBRARY_PATH` 是为了避开 conda 污染；`export BLINKA_FT232H=1` 是为了启用 FT232H 硬件。

## 6. 支持的灯效

- `solid`: 常亮
- `blink`: 闪烁
- `breathe`: 呼吸灯
- `wipe`: 从头扫亮
- `rainbow`: 彩虹
- `theater_chase`: 剧院追逐灯
- `comet`: 流星拖尾
- `sparkle`: 随机闪烁
- `police`: 红蓝警示
- `color_cycle`: 多色循环

## 7. 配置参数

在 `config/led_controller.yaml` 中修改：

```yaml
led_count: 60              # 灯珠数量
backend: "hardware"        # hardware 或 simulate
pixel_order: "GRB"         # 常见 WS2811/WS2812 是 GRB
default_brightness: 0.4    # 默认亮度，0.0 到 1.0
action_name: "led_pattern" # action 名称
frame_rate: 40.0           # 动画刷新率
```

如果 ROS launch 提示不能写入 `~/.ros/log`，可以临时把日志目录放到 `/tmp`：

```bash
mkdir -p /tmp/ros_logs
ROS_LOG_DIR=/tmp/ros_logs ros2 launch led_controller led_controller.launch.py
```
