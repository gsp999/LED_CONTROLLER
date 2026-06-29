import colorsys
import math
import random
import time
from dataclasses import dataclass
from typing import Callable, Iterable, List, Sequence, Tuple

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from led_controller.action import LedPattern

Color = Tuple[int, int, int]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def clamp_color(values: Sequence[int], fallback: Color) -> Color:
    if len(values) < 3:
        return fallback
    return tuple(int(clamp(v, 0, 255)) for v in values[:3])


def scale_color(color: Color, brightness: float) -> Color:
    return tuple(int(channel * brightness) for channel in color)


def blend(a: Color, b: Color, amount: float) -> Color:
    amount = clamp(amount, 0.0, 1.0)
    return tuple(int(a[i] + (b[i] - a[i]) * amount) for i in range(3))


def hsv_color(hue: float, saturation: float = 1.0, value: float = 1.0) -> Color:
    r, g, b = colorsys.hsv_to_rgb(hue % 1.0, saturation, value)
    return int(r * 255), int(g * 255), int(b * 255)


class LedBackend:
    def __init__(self, led_count: int, brightness: float) -> None:
        self.led_count = led_count
        self.brightness = brightness
        self.name = 'unknown'

    def show(self, pixels: Sequence[Color]) -> None:
        raise NotImplementedError

    def clear(self) -> None:
        self.show([(0, 0, 0)] * self.led_count)


class SimulatedBackend(LedBackend):
    def __init__(self, led_count: int, brightness: float, logger) -> None:
        super().__init__(led_count, brightness)
        self.name = 'simulate'
        self._logger = logger
        self._frame = 0

    def show(self, pixels: Sequence[Color]) -> None:
        self._frame += 1
        if self._frame == 1 or self._frame % 40 == 0:
            lit = sum(1 for pixel in pixels if pixel != (0, 0, 0))
            preview = list(pixels[: min(8, len(pixels))])
            self._logger.info(f'sim frame={self._frame} lit={lit}/{self.led_count} preview={preview}')


class Ft232hSpiBackend(LedBackend):
    def __init__(self, led_count: int, brightness: float, pixel_order: str) -> None:
        super().__init__(led_count, brightness)
        self.name = 'hardware'
        import board
        import neopixel_spi

        order_name = pixel_order.upper()
        order = getattr(neopixel_spi, order_name, neopixel_spi.GRB)
        self._pixels = neopixel_spi.NeoPixel_SPI(
            board.SPI(),
            led_count,
            pixel_order=order,
            auto_write=False,
        )
        self._pixels.brightness = brightness

    def show(self, pixels: Sequence[Color]) -> None:
        for index, color in enumerate(pixels[: self.led_count]):
            self._pixels[index] = color
        self._pixels.show()


@dataclass
class PatternContext:
    goal_handle: object
    backend: LedBackend
    led_count: int
    frame_delay: float
    color: Color
    secondary_color: Color
    brightness: float
    speed: float
    duration: float
    loop: bool

    def publish_feedback(self, percent: float, step: int, message: str) -> None:
        feedback = LedPattern.Feedback()
        feedback.percent_complete = float(clamp(percent, 0.0, 100.0))
        feedback.current_step = int(step)
        feedback.message = message
        self.goal_handle.publish_feedback(feedback)


PatternFactory = Callable[[PatternContext], Iterable[List[Color]]]


class LedActionServer(Node):
    def __init__(self) -> None:
        super().__init__('led_action_server')
        self.declare_parameter('led_count', 60)
        self.declare_parameter('backend', 'simulate')
        self.declare_parameter('pixel_order', 'GRB')
        self.declare_parameter('default_brightness', 0.4)
        self.declare_parameter('action_name', 'led_pattern')
        self.declare_parameter('frame_rate', 40.0)

        self.led_count = int(self.get_parameter('led_count').value)
        self.frame_rate = float(self.get_parameter('frame_rate').value)
        self.default_brightness = float(self.get_parameter('default_brightness').value)
        backend_name = str(self.get_parameter('backend').value)
        pixel_order = str(self.get_parameter('pixel_order').value)
        action_name = str(self.get_parameter('action_name').value)

        self._backend = self._make_backend(backend_name, pixel_order)
        self._patterns = {
            'solid': self._solid,
            'blink': self._blink,
            'breathe': self._breathe,
            'wipe': self._wipe,
            'rainbow': self._rainbow,
            'theater_chase': self._theater_chase,
            'comet': self._comet,
            'sparkle': self._sparkle,
            'police': self._police,
            'color_cycle': self._color_cycle,
        }
        self._active_goal = False
        self._action_server = ActionServer(
            self,
            LedPattern,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info(
            f'LED action server ready: action=/{action_name}, backend={self._backend.name}, leds={self.led_count}'
        )

    def destroy_node(self) -> bool:
        self._backend.clear()
        self._action_server.destroy()
        return super().destroy_node()

    def _make_backend(self, backend_name: str, pixel_order: str) -> LedBackend:
        brightness = clamp(self.default_brightness, 0.0, 1.0)
        if backend_name == 'hardware':
            try:
                return Ft232hSpiBackend(self.led_count, brightness, pixel_order)
            except Exception as exc:
                self.get_logger().error(f'failed to initialize FT232H backend: {exc}')
                self.get_logger().warning('falling back to simulate backend')
        return SimulatedBackend(self.led_count, brightness, self.get_logger())

    def goal_callback(self, goal_request) -> GoalResponse:
        pattern = goal_request.pattern.strip().lower()
        if self._active_goal:
            self.get_logger().warning('rejecting goal because another LED pattern is active')
            return GoalResponse.REJECT
        if pattern not in self._patterns:
            self.get_logger().warning(f'rejecting unknown pattern: {goal_request.pattern}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('cancel requested')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        request = goal_handle.request
        pattern = request.pattern.strip().lower()
        brightness = self.default_brightness if request.brightness <= 0.0 else request.brightness
        ctx = PatternContext(
            goal_handle=goal_handle,
            backend=self._backend,
            led_count=self.led_count,
            frame_delay=1.0 / max(self.frame_rate, 1.0),
            color=clamp_color(request.color, (255, 255, 255)),
            secondary_color=clamp_color(request.secondary_color, (0, 0, 0)),
            brightness=clamp(brightness, 0.0, 1.0),
            speed=max(float(request.speed), 0.05),
            duration=max(float(request.duration), 0.0),
            loop=bool(request.loop),
        )

        result = LedPattern.Result()
        self._active_goal = True
        self.get_logger().info(
            f'goal accepted pattern={pattern} duration={ctx.duration:.2f}s loop={ctx.loop}'
        )

        started = self.get_clock().now()
        step = 0
        try:
            while rclpy.ok():
                for pixels in self._patterns[pattern](ctx):
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        self._backend.clear()
                        result.success = False
                        result.message = f'pattern {pattern} canceled'
                        return result

                    elapsed = (self.get_clock().now() - started).nanoseconds / 1_000_000_000.0
                    if ctx.duration > 0.0 and elapsed >= ctx.duration:
                        goal_handle.succeed()
                        self._backend.clear()
                        result.success = True
                        result.message = f'pattern {pattern} completed'
                        return result

                    self._backend.show([scale_color(pixel, ctx.brightness) for pixel in pixels])
                    percent = 0.0 if ctx.duration <= 0.0 else elapsed / ctx.duration * 100.0
                    if step % max(1, int(self.frame_rate / 4.0)) == 0:
                        ctx.publish_feedback(percent, step, f'running {pattern}')
                    step += 1
                    time.sleep(ctx.frame_delay / ctx.speed)

                if not ctx.loop and ctx.duration <= 0.0:
                    break

            goal_handle.succeed()
            self._backend.clear()
            result.success = True
            result.message = f'pattern {pattern} completed'
            return result
        except Exception as exc:
            self.get_logger().error(f'pattern {pattern} failed: {exc}')
            goal_handle.abort()
            self._backend.clear()
            result.success = False
            result.message = f'pattern {pattern} failed: {exc}'
            return result
        finally:
            self._active_goal = False

    def _solid(self, ctx: PatternContext) -> Iterable[List[Color]]:
        yield [ctx.color] * ctx.led_count

    def _blink(self, ctx: PatternContext) -> Iterable[List[Color]]:
        frames = max(1, int(self.frame_rate * 0.35))
        for on in (True, False):
            color = ctx.color if on else ctx.secondary_color
            for _ in range(frames):
                yield [color] * ctx.led_count

    def _breathe(self, ctx: PatternContext) -> Iterable[List[Color]]:
        frames = max(12, int(self.frame_rate * 2.0))
        for index in range(frames):
            wave = (1.0 - math.cos(index / frames * math.tau)) / 2.0
            color = blend(ctx.secondary_color, ctx.color, wave)
            yield [color] * ctx.led_count

    def _wipe(self, ctx: PatternContext) -> Iterable[List[Color]]:
        for index in range(ctx.led_count + 1):
            pixels = [ctx.secondary_color] * ctx.led_count
            for lit in range(index):
                pixels[lit] = ctx.color
            yield pixels

    def _rainbow(self, ctx: PatternContext) -> Iterable[List[Color]]:
        frames = max(1, int(self.frame_rate * 3.0))
        for frame in range(frames):
            yield [
                hsv_color((pixel / ctx.led_count) + (frame / frames))
                for pixel in range(ctx.led_count)
            ]

    def _theater_chase(self, ctx: PatternContext) -> Iterable[List[Color]]:
        frames_per_phase = max(1, int(self.frame_rate * 0.12))
        for phase in range(3):
            pixels = [
                ctx.color if (index + phase) % 3 == 0 else ctx.secondary_color
                for index in range(ctx.led_count)
            ]
            for _ in range(frames_per_phase):
                yield pixels

    def _comet(self, ctx: PatternContext) -> Iterable[List[Color]]:
        tail = max(4, ctx.led_count // 6)
        for head in range(ctx.led_count + tail):
            pixels = [ctx.secondary_color] * ctx.led_count
            for offset in range(tail):
                index = head - offset
                if 0 <= index < ctx.led_count:
                    pixels[index] = blend(ctx.secondary_color, ctx.color, 1.0 - offset / tail)
            yield pixels

    def _sparkle(self, ctx: PatternContext) -> Iterable[List[Color]]:
        frames = max(1, int(self.frame_rate * 1.5))
        active = max(1, ctx.led_count // 10)
        for _ in range(frames):
            pixels = [ctx.secondary_color] * ctx.led_count
            for index in random.sample(range(ctx.led_count), k=min(active, ctx.led_count)):
                pixels[index] = blend(ctx.color, (255, 255, 255), random.random() * 0.4)
            yield pixels

    def _police(self, ctx: PatternContext) -> Iterable[List[Color]]:
        red = ctx.color if ctx.color != (255, 255, 255) else (255, 0, 0)
        blue = ctx.secondary_color if ctx.secondary_color != (0, 0, 0) else (0, 0, 255)
        frames = max(1, int(self.frame_rate * 0.12))
        half = ctx.led_count // 2
        for phase in range(4):
            left = red if phase % 2 == 0 else (0, 0, 0)
            right = blue if phase % 2 == 1 else (0, 0, 0)
            pixels = [left] * half + [right] * (ctx.led_count - half)
            for _ in range(frames):
                yield pixels

    def _color_cycle(self, ctx: PatternContext) -> Iterable[List[Color]]:
        palette = [
            ctx.color,
            ctx.secondary_color,
            (255, 120, 0),
            (0, 255, 80),
            (0, 160, 255),
            (180, 0, 255),
        ]
        frames = max(1, int(self.frame_rate * 0.5))
        for index, color in enumerate(palette):
            for step in range(frames):
                amount = step / frames
                previous = palette[index - 1]
                yield [blend(previous, color, amount)] * ctx.led_count


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LedActionServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
