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
from std_srvs.srv import Trigger

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
        order = self._resolve_pixel_order(order_name)
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

    @staticmethod
    def _resolve_pixel_order(pixel_order: str):
        channel_index = {'R': 0, 'G': 1, 'B': 2, 'W': 3}
        if set(pixel_order).issubset(channel_index) and len(pixel_order) in (3, 4):
            return tuple(channel_index[channel] for channel in pixel_order)
        return tuple(channel_index[channel] for channel in 'BRG')


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
        self.declare_parameter('pixel_order', 'BRG')
        self.declare_parameter('default_brightness', 0.4)
        self.declare_parameter('action_name', 'led_pattern')
        self.declare_parameter('cancel_service_name', 'cancel_led_pattern')
        self.declare_parameter('frame_rate', 40.0)

        self.led_count = int(self.get_parameter('led_count').value)
        self.frame_rate = float(self.get_parameter('frame_rate').value)
        self.default_brightness = float(self.get_parameter('default_brightness').value)
        backend_name = str(self.get_parameter('backend').value)
        pixel_order = str(self.get_parameter('pixel_order').value)
        action_name = str(self.get_parameter('action_name').value)
        cancel_service_name = str(self.get_parameter('cancel_service_name').value)

        self._backend = self._make_backend(backend_name, pixel_order)
        self._stop_requested = False
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
            'lightning': self._lightning,
            'scanner_flash': self._scanner_flash,
            'state_1_green_single_pulse': self._state_1_green_single_pulse,
            'state_2_blue_slow_pulse': self._state_2_blue_slow_pulse,
            'state_3_yellow_slow_blink': self._state_3_yellow_slow_blink,
            'state_4_red_fast_blink': self._state_4_red_fast_blink,
            'state_5_magenta_double_pulse': self._state_5_magenta_double_pulse,
            'state_6_orange_five_pulse': self._state_6_orange_five_pulse,
            'state_7_white_heartbeat': self._state_7_white_heartbeat,
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
        self._cancel_service = self.create_service(
            Trigger,
            cancel_service_name,
            self.cancel_pattern_callback,
        )
        self.get_logger().info(
            f'LED action server ready: action=/{action_name}, cancel_service=/{cancel_service_name}, '
            f'backend={self._backend.name}, leds={self.led_count}'
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

    def cancel_pattern_callback(self, request, response):
        del request
        self._stop_requested = True
        self._backend.clear()
        response.success = True
        response.message = 'cancel requested for current LED pattern'
        return response

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
        )

        result = LedPattern.Result()
        self._active_goal = True
        self._stop_requested = False
        self.get_logger().info(
            f'goal accepted pattern={pattern} duration={ctx.duration:.2f}s'
        )

        started = self.get_clock().now()
        step = 0
        try:
            while rclpy.ok():
                for pixels in self._patterns[pattern](ctx):
                    if goal_handle.is_cancel_requested or self._stop_requested:
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

                if ctx.duration <= 0.0:
                    continue

                elapsed = (self.get_clock().now() - started).nanoseconds / 1_000_000_000.0
                if elapsed < ctx.duration:
                    continue
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

    def _all_pixels(self, ctx: PatternContext, color: Color) -> List[Color]:
        return [color] * ctx.led_count

    def _hold_color(
        self,
        ctx: PatternContext,
        color: Color,
        seconds: float,
    ) -> Iterable[List[Color]]:
        frames = max(1, int(self.frame_rate * seconds))
        for _ in range(frames):
            yield self._all_pixels(ctx, color)

    def _pulse_sequence(
        self,
        ctx: PatternContext,
        color: Color,
        sequence: Sequence[Tuple[bool, float]],
    ) -> Iterable[List[Color]]:
        for is_on, seconds in sequence:
            actual_color = color if is_on else (0, 0, 0)
            yield from self._hold_color(ctx, actual_color, seconds)

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

    def _state_1_green_single_pulse(self, ctx: PatternContext) -> Iterable[List[Color]]:
        yield from self._pulse_sequence(
            ctx,
            (0, 255, 0),
            [
                (True, 0.18),
                (False, 0.82),
            ],
        )

    def _state_2_blue_slow_pulse(self, ctx: PatternContext) -> Iterable[List[Color]]:
        color = (0, 80, 255)
        base = (0, 0, 12)
        frames = max(12, int(self.frame_rate * 1.6))
        for index in range(frames):
            wave = (1.0 - math.cos(index / frames * math.tau)) / 2.0
            yield self._all_pixels(ctx, blend(base, color, wave))

    def _state_3_yellow_slow_blink(self, ctx: PatternContext) -> Iterable[List[Color]]:
        yield from self._pulse_sequence(
            ctx,
            (255, 180, 0),
            [
                (True, 0.24),
                (False, 0.24),
                (True, 0.24),
                (False, 0.78),
            ],
        )

    def _state_4_red_fast_blink(self, ctx: PatternContext) -> Iterable[List[Color]]:
        yield from self._pulse_sequence(
            ctx,
            (220, 0, 0),
            [
                (True, 0.16),
                (False, 0.14),
                (True, 0.16),
                (False, 0.14),
                (True, 0.16),
                (False, 0.14),
                (True, 0.16),
                (False, 0.70),
            ],
        )

    def _state_5_magenta_double_pulse(self, ctx: PatternContext) -> Iterable[List[Color]]:
        yield from self._pulse_sequence(
            ctx,
            (255, 0, 180),
            [
                (True, 0.12),
                (False, 0.12),
                (True, 0.32),
                (False, 0.84),
            ],
        )

    def _state_6_orange_five_pulse(self, ctx: PatternContext) -> Iterable[List[Color]]:
        yield from self._pulse_sequence(
            ctx,
            (255, 90, 0),
            [
                (True, 0.08),
                (False, 0.08),
                (True, 0.08),
                (False, 0.08),
                (True, 0.08),
                (False, 0.08),
                (True, 0.08),
                (False, 0.08),
                (True, 0.08),
                (False, 0.70),
            ],
        )

    def _state_7_white_heartbeat(self, ctx: PatternContext) -> Iterable[List[Color]]:
        yield from self._pulse_sequence(
            ctx,
            (255, 255, 255),
            [
                (True, 0.08),
                (False, 0.10),
                (True, 0.08),
                (False, 0.10),
                (True, 0.36),
                (False, 0.78),
            ],
        )

    def _lightning(self, ctx: PatternContext) -> Iterable[List[Color]]:
        flashes = random.randint(3, 7)
        for flash in range(flashes):
            intensity = random.uniform(0.45, 1.0)
            flash_color = blend(ctx.color, (255, 255, 255), intensity * 0.55)
            active_count = random.randint(max(1, ctx.led_count // 5), max(1, ctx.led_count // 2))
            active_indexes = set(random.sample(range(ctx.led_count), k=min(active_count, ctx.led_count)))
            pixels = [
                blend(ctx.secondary_color, flash_color, random.uniform(0.65, 1.0))
                if index in active_indexes
                else ctx.secondary_color
                for index in range(ctx.led_count)
            ]
            for _ in range(random.randint(1, 3)):
                yield pixels

            dark_frames = random.randint(1, max(2, int(self.frame_rate * 0.08)))
            dim_tail = blend(ctx.secondary_color, ctx.color, 0.08 + flash * 0.03)
            for _ in range(dark_frames):
                yield [dim_tail] * ctx.led_count

        rest_frames = random.randint(
            max(2, int(self.frame_rate * 0.12)),
            max(3, int(self.frame_rate * 0.45)),
        )
        for _ in range(rest_frames):
            yield [ctx.secondary_color] * ctx.led_count

    def _scanner_flash(self, ctx: PatternContext) -> Iterable[List[Color]]:
        segment_count = 6
        segment_size = max(1, math.ceil(ctx.led_count / segment_count))
        hold_frames = max(1, int(self.frame_rate * 0.05))
        fade_frames = max(2, int(self.frame_rate * 0.12))

        for segment in range(segment_count):
            start = segment * segment_size
            end = min(ctx.led_count, start + segment_size)
            pixels = [ctx.secondary_color] * ctx.led_count
            for index in range(start, end):
                pixels[index] = ctx.color
            for _ in range(hold_frames):
                yield pixels

        for step in range(fade_frames):
            amount = 1.0 - step / fade_frames
            flash_color = blend(ctx.secondary_color, ctx.color, amount)
            yield [flash_color] * ctx.led_count

        for phase in range(4):
            color = ctx.color if phase % 2 == 0 else ctx.secondary_color
            for _ in range(max(1, int(self.frame_rate * 0.06))):
                yield [color] * ctx.led_count

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
