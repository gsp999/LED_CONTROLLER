#!/usr/bin/python3
import json
import time
from collections import deque
from pathlib import Path
from typing import Deque, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from led_controller.action import RecognizeLedState

Color = Tuple[int, int, int]
Box = Tuple[int, int, int, int]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def load_state_map(path: str) -> Dict[str, str]:
    if not path:
        return {}
    map_path = Path(path).expanduser()
    if not map_path.exists():
        return {}
    with map_path.open('r', encoding='utf-8') as handle:
        data = json.load(handle)
    return {str(key): str(value) for key, value in data.items()}


def visible_video_devices() -> List[str]:
    return sorted(str(path) for path in Path('/dev').glob('video*'))


def open_camera(camera_index: int):
    if camera_index >= 0:
        camera = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        if camera.isOpened():
            return camera, camera_index
        camera.release()
        camera = cv2.VideoCapture(camera_index)
        if camera.isOpened():
            return camera, camera_index
        camera.release()
        return None, camera_index

    for index in range(10):
        camera = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if camera.isOpened():
            return camera, index
        camera.release()
    return None, camera_index


def camera_open_error(camera_index: int) -> str:
    devices = visible_video_devices()
    device_text = ', '.join(devices) if devices else 'none visible'
    return (
        f'failed to open camera {camera_index}; visible devices: {device_text}; '
        'try another camera_index, or use camera_index: -1 to auto scan; '
        'if /dev/video* exists but cannot open, add current user to the video group and re-login'
    )


def create_mask(frame: np.ndarray, min_value: int, min_saturation: int) -> np.ndarray:
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]
    dynamic_value = max(min_value, int(np.percentile(value, 95) * 0.55))
    color_mask = (value >= dynamic_value) & (saturation >= min_saturation)
    white_mask = (value >= max(dynamic_value, 150)) & (saturation < 95)
    mask = np.where(color_mask | white_mask, 255, 0).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    # Join the six physical LEDs in one WS2811 group into a single candidate box.
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 21), np.uint8))
    mask = cv2.dilate(mask, np.ones((5, 9), np.uint8), iterations=1)
    return mask


def find_candidate_boxes(mask: np.ndarray, expected_groups: int) -> List[Box]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    frame_area = mask.shape[0] * mask.shape[1]
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < max(20.0, frame_area * 0.00003):
            continue
        if area > frame_area * 0.25:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if w < 4 or h < 4:
            continue
        fill = area / max(float(w * h), 1.0)
        score = area * (0.4 + fill)
        candidates.append((score, (x, y, w, h)))

    candidates.sort(key=lambda item: item[0], reverse=True)
    boxes = [box for _, box in candidates[: max(expected_groups * 3, expected_groups)]]
    boxes = merge_nearby_boxes(boxes)
    boxes.sort(key=lambda box: box[2] * box[3], reverse=True)
    selected = boxes[:expected_groups]
    selected.sort(key=lambda box: box[0])
    return selected


def merge_nearby_boxes(boxes: Sequence[Box]) -> List[Box]:
    merged: List[Box] = []
    for box in boxes:
        x, y, w, h = box
        cx = x + w / 2.0
        cy = y + h / 2.0
        placed = False
        for index, existing in enumerate(merged):
            ex, ey, ew, eh = existing
            ecx = ex + ew / 2.0
            ecy = ey + eh / 2.0
            near_x = abs(cx - ecx) < max(w, ew) * 1.2
            near_y = abs(cy - ecy) < max(h, eh) * 1.5
            if near_x and near_y:
                nx = min(x, ex)
                ny = min(y, ey)
                nx2 = max(x + w, ex + ew)
                ny2 = max(y + h, ey + eh)
                merged[index] = (nx, ny, nx2 - nx, ny2 - ny)
                placed = True
                break
        if not placed:
            merged.append(box)
    return merged


def smooth_boxes(previous: Sequence[Box], current: Sequence[Box], expected_groups: int) -> List[Box]:
    if not previous:
        return list(current)
    if not current:
        return list(previous)

    result: List[Box] = []
    used = set()
    for prev in previous:
        px, py, pw, ph = prev
        pcx = px + pw / 2.0
        pcy = py + ph / 2.0
        best_index = None
        best_distance = float('inf')
        for index, box in enumerate(current):
            if index in used:
                continue
            x, y, w, h = box
            distance = abs((x + w / 2.0) - pcx) + abs((y + h / 2.0) - pcy)
            if distance < best_distance:
                best_distance = distance
                best_index = index
        if best_index is None:
            result.append(prev)
            continue
        used.add(best_index)
        x, y, w, h = current[best_index]
        alpha = 0.35
        result.append((
            int(px * (1 - alpha) + x * alpha),
            int(py * (1 - alpha) + y * alpha),
            int(pw * (1 - alpha) + w * alpha),
            int(ph * (1 - alpha) + h * alpha),
        ))
    for index, box in enumerate(current):
        if index not in used and len(result) < expected_groups:
            result.append(box)
    result = result[:expected_groups]
    result.sort(key=lambda box: box[0])
    return result


def crop_box(frame: np.ndarray, box: Box, padding: int = 4) -> np.ndarray:
    x, y, w, h = box
    x1 = max(0, x - padding)
    y1 = max(0, y - padding)
    x2 = min(frame.shape[1], x + w + padding)
    y2 = min(frame.shape[0], y + h + padding)
    return frame[y1:y2, x1:x2]


def masked_mean_bgr(crop: np.ndarray, min_value: int, min_saturation: int) -> Tuple[Color, float]:
    if crop.size == 0:
        return (0, 0, 0), 0.0
    mask = create_mask(crop, min_value, min_saturation)
    active = cv2.countNonZero(mask)
    if active < 4:
        return (0, 0, 0), 0.0
    mean = cv2.mean(crop, mask=mask)[:3]
    bgr = int(mean[0]), int(mean[1]), int(mean[2])
    hsv = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0][0]
    return bgr, float(hsv[2])


def classify_color(mean_bgr: Color, active: bool) -> str:
    if not active:
        return 'off'
    hsv = cv2.cvtColor(np.uint8([[mean_bgr]]), cv2.COLOR_BGR2HSV)[0][0]
    hue, saturation, value = int(hsv[0]), int(hsv[1]), int(hsv[2])
    if value < 35:
        return 'off'
    if saturation < 35 and value > 115:
        return 'white'
    if hue < 8 or hue >= 172:
        return 'red'
    if hue < 22:
        return 'orange'
    if hue < 38:
        return 'yellow'
    if hue < 82:
        return 'green'
    if hue < 100:
        return 'cyan'
    if hue < 132:
        return 'blue'
    if hue < 172:
        return 'magenta'
    return 'unknown'


def estimate_frequency(samples: Sequence[Tuple[float, float]]) -> float:
    if len(samples) < 8:
        return 0.0
    times = np.array([item[0] for item in samples], dtype=np.float32)
    values = np.array([item[1] for item in samples], dtype=np.float32)
    if float(times[-1] - times[0]) < 1.0:
        return 0.0
    low = float(np.percentile(values, 15))
    high = float(np.percentile(values, 90))
    if high - low < 18.0:
        return 0.0
    threshold = low + (high - low) * 0.50
    hysteresis = max(5.0, (high - low) * 0.12)
    armed = values[0] < threshold
    rising_edges = []
    last_edge = -10.0
    for timestamp, value in zip(times, values):
        if armed and value >= threshold + hysteresis and timestamp - last_edge > 0.16:
            rising_edges.append(float(timestamp))
            last_edge = float(timestamp)
            armed = False
        elif value <= threshold - hysteresis:
            armed = True
    if len(rising_edges) < 2:
        return 0.0
    intervals = np.diff(np.array(rising_edges))
    intervals = intervals[(intervals > 0.16) & (intervals < 5.0)]
    if len(intervals) == 0:
        return 0.0
    return float(1.0 / np.median(intervals))


def quantize_frequency(hz: float) -> str:
    if hz < 0.25:
        return '0'
    if hz < 0.75:
        return '0.5'
    if hz < 1.5:
        return '1'
    if hz < 2.5:
        return '2'
    if hz < 3.8:
        return '3'
    return f'{hz:.1f}'


class GroupTracker:
    def __init__(self, window_seconds: float) -> None:
        self.window_seconds = window_seconds
        self.samples: Deque[Tuple[float, float]] = deque()
        self.color_votes: Deque[Tuple[float, str]] = deque()

    def update(self, timestamp: float, brightness: float, color: str) -> None:
        self.samples.append((timestamp, brightness))
        self.color_votes.append((timestamp, color))
        while self.samples and timestamp - self.samples[0][0] > self.window_seconds:
            self.samples.popleft()
        while self.color_votes and timestamp - self.color_votes[0][0] > self.window_seconds:
            self.color_votes.popleft()

    def frequency(self) -> float:
        return estimate_frequency(list(self.samples))

    def stable_color(self) -> str:
        votes: Dict[str, int] = {}
        for _, color in self.color_votes:
            if color != 'off':
                votes[color] = votes.get(color, 0) + 1
        if not votes:
            return 'off'
        return max(votes, key=votes.get)


def flatten_boxes(boxes: Sequence[Box]) -> List[int]:
    data: List[int] = []
    for x, y, w, h in boxes:
        data.extend([int(x), int(y), int(w), int(h)])
    return data


def build_signature(colors: Sequence[str], frequencies: Sequence[float]) -> str:
    return '|'.join(
        f'g{index}:{color}@{quantize_frequency(frequency)}'
        for index, (color, frequency) in enumerate(zip(colors, frequencies))
    )


class LedCameraRecognitionActionServer(Node):
    def __init__(self) -> None:
        super().__init__('led_camera_recognition_action_server')
        self.declare_parameter('action_name', 'recognize_led_state')
        self.declare_parameter('default_camera_index', 0)
        self.declare_parameter('default_duration', 5.0)
        self.declare_parameter('expected_groups', 3)
        self.declare_parameter('min_value', 55.0)
        self.declare_parameter('min_saturation', 45.0)
        self._action_name = str(self.get_parameter('action_name').value)
        self._action_server = ActionServer(
            self,
            RecognizeLedState,
            self._action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info(f'LED camera recognition action ready: /{self._action_name}')

    def destroy_node(self) -> bool:
        self._action_server.destroy()
        return super().destroy_node()

    def goal_callback(self, goal_request) -> GoalResponse:
        del goal_request
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        del goal_handle
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        request = goal_handle.request
        camera_index = int(request.camera_index)
        if camera_index < 0:
            camera_index = -1
        duration = float(request.duration)
        if duration <= 0.0:
            duration = float(self.get_parameter('default_duration').value)
        expected_groups = int(request.expected_groups)
        if expected_groups <= 0:
            expected_groups = int(self.get_parameter('expected_groups').value)
        min_value = int(request.min_value if request.min_value > 0 else self.get_parameter('min_value').value)
        min_saturation = int(
            request.min_saturation if request.min_saturation > 0 else self.get_parameter('min_saturation').value
        )
        state_map = load_state_map(request.state_map_path)
        result = RecognizeLedState.Result()

        camera, opened_index = open_camera(camera_index)
        if camera is None:
            goal_handle.abort()
            result.success = False
            result.message = camera_open_error(camera_index)
            return result
        camera_index = opened_index
        self.get_logger().info(f'opened camera {camera_index}')

        trackers = [GroupTracker(duration) for _ in range(expected_groups)]
        boxes: List[Box] = []
        started = time.monotonic()
        last_feedback = 0.0
        colors = ['off'] * expected_groups
        frequencies = [0.0] * expected_groups
        signature = ''

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'recognition canceled'
                    return result

                ok, frame = camera.read()
                if not ok:
                    goal_handle.abort()
                    result.success = False
                    result.message = 'failed to read camera frame'
                    return result

                timestamp = time.monotonic()
                elapsed = timestamp - started
                mask = create_mask(frame, min_value, min_saturation)
                detected = find_candidate_boxes(mask, expected_groups)
                boxes = smooth_boxes(boxes, detected, expected_groups)

                for index in range(expected_groups):
                    if index >= len(boxes):
                        trackers[index].update(timestamp, 0.0, 'off')
                        continue
                    mean_bgr, brightness = masked_mean_bgr(crop_box(frame, boxes[index]), min_value, min_saturation)
                    color = classify_color(mean_bgr, brightness > 10.0)
                    trackers[index].update(timestamp, brightness, color)

                colors = [tracker.stable_color() for tracker in trackers]
                frequencies = [tracker.frequency() for tracker in trackers]
                signature = build_signature(colors, frequencies)

                if request.show_debug:
                    debug = frame.copy()
                    self._draw_debug(debug, boxes, colors, frequencies, signature)
                    cv2.imshow('LED auto recognition', debug)
                    cv2.waitKey(1)

                if timestamp - last_feedback >= 0.25:
                    feedback = RecognizeLedState.Feedback()
                    feedback.percent_complete = float(clamp(elapsed / duration * 100.0, 0.0, 100.0))
                    feedback.message = f'detected {len(boxes)}/{expected_groups} groups'
                    feedback.signature = signature
                    feedback.group_colors = colors
                    feedback.group_frequencies = [float(value) for value in frequencies]
                    feedback.group_boxes = flatten_boxes(boxes)
                    goal_handle.publish_feedback(feedback)
                    last_feedback = timestamp

                if elapsed >= duration:
                    break

            goal_handle.succeed()
            result.success = len(boxes) >= expected_groups
            result.message = f'detected {len(boxes)}/{expected_groups} groups'
            result.signature = signature
            result.state_name = state_map.get(signature, signature)
            result.group_colors = colors
            result.group_frequencies = [float(value) for value in frequencies]
            result.group_boxes = flatten_boxes(boxes)
            return result
        finally:
            camera.release()
            if request.show_debug:
                cv2.destroyWindow('LED auto recognition')

    @staticmethod
    def _draw_debug(
        frame: np.ndarray,
        boxes: Sequence[Box],
        colors: Sequence[str],
        frequencies: Sequence[float],
        signature: str,
    ) -> None:
        cv2.putText(frame, signature, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        for index, box in enumerate(boxes):
            x, y, w, h = box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
            color = colors[index] if index < len(colors) else 'unknown'
            hz = frequencies[index] if index < len(frequencies) else 0.0
            cv2.putText(
                frame,
                f'G{index} {color} {hz:.2f}Hz',
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LedCameraRecognitionActionServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
