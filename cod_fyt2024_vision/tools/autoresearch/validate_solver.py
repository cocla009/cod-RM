#!/usr/bin/env python3
"""Deterministic offline evaluator for the armor-solver trajectory parameters.

The evaluator intentionally uses only Python's standard library so it can run in
the repository's dedicated venv without adding a dependency. It mirrors the
current solver's prediction delay, plate selection, center-tracking transition,
and fire-tolerance rules over several synthetic RoboMaster scenarios.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import statistics
import time
from dataclasses import dataclass
from pathlib import Path


TWO_PI = 2.0 * math.pi
SMALL_HALF_WIDTH = 0.133 / 2.0
LARGE_HALF_WIDTH = 0.225 / 2.0


def normalize(angle: float) -> float:
    return math.remainder(angle, TWO_PI)


def clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))


def read_numeric_parameters(path: Path) -> dict[str, float]:
    values: dict[str, float] = {}
    pattern = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*):\s*([-+0-9.eE]+)")
    for line in path.read_text(encoding="utf-8").splitlines():
        match = pattern.match(line)
        if match:
            values[match.group(1)] = float(match.group(2))
    return values


@dataclass(frozen=True)
class Scenario:
    name: str
    distance: float
    yaw_rate: float
    target_age: float
    yaw_noise: float
    occlusion_period: int
    occlusion_length: int


SCENARIOS = (
    Scenario("static_target", 3.0, 0.0, 0.0, 0.002, 0, 0),
    Scenario("low_speed_spin", 3.0, 1.2, 0.0, 0.004, 0, 0),
    Scenario("high_speed_spin", 1.2, 5.5, 0.0, 0.008, 0, 0),
    Scenario("occluded_fast_target", 3.5, 3.8, 0.12, 0.010, 80, 12),
)


def armor_position(
    distance: float, center_z: float, yaw: float, index: int, count: int
) -> tuple[float, float, float]:
    long_armor = count == 4 and index % 2 == 1
    radius = 0.24 if long_armor else 0.20
    height = 0.08 if long_armor else 0.0
    angle = yaw + index * TWO_PI / count
    return (
        distance - radius * math.cos(angle),
        -radius * math.sin(angle),
        center_z + height,
    )


def choose_armor(
    yaw: float,
    yaw_rate: float,
    distance: float,
    params: dict[str, float],
    lock_id: int,
    count: int = 4,
) -> tuple[int, float]:
    deltas = [normalize(yaw + i * TWO_PI / count) for i in range(count)]
    min_switch = params["min_switching_v_yaw"]
    if abs(yaw_rate) < min_switch:
        selected = min(range(count), key=lambda i: abs(deltas[i]))
        if 0 <= lock_id < count:
            lock_delta = abs(deltas[lock_id])
            best_delta = abs(deltas[selected])
            if lock_delta < math.pi / 3.0 and lock_delta - best_delta < math.pi / 6.0:
                selected = lock_id
    else:
        coming = math.radians(params["coming_angle"])
        leaving = math.radians(params["leaving_angle"])
        selected = None
        best_score = float("inf")
        for i, delta in enumerate(deltas):
            if yaw_rate > 0.0:
                in_zone = -coming < delta < leaving
            else:
                in_zone = -leaving < delta < coming
            if in_zone and abs(delta) < best_score:
                best_score = abs(delta)
                selected = i
        if selected is None:
            selected = min(range(count), key=lambda i: abs(deltas[i]))
    return selected, deltas[selected]


def tolerance(
    distance: float, armor_count: int, delta_angle: float, params: dict[str, float]
) -> float:
    half_width = LARGE_HALF_WIDTH if armor_count == 2 else SMALL_HALF_WIDTH
    projected = half_width * abs(math.cos(delta_angle))
    margin = max(0.1, params["fire_margin"])
    lower = math.radians(params["min_fire_tolerance"])
    upper = math.radians(max(params["min_fire_tolerance"], params["max_fire_tolerance"]))
    return clamp(math.atan2(projected, distance) * margin, lower, upper)


def run_scenario(
    scenario: Scenario, params: dict[str, float], frames: int, dt: float
) -> dict[str, float]:
    actual_yaw = 0.0
    actual_pitch = 0.0
    lock_id = -1
    errors: list[float] = []
    fires = 0
    true_fires = 0
    fire_opportunities = 0
    false_fires = 0
    step_times: list[float] = []
    center_tracking_frames = 0

    bullet_speed = max(10.0, min(25.0, params["bullet_speed"]))
    for frame in range(frames):
        start = time.perf_counter_ns()
        t = frame * dt
        true_yaw = scenario.yaw_rate * t
        occluded = (
            scenario.occlusion_period > 0
            and frame % scenario.occlusion_period < scenario.occlusion_length
        )
        age = scenario.target_age if occluded else 0.0
        estimated_yaw = scenario.yaw_rate * max(0.0, t - age)
        estimated_yaw += scenario.yaw_noise * math.sin(13.0 * t + 0.3)

        flight_time = math.sqrt(scenario.distance**2 + 0.25**2) / bullet_speed
        total_delay = max(0.0, params["prediction_delay"])
        total_delay += max(0.0, params["controller_delay"]) + flight_time
        predicted_yaw = estimated_yaw + total_delay * scenario.yaw_rate
        selected, selected_delta = choose_armor(
            predicted_yaw,
            scenario.yaw_rate,
            scenario.distance,
            params,
            lock_id,
        )
        lock_id = selected

        fast_and_near = (
            abs(scenario.yaw_rate) > params["max_tracking_v_yaw"]
            and scenario.distance < params["center_tracking_distance"]
        )
        center_tracking = fast_and_near
        if center_tracking:
            center_tracking_frames += 1

        px, py, pz = armor_position(scenario.distance, 0.20, predicted_yaw, selected, 4)
        desired_yaw = math.atan2(py, px)
        desired_pitch = math.atan2(pz, math.hypot(px, py))
        if center_tracking:
            desired_yaw = 0.0
            desired_pitch = math.atan2(0.20, scenario.distance)

        max_yaw_step = 8.0 * dt
        max_pitch_step = 5.0 * dt
        actual_yaw += clamp(normalize(desired_yaw - actual_yaw), -max_yaw_step, max_yaw_step)
        actual_pitch += clamp(desired_pitch - actual_pitch, -max_pitch_step, max_pitch_step)

        true_positions = [armor_position(scenario.distance, 0.20, true_yaw, i, 4) for i in range(4)]
        true_index = min(
            range(4), key=lambda i: abs(normalize(true_yaw + i * TWO_PI / 4.0))
        )
        tx, ty, tz = true_positions[true_index]
        true_yaw_command = math.atan2(ty, tx)
        true_pitch_command = math.atan2(tz, math.hypot(tx, ty))
        error = math.hypot(
            normalize(actual_yaw - true_yaw_command), actual_pitch - true_pitch_command
        )
        errors.append(error)

        predicted_tolerance = tolerance(scenario.distance, 4, selected_delta, params)
        true_tolerance = tolerance(scenario.distance, 4, 0.0, params)
        fire = (not center_tracking) and error < predicted_tolerance
        if not center_tracking:
            fire_opportunities += 1
        if fire:
            fires += 1
            if error < true_tolerance and not occluded:
                true_fires += 1
            else:
                false_fires += 1

        step_times.append((time.perf_counter_ns() - start) / 1e6)

    warmup = min(50, len(errors) // 5)
    stable_errors = errors[warmup:]
    stable_steps = step_times[warmup:]
    stable_errors.sort()
    stable_steps.sort()
    p95_error = stable_errors[int(0.95 * (len(stable_errors) - 1))]
    p95_step = stable_steps[int(0.95 * (len(stable_steps) - 1))]
    mean_error = statistics.fmean(stable_errors)
    false_rate = false_fires / max(1, fires)
    recall = true_fires / fire_opportunities if fire_opportunities else 1.0
    score = math.degrees(p95_error) + 2.0 * math.degrees(mean_error)
    score += 20.0 * false_rate + 5.0 * (1.0 - recall)
    return {
        "p95_error_deg": math.degrees(p95_error),
        "mean_error_deg": math.degrees(mean_error),
        "false_fire_rate": false_rate,
        "fire_recall": recall,
        "p95_step_ms": p95_step,
        "center_tracking_ratio": center_tracking_frames / frames,
        "score": score,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=600)
    parser.add_argument("--dt", type=float, default=0.01)
    args = parser.parse_args()

    params = read_numeric_parameters(args.config)
    required = {
        "bullet_speed",
        "prediction_delay",
        "controller_delay",
        "max_tracking_v_yaw",
        "center_tracking_distance",
        "min_switching_v_yaw",
        "coming_angle",
        "leaving_angle",
        "fire_margin",
        "min_fire_tolerance",
        "max_fire_tolerance",
    }
    missing = sorted(required - params.keys())
    if missing:
        raise SystemExit(f"missing parameters in {args.config}: {', '.join(missing)}")

    results = {
        scenario.name: run_scenario(scenario, params, args.frames, args.dt)
        for scenario in SCENARIOS
    }
    aggregate_score = statistics.fmean(item["score"] for item in results.values())
    aggregate_p95_error = max(item["p95_error_deg"] for item in results.values())
    aggregate_false_rate = max(item["false_fire_rate"] for item in results.values())
    aggregate_step = max(item["p95_step_ms"] for item in results.values())
    output = {
        "score": aggregate_score,
        "worst_p95_error_deg": aggregate_p95_error,
        "worst_false_fire_rate": aggregate_false_rate,
        "worst_p95_step_ms": aggregate_step,
        "scenarios": results,
    }
    print(json.dumps(output, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
