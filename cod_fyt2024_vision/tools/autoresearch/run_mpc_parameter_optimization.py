#!/usr/bin/env python3
"""Bounded, resumable phase-6 search for armor-solver YAML parameters.

The validator is deliberately treated as an immutable black box.  This runner
only creates candidate YAML copies, executes the frozen validator, and records
every decision before moving to the next candidate.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import random
import re
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone, timedelta
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BASELINE = ROOT / "rm_bringup/config/node_params/armor_solver_params.yaml"
BEST = ROOT / "experiments/best_armor_solver_params.yaml"
CANDIDATES = ROOT / "experiments/parameter_candidates"
LOG = ROOT / "experiments/autoresearch_log.jsonl"
STATUS = ROOT / "experiments/phase_status.json"
STATE_DIR = ROOT.parent / ".codex-run-armor-solver-mpc-params-v1"
STATE = STATE_DIR / "state.json"
VALIDATOR = ROOT / "tools/autoresearch/validate_solver.py"
EXPECTED_VALIDATOR_SHA256 = "b399b1710ca17bc7412f070ecd83e7fe3b4bcc50ec78740bb0fdc1c241468ca3"
PARAM_RANGES = {
    "prediction_delay": (0.0, 0.15),
    "controller_delay": (0.0, 0.10),
    "max_tracking_v_yaw": (3.0, 8.0),
    "center_tracking_distance": (0.8, 3.0),
    "min_switching_v_yaw": (0.3, 3.0),
    "coming_angle": (25.0, 80.0),
    "leaving_angle": (5.0, 45.0),
    "fire_margin": (0.3, 1.5),
    "min_fire_tolerance": (0.3, 2.0),
    "max_fire_tolerance": (2.0, 8.0),
}
PARAM_KEYS = tuple(PARAM_RANGES)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def now() -> str:
    return datetime.now(timezone(timedelta(hours=8))).isoformat(timespec="seconds")


def read_numeric(path: Path) -> dict[str, float]:
    pattern = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*):\s*([-+0-9.eE]+)")
    values: dict[str, float] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        match = pattern.match(line)
        if match:
            values[match.group(1)] = float(match.group(2))
    return values


def validate_proposal(values: dict[str, float]) -> None:
    for key, (lower, upper) in PARAM_RANGES.items():
        value = values[key]
        if not math.isfinite(value) or not lower <= value <= upper:
            raise ValueError(f"range:{key}")
    if values["leaving_angle"] >= values["coming_angle"]:
        raise ValueError("constraint:leaving_angle<coming_angle")
    if values["max_fire_tolerance"] < values["min_fire_tolerance"]:
        raise ValueError("constraint:max_fire_tolerance>=min_fire_tolerance")


def canonical(values: dict[str, float]) -> str:
    return json.dumps({key: float(values[key]) for key in PARAM_KEYS}, sort_keys=True)


def write_candidate(values: dict[str, float], path: Path) -> None:
    text = BASELINE.read_text(encoding="utf-8")
    for key in PARAM_KEYS:
        replacement = rf"\g<1>{values[key]:.12g}\g<3>"
        text, count = re.subn(
            rf"(?m)^(\s*{re.escape(key)}:\s*)([-+0-9.eE]+)(\s*(?:#.*)?)$",
            replacement,
            text,
            count=1,
        )
        if count != 1:
            raise RuntimeError(f"candidate_write_missing:{key}")
    path.write_text(text, encoding="utf-8")


def validator_result(path: Path, timeout_seconds: int) -> dict:
    command = [
        str(ROOT / "venv/bin/python"),
        str(VALIDATOR),
        "--config",
        str(path),
        "--frames",
        "600",
        "--dt",
        "0.01",
    ]
    completed = subprocess.run(
        command,
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=timeout_seconds,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(f"validator_exit:{completed.returncode}:{completed.stderr[-400:]}")
    lines = [line for line in completed.stdout.splitlines() if line.strip()]
    if not lines:
        raise RuntimeError("validator_empty_output")
    try:
        result = json.loads(lines[-1])
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"validator_invalid_json:{exc.msg}") from exc
    required = {"score", "worst_p95_error_deg", "worst_false_fire_rate", "scenarios"}
    if not required.issubset(result) or not all(
        math.isfinite(float(result[key])) for key in required - {"scenarios"}
    ):
        raise RuntimeError("validator_invalid_metrics")
    return result


def safe(result: dict, baseline_result: dict) -> bool:
    scenarios = result["scenarios"]
    baseline_scenarios = baseline_result["scenarios"]
    return (
        result["worst_false_fire_rate"] == 0.0
        and scenarios["high_speed_spin"]["center_tracking_ratio"]
        >= baseline_scenarios["high_speed_spin"]["center_tracking_ratio"]
        and scenarios["static_target"]["fire_recall"]
        >= baseline_scenarios["static_target"]["fire_recall"]
        and result.get("worst_p95_step_ms", 0.0) <= 20.0
    )


def load_rows() -> list[dict]:
    if not LOG.exists():
        return []
    return [json.loads(line) for line in LOG.read_text(encoding="utf-8").splitlines() if line.strip()]


def update_status(state: dict) -> None:
    payload = {
        "algorithm_phase_complete": True,
        "completed_tasks": ["ALG-1", "ALG-2"],
        "phase": state["phase"],
        "next_task_id": "PARAM-6" if not state.get("exit") else "FINAL",
        "next_trial": state["next_trial"],
        "valid_candidates": state["valid_candidates"],
        "no_improvement": state["no_improvement"],
        "best_score": state["best_score"],
        "best_candidate": state["best_candidate"],
        "baseline_yaml_sha256": state["baseline_yaml_sha256"],
        "validator_sha256": state["validator_sha256"],
        "exit": state.get("exit", False),
        "exit_reason": state.get("exit_reason"),
        "updated_at": now(),
    }
    STATUS.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    STATE.write_text(json.dumps(state, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def append_log(row: dict) -> None:
    with LOG.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")


def build_proposals(incumbent: dict[str, float], seed: int) -> list[dict[str, float]]:
    proposals: list[dict[str, float]] = [copy.deepcopy(incumbent)]
    coordinate_values = {
        "prediction_delay": (0.0, 0.03, 0.06, 0.09, 0.12, 0.15),
        "controller_delay": (0.0, 0.02, 0.04, 0.06, 0.08, 0.10),
        "max_tracking_v_yaw": (3.0, 4.0, 4.5, 5.0, 6.0, 7.0, 8.0),
        "center_tracking_distance": (0.8, 1.0, 1.5, 2.0, 2.5, 3.0),
        "min_switching_v_yaw": (0.3, 0.8, 1.0, 1.5, 2.0, 2.5, 3.0),
        "coming_angle": (25.0, 40.0, 55.0, 65.0, 70.0, 80.0),
        "leaving_angle": (5.0, 15.0, 20.0, 30.0, 40.0, 45.0),
        "fire_margin": (0.3, 0.6, 0.8, 1.0, 1.2, 1.5),
        "min_fire_tolerance": (0.3, 0.6, 1.0, 1.2, 1.5, 2.0),
        "max_fire_tolerance": (2.0, 3.0, 4.0, 5.0, 6.0, 8.0),
    }
    for key, values in coordinate_values.items():
        for value in values:
            proposal = copy.deepcopy(incumbent)
            proposal[key] = value
            proposals.append(proposal)
    rng = random.Random(seed)
    for _ in range(180):
        proposal = copy.deepcopy(incumbent)
        for key, (lower, upper) in PARAM_RANGES.items():
            proposal[key] = round(rng.uniform(lower, upper), 8)
        proposals.append(proposal)
    return proposals


def dry_run(timeout_seconds: int) -> int:
    if sha256(VALIDATOR) != EXPECTED_VALIDATOR_SHA256:
        raise RuntimeError("validator_hash_changed")
    baseline_values = read_numeric(BASELINE)
    validate_proposal({key: baseline_values[key] for key in PARAM_KEYS})
    baseline_hash = sha256(BASELINE)
    result = validator_result(BASELINE, timeout_seconds)
    dry_path = CANDIDATES / "phase6-dry-run.yaml"
    write_candidate({key: baseline_values[key] for key in PARAM_KEYS}, dry_path)
    candidate_result = validator_result(dry_path, timeout_seconds)
    if result["score"] != candidate_result["score"] or sha256(BASELINE) != baseline_hash:
        raise RuntimeError("dry_run_baseline_mismatch")
    print(json.dumps({"dry_run": "pass", "score": result["score"], "candidate": str(dry_path)}))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--max-candidates", type=int, default=200)
    parser.add_argument("--timeout", type=int, default=120)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    CANDIDATES.mkdir(parents=True, exist_ok=True)
    if args.dry_run:
        return dry_run(args.timeout)

    validator_hash = sha256(VALIDATOR)
    baseline_hash = sha256(BASELINE)
    if validator_hash != EXPECTED_VALIDATOR_SHA256:
        raise RuntimeError("validator_hash_changed")
    baseline_values = read_numeric(BASELINE)
    baseline_result = validator_result(BASELINE, args.timeout)
    rows = load_rows()
    prior_keys = {canonical(row["proposal"]) for row in rows if row.get("phase") == "parameter" and row.get("proposal")}
    prior_trials = [int(row.get("trial", -1)) for row in rows if isinstance(row.get("trial"), int)]
    incumbent = read_numeric(BEST) if BEST.exists() else baseline_values
    incumbent = {key: incumbent[key] for key in PARAM_KEYS}
    validate_proposal(incumbent)
    best_result = validator_result(BEST if BEST.exists() else BASELINE, args.timeout)
    if STATE.exists():
        persisted = json.loads(STATE.read_text(encoding="utf-8"))
        if persisted.get("exit"):
            print(json.dumps({"status": "already_completed", "valid_candidates": persisted.get("valid_candidates", 0),
                              "best_score": persisted.get("best_score"),
                              "exit_reason": persisted.get("exit_reason")}, ensure_ascii=False))
            return 0
    state = {
        "phase": "phase_6_parameter_optimization",
        "next_trial": max(prior_trials, default=-1) + 1,
        "valid_candidates": 0,
        "no_improvement": 0,
        "best_score": best_result["score"],
        "best_candidate": str(BEST.relative_to(ROOT)) if BEST.exists() else str(BASELINE.relative_to(ROOT)),
        "baseline_yaml_sha256": baseline_hash,
        "validator_sha256": validator_hash,
        "consecutive_setup_failures": 0,
        "consecutive_validation_failures": 0,
        "same_errors": {},
        "exit": False,
        "exit_reason": None,
    }
    if STATE.exists():
        persisted = json.loads(STATE.read_text(encoding="utf-8"))
        for key in (
            "next_trial", "valid_candidates", "no_improvement", "best_score",
            "best_candidate", "consecutive_setup_failures", "consecutive_validation_failures",
            "same_errors",
        ):
            if key in persisted:
                state[key] = persisted[key]
    update_status(state)
    proposals = build_proposals(incumbent, 20260920)
    for proposal in proposals:
        if state["valid_candidates"] >= args.max_candidates or state["no_improvement"] >= 30:
            state["exit"] = True
            state["exit_reason"] = "stop_condition:max_candidates_or_30_no_improvement"
            break
        try:
            validate_proposal(proposal)
        except ValueError:
            continue
        key = canonical(proposal)
        if key in prior_keys:
            continue
        trial = state["next_trial"]
        candidate = CANDIDATES / f"phase6-trial-{trial}.yaml"
        try:
            write_candidate(proposal, candidate)
            result = validator_result(candidate, args.timeout)
            state["consecutive_validation_failures"] = 0
            state["consecutive_setup_failures"] = 0
        except (OSError, subprocess.TimeoutExpired, RuntimeError) as exc:
            signature = re.sub(r"[/\\][^ :]+", "<path>", str(exc))
            same = state["same_errors"].get(signature, 0) + 1
            state["same_errors"][signature] = same
            state["consecutive_validation_failures"] += 1
            row = {
                "trial": trial, "timestamp": now(), "phase": "parameter", "task_id": "PARAM-6",
                "proposal": proposal, "candidate_path": str(candidate.relative_to(ROOT)),
                "decision": "exit" if same >= 10 or state["consecutive_validation_failures"] >= 11 else "blocked",
                "reason": str(exc), "failure_type": "validation", "error_signature": signature,
                "same_error_count": same,
                "consecutive_setup_failures": state["consecutive_setup_failures"],
                "consecutive_validation_failures": state["consecutive_validation_failures"],
            }
            append_log(row)
            state["next_trial"] += 1
            if row["decision"] == "exit":
                state["exit"] = True
                state["exit_reason"] = f"exception_threshold:{signature}"
                update_status(state)
                break
            update_status(state)
            continue
        prior_keys.add(key)
        state["valid_candidates"] += 1
        safe_result = safe(result, baseline_result)
        accepted = safe_result and result["score"] < state["best_score"]
        if accepted:
            state["best_score"] = result["score"]
            shutil.copyfile(candidate, BEST)
            state["best_candidate"] = str(BEST.relative_to(ROOT))
            state["no_improvement"] = 0
            decision = "accepted"
        else:
            state["no_improvement"] += 1
            decision = "rejected"
        append_log({
            "trial": trial, "timestamp": now(), "phase": "parameter", "task_id": "PARAM-6",
            "next_trial": trial + 1, "candidate_path": str(candidate.relative_to(ROOT)),
            "proposal": proposal, "score": result["score"],
            "worst_p95_error_deg": result["worst_p95_error_deg"],
            "worst_false_fire_rate": result["worst_false_fire_rate"],
            "worst_p95_step_ms": result.get("worst_p95_step_ms"),
            "scenario_metrics": result["scenarios"], "decision": decision,
            "reason": "safe improvement" if accepted else ("safety gate failed" if not safe_result else "not better than incumbent"),
            "failure_type": None, "error_signature": None, "same_error_count": 0,
            "consecutive_setup_failures": 0, "consecutive_validation_failures": 0,
        })
        state["next_trial"] += 1
        update_status(state)
        if result["score"] <= 9.5:
            state["exit"] = True
            state["exit_reason"] = "target_reached:score<=9.5"
            break
    else:
        state["exit"] = True
        state["exit_reason"] = "stop_condition:proposal_pool_exhausted"
    update_status(state)
    print(json.dumps({"status": "completed", "valid_candidates": state["valid_candidates"],
                      "best_score": state["best_score"], "best_candidate": state["best_candidate"],
                      "exit_reason": state["exit_reason"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"phase6 runner failed: {exc}", file=sys.stderr)
        raise
