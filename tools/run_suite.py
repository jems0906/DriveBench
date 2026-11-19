"""Evaluate every scenario under a driver version and one or more seeds."""

import argparse
import hashlib
import json
from pathlib import Path

import pydrivebench


DRIVERS = ("driver_v1", "driver_v2", "never_brake")


def scenario_paths(path: Path):
    files = sorted(path.glob("*.json")) if path.is_dir() else [path]
    if not files:
        raise ValueError(f"no scenario JSON files in {path}")
    return files


def seed_range(seed: int, count: int):
    if count < 1 or seed < 0 or seed + count > 2**64:
        raise ValueError("seed count must be positive and all seeds must fit in uint64")
    return range(seed, seed + count)


def describe(result):
    return {
        "scenario": result.scenario,
        "driver_version": result.driver_version,
        "seed": result.seed,
        "passed": result.verdict.passed,
        "failures": result.verdict.failures,
        "metrics": result.metrics,
        "trajectory_sha256": hashlib.sha256(result.trajectory_bytes()).hexdigest(),
    }


def write_json(path: Path, value, *, compact: bool = False):
    text = json.dumps(value, indent=None if compact else 2, allow_nan=False)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as output:
        output.write(text + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenarios", type=Path)
    parser.add_argument("--version", choices=DRIVERS, default="driver_v1")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--seed-count", type=int, default=1)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--trajectories", action="store_true", help="include each frame in the JSON report")
    args = parser.parse_args()
    try:
        rows, verdicts = [], []
        seeds = seed_range(args.seed, args.seed_count)
        for path in scenario_paths(args.scenarios):
            for seed in seeds:
                result = pydrivebench.run_scenario(path, args.version, seed)
                row = {"input": path.name, **describe(result)}
                if args.trajectories:
                    row["trajectory"] = result.trajectory
                rows.append(row)
                verdicts.append(result.verdict)
        report = {"summary": pydrivebench.summarize(verdicts), "runs": rows}
        if args.output:
            write_json(args.output, report)
        else:
            print(json.dumps(report, indent=2, allow_nan=False))
    except (OSError, ValueError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()