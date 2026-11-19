"""Compare paired runs and retain trajectories for every category change."""

import argparse
from pathlib import Path

import pydrivebench

from run_suite import DRIVERS, describe, scenario_paths, seed_range, write_json


def compare(paths, seeds, baseline, candidate, output):
    pairs, before_verdicts, after_verdicts = [], [], []
    for path in paths:
        for seed in seeds:
            before = pydrivebench.run_scenario(path, baseline, seed)
            after = pydrivebench.run_scenario(path, candidate, seed)
            before_verdicts.append(before.verdict)
            after_verdicts.append(after.verdict)
            removed = sorted(set(before.verdict.failures) - set(after.verdict.failures))
            added = sorted(set(after.verdict.failures) - set(before.verdict.failures))
            pair = {"input": path.name, "seed": seed, "baseline": describe(before), "candidate": describe(after),
                    "resolved_failures": removed, "new_failures": added}
            if removed or added:
                replay = Path("trajectories") / f"{path.stem}.seed-{seed}.json"
                write_json(output / replay, {"baseline": before.trajectory, "candidate": after.trajectory}, compact=True)
                pair["replay"] = replay.as_posix()
            pairs.append(pair)
    before_summary = pydrivebench.summarize(before_verdicts)
    after_summary = pydrivebench.summarize(after_verdicts)
    old_failures, new_failures = before_summary["failures"], after_summary["failures"]
    categories = sorted(old_failures.keys() | new_failures.keys())
    return {
        "baseline": {"version": baseline, **before_summary},
        "candidate": {"version": candidate, **after_summary},
        "failure_delta": {key: new_failures.get(key, 0) - old_failures.get(key, 0) for key in categories},
        "pairs": pairs,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenarios", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--baseline", choices=DRIVERS, default="driver_v1")
    parser.add_argument("--candidate", choices=DRIVERS, default="driver_v2")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--seed-count", type=int, default=1)
    parser.add_argument("--fail-on-regression", action="store_true")
    args = parser.parse_args()
    try:
        report = compare(scenario_paths(args.scenarios), seed_range(args.seed, args.seed_count),
                         args.baseline, args.candidate, args.output)
        write_json(args.output / "comparison.json", report)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(f"{args.baseline}: {report['baseline']['passed']}/{report['baseline']['runs']} passed; "
          f"{args.candidate}: {report['candidate']['passed']}/{report['candidate']['runs']} passed")
    print(f"Failure delta: {report['failure_delta']}")
    if args.fail_on_regression and any(pair["new_failures"] for pair in report["pairs"]):
        raise SystemExit(1)


if __name__ == "__main__":
    main()