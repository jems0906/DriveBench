"""Generate bounded, reproducible traffic perturbations for a base scenario."""

import argparse
import json
import random
from copy import deepcopy
from pathlib import Path


def variations(base: dict, count: int, seed: int):
    if not base["agents"]:
        raise ValueError("base scenario must contain traffic to vary")
    if count < 1:
        raise ValueError("count must be positive")
    randomizer = random.Random(seed)
    for index in range(count):
        variant = deepcopy(base)
        variant["name"] = f"{base['name']}_var_{index:03d}"
        for actor in variant["agents"]:
            factor = randomizer.uniform(0.8, 1.2)
            for key in ("speed", "cruise_speed"):
                actor[key] = min(60, actor[key] * factor)
            points = [actor["position"], *actor["waypoints"]]
            for axis in range(2):
                values = [point[axis] for point in points]
                offset = randomizer.uniform(max(-0.75, -10000 - min(values)), min(0.75, 10000 - max(values)))
                for point in points:
                    point[axis] += offset
            keys = [key for key in ("start_tick", "brake_tick", "resume_tick") if key in actor]
            earliest = min(actor[key] for key in keys)
            latest = max(actor[key] for key in keys)
            shift = randomizer.randint(max(-5, -earliest), min(5, base["max_ticks"] - latest))
            for key in keys:
                actor[key] += shift
        yield variant


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("base", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    try:
        base = json.loads(args.base.read_text(encoding="utf-8"))
        args.output.mkdir(parents=True, exist_ok=True)
        for index, variant in enumerate(variations(base, args.count, args.seed)):
            path = args.output / f"{args.base.stem}.var-{index:03d}.json"
            path.write_text(json.dumps(variant, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    except (OSError, KeyError, TypeError, ValueError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()