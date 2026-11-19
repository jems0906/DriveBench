# Reference Evaluation

The seed-42 comparison covers all 16 scenarios. These artifacts were produced with MSVC 19.39 Release, Windows x64, and Python 3.13.3. To reproduce from the repository root with the Python binding on `PYTHONPATH`:

```sh
python tools/compare_versions.py scenarios build/comparison --seed 42
```

| Driver | Passed | Pass rate | Collision | Goal not reached |
| --- | ---: | ---: | ---: | ---: |
| `driver_v1` | 15/16 | 93.75% | 1 | 1 |
| `driver_v2` | 13/16 | 81.25% | 3 | 3 |
| Candidate minus baseline | -2 | -12.50 pp | +2 | +2 |

| Changed case | Baseline | Candidate | Paired ground-truth and perception replay |
| --- | --- | --- | --- |
| Cyclist lane entry | Collision at tick 21; 0 m clearance | Goal at tick 64; 0.262 m minimum clearance | [Cyclist replay](../reports/reference/trajectories/cyclist_lane_entry.seed-42.json) |
| Pedestrian crossing | Goal at tick 65; 0.458 m clearance | Collision at tick 20; 0 m minimum clearance | [Pedestrian replay](../reports/reference/trajectories/pedestrian_crossing.seed-42.json) |
| Delayed pedestrian | Goal at tick 86; 0.392 m clearance | Collision at tick 33; 0 m minimum clearance | [Delayed replay](../reports/reference/trajectories/delayed_pedestrian.seed-42.json) |
| Staggered crossings | Goal at tick 90; 0.343 m clearance | Collision at tick 21; 0 m minimum clearance | [Staggered replay](../reports/reference/trajectories/staggered_crossings.seed-42.json) |

The longer cyclist TTC window avoided a real simulated collision. The shorter pedestrian window delayed braking and turned three passes into failures. The net collision delta of +2 comprises three new collisions and one resolved collision; the aggregate alone hides the improvement. The release gate rejects v2 with `--fail-on-regression`.

The [machine-readable comparison](../reports/reference/comparison.json) includes individual metrics, categories, and binary trajectory digests for all 32 runs. The replay files contain start-to-terminal frames for both drivers, including commands and perceived boxes. Exact digests are guaranteed to repeat within the same build and environment, not across platforms.