# Metric Definitions

Scoring consumes ground-truth frames in time order, including both the spawn frame and the terminal frame. A run stops at its first sampled collision, goal, or tick limit. Terminal conditions and verdicts are separate: arriving on a frame with a collision still fails whenever no collision is required.

| Metric | Definition |
| --- | --- |
| Collision | Any sampled overlap, including contact, between ego and any active actor's oriented box. |
| Minimum TTC | Smallest nonnegative predicted time to OBB contact under constant velocity and fixed heading. Current overlap is zero; no predicted contact is `null` in Python/JSON. |
| Minimum distance per class | Smallest Euclidean edge-to-edge box clearance for each class. Overlap is zero; absent classes are `null` in Python/JSON. |
| Goal reached | Any ego center sample within the configured goal radius. |
| Off-road events | Entries into a state where an ego footprint sample falls outside the lane polygon union. A run starting off-road counts once. |
| Distance traveled / route length | Sum of center-to-center ego displacements / sum of specified route segment lengths, both in meters. |
| Stop-sign violations | First forward crossing of a sign without five consecutive samples at speed at most 0.1 m/s within 1.5 m upstream. A sign never crossed is not scored as a violation. |
| Speed-limit violations | Number of samples exceeding the speed limit by more than 0.05 m/s. |
| Lane departures | Entries into a state where route-centerline deviation exceeds `lane_tolerance`. This route-based proxy is separate from footprint containment. |
| Maximum acceleration and jerk | Largest absolute longitudinal speed derivative and acceleration derivative from adjacent samples, in m/s² and m/s³. No artificial derivative is added before the first difference. |
| Harsh brakes | Contiguous segments at realized longitudinal acceleration at or below -3 m/s², counted once each. |

Suite pass rate is passed runs divided by attempted runs. Each run contributes to every enabled, violated category, so counts can sum to more than the number of failed runs. Invalid scenarios are input errors, not failed runs.

## Five-second fixture

The [acceptance test](../core/tests/test_acceptance.cpp) supplies 51 samples and one stationary 2 m box centered at x = 16 m. Ego starts with the same box length, x = 0 m, and speed 2 m/s, giving 14 m clearance and 7 s TTC at tick zero. The last sample is at x = 16.6 m (1.8 + 0.3 + 1.5 + 13.0), so the boxes overlap and the minima are zero. Planned length is 17 m. Each of the other class buckets is also checked with the same known-clearance box as a separate one-actor fixture.

Recorded road and lane states create two events each. Ego stops for the first sign but not the second. There are 27 speeding samples (ticks 24-50), one sustained harsh brake, peak absolute acceleration 10 m/s², and peak jerk 100 m/s³. The fixture checks the goal flag and the exact configured failure categories as well.

## Adding a metric

1. Define units, thresholds and behavior for missing data here. Add a field to `Metrics` in [run_result.hpp](../core/include/drivebench/run_result.hpp).
2. Compute it from recorded frames in [evaluator.cpp](../core/src/evaluator.cpp); compare with a hand-calculated fixture in the acceptance test. Add a recorded field only if the existing frame cannot supply the evidence, and version `DBR1` if the byte layout changes.
3. Expose the field in [pydrivebench.cpp](../bindings/pydrivebench.cpp). Keep unavailable numbers JSON-safe (`None` / `null`).
4. For a gated metric, add its criteria field and JSON validation, map its violation to a stable failure label, and test both sides of the threshold. Batch and comparison tools consume those labels automatically.