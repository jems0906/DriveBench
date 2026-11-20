# Scenario Guide

All positions are meters in a flat Cartesian plane, headings are radians from +x, and speeds are m/s. Times use integer ticks of 0.1 s.

1. Copy a scenario close to your case, give it a unique `name`, and add it to [scenarios](../scenarios).
2. Set `ego` with `position`, `heading`, `speed`, `length`, and `width`. Provide ordered `route` points; the last point is the goal with `goal_radius`. Set `max_ticks` (1-6000). Set friction with `weather.friction` (0.1-1); the legacy top-level `friction` field remains accepted, but the two forms cannot be combined. Set `perception_noise` (0-1 m per actor-position axis).
3. Give `road` a list of nondegenerate `lanes` polygons, a positive `speed_limit`, `lane_tolerance`, and sorted `stop_signs` measured in meters along the route. An empty stop list is valid. Use simple polygons and non-self-intersecting routes, and keep stop signs before the goal radius.
4. Give each `agents` entry a unique `id`, a `class` of `vehicle`, `pedestrian`, or `cyclist`, the same five box fields as ego, plus `waypoints`, `cruise_speed`, and `start_tick`. An empty waypoint list is for a stationary actor; set both speeds to zero. Optional `yields_to_ego` enables conflict reactions, and `brake_tick` plus optional later `resume_tick` scripts a drop in target speed.
5. Explicitly set `criteria.no_collision`, `criteria.goal_reached`, and `criteria.no_off_road`. Optional booleans are `obey_stop_signs`, `obey_speed_limit`, and `keep_lane`; `max_harsh_brakes` optionally limits event count. Enabled criteria are ANDed. Run `python tools/run_suite.py` on the new file and add a focused assertion to the acceptance test when the behavior is a regression target.

`vary_scenario.py` perturbs each actor's speeds by up to 20%, offsets its entire path by up to 0.75 m per axis, and shifts its spawn and brake schedule together by up to five ticks within bounds. It preserves ego, road, and verdict criteria. Use a separate output directory containing only variants you intend to run.

All shipped scenarios require no collision, goal reached, and no off-road event. Additional criteria are listed below. `*` marks the harsh acceptance subset (v1 passes all six; `never_brake` collides in all six at seed 42).

| Scenario | Challenge | Additional pass criteria |
| --- | --- | --- |
| `clear_road` | Straight clear route | Speed, lane, zero harsh brakes |
| `unprotected_left` * | Left turn across moving traffic | Lane |
| `pedestrian_crossing` * | Near-side crosswalk | Speed, lane |
| `cyclist_share` | Passing a cyclist with lateral clearance | Lane, zero harsh brakes |
| `cyclist_lane_entry` | Cyclist crosses into ego's path | Lane |
| `driveway_pullout` * | Delayed vehicle from a driveway | Speed, lane |
| `cutin_merge` * | A slower vehicle merges from the side | Lane |
| `sudden_brake` * | Lead vehicle brakes, then resumes | Speed |
| `wet_lead` * | Lead vehicle brakes on reduced friction | Lane |
| `yielding_cross_traffic` | Cross traffic responds to ego | Lane |
| `stop_sign` | Single full stop | Stop, speed |
| `two_stop_signs` | Two stops on lower friction | Stop, speed, lane |
| `parked_car` | Stationary car beside the route | Lane, zero harsh brakes |
| `right_turn` | Curved right turn without traffic | Speed, lane |
| `delayed_pedestrian` | A later, opposite-side crossing on reduced friction | Lane |
| `staggered_crossings` | Two actors cross at different times | Speed |
| `construction_merge` | Narrow lane with a work-zone vehicle | Lane |
| `school_crossing` | Crossing guard and pedestrians near a school | Stop, speed |
| `roundabout_entry` | Yield into circulating traffic | Collision, lane |
| `intersection_conflict` | Crossing vehicle enters on a red-light violation | Collision, speed |