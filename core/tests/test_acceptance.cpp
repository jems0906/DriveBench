#include "drivebench/evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
#include <stdexcept>

namespace {

using namespace drivebench;

void check(bool valid, const char* message) {
    if (!valid) throw std::runtime_error(message);
}

void near(double actual, double expected, const char* message) {
    if (!(std::abs(actual - expected) < 1e-10)) {
        std::cerr << "Expected " << expected << ", got " << actual << '\n';
        throw std::runtime_error(message);
    }
}

void fixture() {
    std::vector<Frame> frames;
    double position = 0;
    for (int tick = 0; tick <= 50; ++tick) {
        Frame frame;
        frame.tick = tick;
        double speed = 5;
        if (tick < 10) speed = 2;
        else if (tick <= 13) speed = (13 - tick) * 0.5;
        else if (tick < 20) speed = 0;
        else if (tick < 24) speed = tick - 19;
        if (tick > 0) position += speed * 0.1;
        frame.ego = {{{position, 0}, 0, speed}, 2, 2};
        frame.actors.push_back({"target", AgentClass::vehicle, {{{16, 0}, 0, 0}, 2, 2}});
        frame.goal = tick == 50;
        frame.on_road = !((tick >= 31 && tick <= 34) || tick == 41);
        frame.route = {position, ((tick >= 24 && tick <= 25) || (tick >= 45 && tick <= 47)) ? 2.0 : 0.0};
        frames.push_back(frame);
    }
    const RoadRules rules{4, 1, {2.4, 8}};
    const auto opening = measure({frames.front()}, rules, 17);
    check(!opening.collision, "fixture starts collision free");
    near(opening.min_ttc, 7, "initial TTC = (16 - 2) / 2");
    near(opening.min_distance[0], 14, "initial clearance = 16 - 2");
    const auto metrics = measure(frames, rules, 17);
    check(metrics.collision && metrics.goal_reached, "fixture collision and goal flags");
    near(metrics.min_ttc, 0, "overlap has zero TTC");
    near(metrics.min_distance[0], 0, "overlap has zero clearance");
    check(std::isinf(metrics.min_distance[1]) && std::isinf(metrics.min_distance[2]), "absent classes");
    for (const auto kind : {AgentClass::pedestrian, AgentClass::cyclist}) {
        auto opening_frame = frames.front();
        opening_frame.actors.front().kind = kind;
        near(measure({opening_frame}, rules, 17).min_distance[static_cast<std::size_t>(kind)], 14, "class bucketing");
    }
    check(metrics.off_road_events == 2 && metrics.lane_departures == 2, "count transitions, not frames");
    near(metrics.distance_traveled, 16.6, "distance = 1.8 + 0.3 + 1.5 + 13");
    near(metrics.route_length, 17, "planned route length");
    check(metrics.stop_sign_violations == 1, "one compliant and one missed stop");
    check(metrics.speed_limit_violations == 27, "speed violations at ticks 24 through 50");
    near(metrics.max_acceleration, 10, "1 m/s step per 0.1 s");
    near(metrics.max_jerk, 100, "10 m/s^2 step per 0.1 s");
    check(metrics.harsh_brakes == 1, "one sustained harsh brake event");
    const auto verdict = score(metrics, {true, true, true, true, true, true, 0});
    check(verdict.failures == std::vector<std::string>{"collision", "off_road", "stop_sign", "speed_limit", "lane_keeping", "harsh_braking"},
          "every configured violation reports its category");
}

bool failed_with(const RunResult& result, std::string_view category) {
    const auto& failures = result.verdict.failures;
    return std::find(failures.begin(), failures.end(), category) != failures.end();
}

void scenarios(std::string_view test, const std::filesystem::path& directory) {
    const auto drive = [&](std::string_view name, std::string_view version, std::uint64_t seed = 42) {
        return run_scenario(directory / (std::string(name) + ".json"), version, seed);
    };
    if (test == "determinism") {
        const auto first = drive("pedestrian_crossing", "driver_v1");
        const auto second = drive("pedestrian_crossing", "driver_v1");
        check(first.trajectory.size() > 1, "nontrivial replay");
        check(trajectory_bytes(first.trajectory) == trajectory_bytes(second.trajectory), "bit-identical replay");
    } else if (test == "seed_sweep") {
        std::set<std::string> replays;
        std::set<double> clearances;
        for (std::uint64_t seed = 0; seed < 100; ++seed) {
            const auto result = drive("pedestrian_crossing", "driver_v1", seed);
            replays.insert(trajectory_bytes(result.trajectory));
            clearances.insert(result.metrics.min_distance[1]);
        }
        check(replays.size() == 100 && clearances.size() > 1, "seed changes physical outcomes, not just serialized noise");
    } else if (test == "verdicts") {
        check(drive("clear_road", "driver_v1").verdict.passed, "clear road passes");
        const auto collision = drive("pedestrian_crossing", "never_brake");
        check(!collision.verdict.passed && collision.metrics.collision && failed_with(collision, "collision"), "pedestrian collision fails");

        AgentSpec walker;
        walker.id = "steered_pedestrian";
        walker.kind = AgentClass::pedestrian;
        walker.spawn = {{{4, 0}, 1.5707963267948966, 0.8}, 0.5, 0.5};
        walker.waypoints = {{4, 2.5}};
        walker.cruise_speed = 0.8;
        WorldConfig world;
        world.ego = {{{0, 0}, 0, 6}, 4, 1.8};
        world.lanes = {{{{-5, -3}, {12, -3}, {12, 3}, {-5, 3}}}};
        world.traffic = {walker};
        World steering_world(world);
        bool steering_collision = false;
        for (int tick = 0; tick < 30 && !steering_collision; ++tick) {
            steering_world.step({0, 0.55});
            steering_collision = overlaps(steering_world.ego(), steering_world.traffic().front().box);
        }
        check(steering_collision, "positive steering reaches a pedestrian");
        Frame collided_frame;
        collided_frame.ego = steering_world.ego();
        collided_frame.actors.push_back({"steered_pedestrian", AgentClass::pedestrian, steering_world.traffic().front().box});
        const auto steering_metrics = measure({collided_frame}, {8, 1, {}}, 12);
        const auto steering_verdict = score(steering_metrics, {true, false, false, false, false, false, std::nullopt});
        check(steering_metrics.collision && steering_verdict.failures.size() == 1 &&
              steering_verdict.failures.front() == "collision", "steered pedestrian collision category");
    } else if (test == "harsh_suite") {
        for (const auto name : {"unprotected_left", "pedestrian_crossing", "driveway_pullout", "cutin_merge", "sudden_brake", "wet_lead"}) {
            check(drive(name, "driver_v1").verdict.passed, name);
            check(failed_with(drive(name, "never_brake"), "collision"), name);
        }
    } else if (test == "regression") {
        const auto baseline = summarize({drive("pedestrian_crossing", "driver_v1").verdict,
                                         drive("delayed_pedestrian", "driver_v1").verdict});
        const auto revision = summarize({drive("pedestrian_crossing", "driver_v2").verdict,
                                         drive("delayed_pedestrian", "driver_v2").verdict});
        check(!baseline.failures.contains("collision") && revision.failures.at("collision") == 2, "pedestrian collision delta +2");
        check(failed_with(drive("cyclist_lane_entry", "driver_v1"), "collision") &&
              drive("cyclist_lane_entry", "driver_v2").verdict.passed, "cyclist behavior improves");
    } else if (test == "validation") {
        const auto invalid = directory.parent_path() / "core" / "tests" / "fixtures" / "missing_route.json";
        try {
            load_scenario(invalid);
        } catch (const std::invalid_argument& error) {
            const std::string_view message = error.what();
            check(message.find("missing_route.json") != message.npos && message.find("key 'route' not found") != message.npos,
                  "clear JSON validation error");
            return;
        }
        throw std::runtime_error("missing route accepted");
    } else if (test == "feedback") {
        const auto scenario = load_scenario(directory / "yielding_cross_traffic.json");
        World advancing(scenario.world);
        World braking(scenario.world);
        for (int tick = 0; tick < 30; ++tick) {
            advancing.step({0, 0});
            braking.step({-8, 0});
        }
        check(braking.traffic().front().box.state.position.y > advancing.traffic().front().box.state.position.y + 1,
              "changing only ego actions changes yielding traffic motion");
    } else {
        throw std::runtime_error("unknown test");
    }
}

}

int main(int argc, char** argv) {
    try {
        check(argc == 3, "expected test name and scenario directory");
        const std::string_view test = argv[1];
        if (test == "metrics") fixture();
        else scenarios(test, argv[2]);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}