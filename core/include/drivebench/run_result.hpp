#pragma once

#include "drivebench/driver.hpp"

#include <array>
#include <cstdint>
#include <limits>

namespace drivebench {

struct Frame {
    int tick{};
    Box ego;
    std::vector<ActorBox> actors;
    std::vector<ActorBox> perceived;
    Action action;
    bool on_road{true};
    bool goal{};
    RoutePosition route;
};

struct Metrics {
    bool collision{};
    double min_ttc{std::numeric_limits<double>::infinity()};
    std::array<double, 3> min_distance{std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    bool goal_reached{};
    int off_road_events{};
    double distance_traveled{};
    double route_length{};
    int stop_sign_violations{};
    int speed_limit_violations{};
    int lane_departures{};
    double max_acceleration{};
    double max_jerk{};
    int harsh_brakes{};
};

struct Verdict {
    bool passed{};
    std::vector<std::string> failures;
};

struct RunResult {
    std::string scenario;
    std::string driver_version;
    std::uint64_t seed{};
    Metrics metrics;
    Verdict verdict;
    std::vector<Frame> trajectory;
};

RunResult run_scenario(const std::filesystem::path& path, std::string_view version, std::uint64_t seed);
RunResult run(const Scenario& scenario, std::string_view version, std::uint64_t seed);
std::string trajectory_bytes(const std::vector<Frame>& frames);

}