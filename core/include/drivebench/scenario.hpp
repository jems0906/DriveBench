#pragma once

#include "drivebench/world.hpp"

#include <filesystem>

namespace drivebench {

struct Criteria {
    bool no_collision{true};
    bool goal_reached{true};
    bool no_off_road{true};
    bool obey_stop_signs{};
    bool obey_speed_limit{};
    bool keep_lane{};
    std::optional<int> max_harsh_brakes;
};

struct RoadRules {
    double speed_limit{};
    double lane_tolerance{};
    std::vector<double> stop_signs;
};

struct Scenario {
    std::string name;
    WorldConfig world;
    std::vector<Vec2> route;
    RoadRules rules;
    Criteria criteria;
    int max_ticks{};
    double goal_radius{};
    double perception_noise{};
};

Scenario load_scenario(const std::filesystem::path& path);
double route_length(const std::vector<Vec2>& route);

}