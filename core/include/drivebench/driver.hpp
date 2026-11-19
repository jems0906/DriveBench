#pragma once

#include "drivebench/scenario.hpp"

#include <string_view>

namespace drivebench {

struct ActorBox {
    std::string id;
    AgentClass kind{};
    Box box;
};

struct Observation {
    Box ego;
    std::vector<ActorBox> actors;
};

struct RoutePosition {
    double progress{};
    double deviation{};
};

RoutePosition locate_on_route(const std::vector<Vec2>& route, Vec2 point);

class Driver {
public:
    Driver(std::string_view version, std::vector<Vec2> route, RoadRules rules);
    Action plan(const Observation& observation);

private:
    enum class Version { baseline, revision, no_brake };
    Version version_;
    std::vector<Vec2> route_;
    RoadRules rules_;
    std::size_t next_sign_{};
    int stopped_ticks_{};
};

}