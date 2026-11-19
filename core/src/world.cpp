#include "drivebench/world.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace drivebench {

World::World(WorldConfig config) : config_(std::move(config)), waypoint_indices_(config_.traffic.size()) {
    for (const auto& spec : config_.traffic) traffic_.push_back({spec.spawn, spec.start_tick == 0});
}

void World::step(Action action) {
    const Box previous_ego = config_.ego;
    config_.ego.state = bicycle_step(previous_ego.state, action, 2.7, config_.friction);
    for (std::size_t index = 0; index < traffic_.size(); ++index) {
        auto& traffic = traffic_[index];
        const auto& spec = config_.traffic[index];
        if (!traffic.active) {
            traffic.active = tick_ + 1 >= spec.start_tick;
            continue;
        }
        auto& state = traffic.box.state;
        auto& waypoint = waypoint_indices_[index];
        const double reach = spec.kind == AgentClass::vehicle ? 1.5 : 1e-8;
        while (waypoint < spec.waypoints.size() && distance(state.position, spec.waypoints[waypoint]) <= reach) {
            ++waypoint;
        }
        if (waypoint == spec.waypoints.size()) {
            state.speed = 0;
            continue;
        }
        const auto target = spec.waypoints[waypoint];
        double speed = spec.cruise_speed;
        if (spec.brake_tick && tick_ >= *spec.brake_tick && (!spec.resume_tick || tick_ < *spec.resume_tick)) speed = 0;
        if (spec.yields_to_ego) {
            Box intent = traffic.box;
            intent.state.heading = std::atan2(target.y - state.position.y, target.x - state.position.x);
            intent.state.speed = speed;
            const double ahead = (previous_ego.state.position.x - state.position.x) * std::cos(intent.state.heading) +
                                 (previous_ego.state.position.y - state.position.y) * std::sin(intent.state.heading);
            if (ahead > 0 && time_to_collision(intent, previous_ego) < 1.6) speed = 0;
        }
        if (spec.kind == AgentClass::vehicle) {
            if (waypoint + 1 == spec.waypoints.size()) {
                speed = std::min(speed, std::sqrt(4 * std::max(0.0, distance(state.position, target) - reach)));
            }
            const double acceleration = (speed - state.speed) * 4;
            state = bicycle_step(state, {acceleration, target_steering(state, target, 2.7)}, 2.7, config_.friction);
        } else {
            const double length = distance(state.position, target);
            const double fraction = std::min(1.0, speed * timestep / length);
            state.heading = std::atan2(target.y - state.position.y, target.x - state.position.x);
            state.position.x += (target.x - state.position.x) * fraction;
            state.position.y += (target.y - state.position.y) * fraction;
            state.speed = fraction * length / timestep;
        }
    }
    ++tick_;
}

}