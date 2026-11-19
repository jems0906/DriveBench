#include "drivebench/world.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace drivebench {

double distance(Vec2 first, Vec2 second) {
    return std::hypot(first.x - second.x, first.y - second.y);
}

double target_steering(const State& state, Vec2 target, double wheelbase) {
    const double heading_error = std::atan2(target.y - state.position.y, target.x - state.position.x) - state.heading;
    return std::atan2(2.0 * wheelbase * std::sin(heading_error), std::max(0.5, distance(state.position, target)));
}

State bicycle_step(State state, Action action, double wheelbase, double friction) {
    const double acceleration = std::clamp(action.acceleration, -8.0 * friction, 3.0 * friction);
    const double speed = std::max(0.0, state.speed + acceleration * timestep);
    const double average_speed = (state.speed + speed) * 0.5;
    const double steering = std::clamp(action.steering, -0.55, 0.55);
    const double turn = average_speed * std::tan(steering) * timestep / wheelbase;
    const double mid_heading = state.heading + turn * 0.5;
    state.position.x += average_speed * std::cos(mid_heading) * timestep;
    state.position.y += average_speed * std::sin(mid_heading) * timestep;
    state.heading = std::remainder(state.heading + turn, 2.0 * std::numbers::pi);
    state.speed = speed;
    return state;
}

}