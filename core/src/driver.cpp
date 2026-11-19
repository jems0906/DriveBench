#include "drivebench/driver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace drivebench {
namespace {

Vec2 point_at(const std::vector<Vec2>& route, double progress) {
    for (std::size_t index = 1; index < route.size(); ++index) {
        const auto start = route[index - 1];
        const auto end = route[index];
        const double length = distance(start, end);
        if (progress <= length || index + 1 == route.size()) {
            return {start.x + (end.x - start.x) * progress / length, start.y + (end.y - start.y) * progress / length};
        }
        progress -= length;
    }
    return route.back();
}

}

RoutePosition locate_on_route(const std::vector<Vec2>& route, Vec2 point) {
    RoutePosition result{0, std::numeric_limits<double>::infinity()};
    double prefix = 0;
    for (std::size_t index = 1; index < route.size(); ++index) {
        const auto start = route[index - 1];
        const auto end = route[index];
        const double length = distance(start, end);
        const double progress = std::clamp(((point.x - start.x) * (end.x - start.x) +
                                            (point.y - start.y) * (end.y - start.y)) / length, 0.0, length);
        const Vec2 nearest{start.x + (end.x - start.x) * progress / length,
                           start.y + (end.y - start.y) * progress / length};
        const double deviation = distance(point, nearest);
        if (deviation < result.deviation) result = {prefix + progress, deviation};
        prefix += length;
    }
    return result;
}

Driver::Driver(std::string_view version, std::vector<Vec2> route, RoadRules rules)
    : route_(std::move(route)), rules_(std::move(rules)) {
    if (version == "driver_v1") version_ = Version::baseline;
    else if (version == "driver_v2") version_ = Version::revision;
    else if (version == "never_brake") version_ = Version::no_brake;
    else throw std::invalid_argument("unknown driver version: " + std::string(version));
}

Action Driver::plan(const Observation& observation) {
    const auto& ego = observation.ego.state;
    const auto projection = locate_on_route(route_, ego.position);
    const double lookahead = std::clamp(ego.speed * 0.8, 3.0, 7.0);
    const auto target = point_at(route_, projection.progress + lookahead);
    const double steering = target_steering(ego, target, 2.7);
    double target_speed = rules_.speed_limit / (1 + 3 * std::abs(steering));
    if (next_sign_ < rules_.stop_signs.size()) {
        const double remaining = rules_.stop_signs[next_sign_] - projection.progress;
        if (remaining <= 1.5 && remaining >= 0 && ego.speed <= 0.1) ++stopped_ticks_;
        else stopped_ticks_ = 0;
        if (stopped_ticks_ >= 5 || remaining < 0) {
            ++next_sign_;
            stopped_ticks_ = 0;
        } else {
            target_speed = std::min(target_speed, std::sqrt(3 * std::max(0.0, remaining - 0.85)));
        }
    }
    double acceleration = std::clamp(3 * (target_speed - ego.speed), -8.0, 3.0);
    if (version_ == Version::no_brake) return {std::max(0.0, acceleration), steering};

    Box probe = observation.ego;
    probe.state.speed = std::max(2.0, ego.speed);
    for (const auto& actor : observation.actors) {
        double horizon = 2.0;
        if (actor.kind == AgentClass::pedestrian) horizon = version_ == Version::revision ? 0.35 : 2.2;
        if (actor.kind == AgentClass::cyclist) horizon = version_ == Version::revision ? 2.3 : 0.45;
        Box buffered = actor.box;
        buffered.length += 0.4;
        buffered.width += 0.5;
        if (time_to_collision(probe, buffered) < horizon) acceleration = -8;
    }
    return {acceleration, steering};
}

}