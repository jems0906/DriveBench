#include "drivebench/evaluator.hpp"

#include <algorithm>
#include <cmath>

namespace drivebench {

Metrics measure(const std::vector<Frame>& frames, const RoadRules& rules, double planned_length) {
    Metrics result;
    result.route_length = planned_length;
    bool off_road = false;
    bool departed = false;
    bool harsh = false;
    double previous_acceleration = 0;
    std::vector<int> stopped(rules.stop_signs.size());
    std::vector<bool> crossed(rules.stop_signs.size());
    for (std::size_t index = 0; index < frames.size(); ++index) {
        const auto& frame = frames[index];
        for (const auto& actor : frame.actors) {
            const auto ttc = time_to_collision(frame.ego, actor.box);
            result.collision |= ttc == 0;
            result.min_ttc = std::min(result.min_ttc, ttc);
            auto& closest = result.min_distance.at(static_cast<std::size_t>(actor.kind));
            closest = std::min(closest, clearance(frame.ego, actor.box));
        }
        result.goal_reached |= frame.goal;
        result.off_road_events += off_road == false && !frame.on_road;
        off_road = !frame.on_road;
        const bool outside_lane = frame.route.deviation > rules.lane_tolerance;
        result.lane_departures += !departed && outside_lane;
        departed = outside_lane;
        result.speed_limit_violations += frame.ego.state.speed > rules.speed_limit + 0.05;
        for (std::size_t sign = 0; sign < rules.stop_signs.size(); ++sign) {
            if (crossed[sign]) continue;
            const double remaining = rules.stop_signs[sign] - frame.route.progress;
            if (remaining < 0) {
                result.stop_sign_violations += stopped[sign] < 5;
                crossed[sign] = true;
            } else if (remaining <= 1.5 && frame.ego.state.speed <= 0.1) {
                ++stopped[sign];
            } else if (stopped[sign] < 5) {
                stopped[sign] = 0;
            }
        }
        if (index == 0) continue;
        const auto& previous = frames[index - 1];
        result.distance_traveled += distance(previous.ego.state.position, frame.ego.state.position);
        const double acceleration = (frame.ego.state.speed - previous.ego.state.speed) / timestep;
        result.max_acceleration = std::max(result.max_acceleration, std::abs(acceleration));
        if (index >= 2) result.max_jerk = std::max(result.max_jerk, std::abs(acceleration - previous_acceleration) / timestep);
        previous_acceleration = acceleration;
        const bool braking = acceleration <= -3.0;
        result.harsh_brakes += !harsh && braking;
        harsh = braking;
    }
    return result;
}

Verdict score(const Metrics& metrics, const Criteria& criteria) {
    Verdict result;
    if (criteria.no_collision && metrics.collision) result.failures.emplace_back("collision");
    if (criteria.goal_reached && !metrics.goal_reached) result.failures.emplace_back("goal_not_reached");
    if (criteria.no_off_road && metrics.off_road_events) result.failures.emplace_back("off_road");
    if (criteria.obey_stop_signs && metrics.stop_sign_violations) result.failures.emplace_back("stop_sign");
    if (criteria.obey_speed_limit && metrics.speed_limit_violations) result.failures.emplace_back("speed_limit");
    if (criteria.keep_lane && metrics.lane_departures) result.failures.emplace_back("lane_keeping");
    if (criteria.max_harsh_brakes && metrics.harsh_brakes > *criteria.max_harsh_brakes) result.failures.emplace_back("harsh_braking");
    result.passed = result.failures.empty();
    return result;
}

SuiteSummary summarize(const std::vector<Verdict>& verdicts) {
    SuiteSummary result;
    result.runs = verdicts.size();
    for (const auto& verdict : verdicts) {
        result.passed += verdict.passed;
        for (const auto& failure : verdict.failures) ++result.failures[failure];
    }
    if (result.runs) result.pass_rate = static_cast<double>(result.passed) / static_cast<double>(result.runs);
    return result;
}

}