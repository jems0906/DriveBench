#include "drivebench/evaluator.hpp"

#include <algorithm>
#include <random>

namespace drivebench {

RunResult run(const Scenario& scenario, std::string_view version, std::uint64_t seed) {
    World world(scenario.world);
    Driver driver(version, scenario.route, scenario.rules);
    std::mt19937_64 random(seed);
    const auto noise = [&] { return (2 * static_cast<double>(random() >> 11) * 0x1p-53 - 1) * scenario.perception_noise; };
    RunResult result;
    result.scenario = scenario.name;
    result.driver_version = version;
    result.seed = seed;
    for (int tick = 0; tick <= scenario.max_ticks; ++tick) {
        Frame frame;
        frame.tick = tick;
        frame.ego = world.ego();
        frame.goal = distance(frame.ego.state.position, scenario.route.back()) <= scenario.goal_radius;
        frame.on_road = on_road(frame.ego, scenario.world.lanes);
        frame.route = locate_on_route(scenario.route, frame.ego.state.position);
        bool collision = false;
        for (std::size_t index = 0; index < world.traffic().size(); ++index) {
            const auto& traffic = world.traffic()[index];
            if (!traffic.active) continue;
            const auto& spec = scenario.world.traffic[index];
            collision |= overlaps(frame.ego, traffic.box);
            frame.actors.push_back({spec.id, spec.kind, traffic.box});
            auto perceived = frame.actors.back();
            perceived.box.state.position.x += noise();
            perceived.box.state.position.y += noise();
            frame.perceived.push_back(std::move(perceived));
        }
        const bool terminal = collision || frame.goal || tick == scenario.max_ticks;
        if (!terminal) frame.action = driver.plan({frame.ego, frame.perceived});
        const auto action = frame.action;
        result.trajectory.push_back(std::move(frame));
        if (terminal) break;
        world.step(action);
    }
    result.metrics = measure(result.trajectory, scenario.rules, route_length(scenario.route));
    result.verdict = score(result.metrics, scenario.criteria);
    return result;
}

RunResult run_scenario(const std::filesystem::path& path, std::string_view version, std::uint64_t seed) {
    return run(load_scenario(path), version, seed);
}

}