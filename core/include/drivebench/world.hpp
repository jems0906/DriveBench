#pragma once

#include <optional>
#include <string>
#include <vector>

namespace drivebench {

inline constexpr double timestep = 0.1;

struct Vec2 {
    double x{};
    double y{};
};

struct State {
    Vec2 position;
    double heading{};
    double speed{};
};

struct Action {
    double acceleration{};
    double steering{};
};

enum class AgentClass { vehicle, pedestrian, cyclist };

struct Box {
    State state;
    double length{};
    double width{};
};

using Polygon = std::vector<Vec2>;

struct AgentSpec {
    std::string id;
    AgentClass kind{};
    Box spawn;
    std::vector<Vec2> waypoints;
    double cruise_speed{};
    int start_tick{};
    bool yields_to_ego{};
    std::optional<int> brake_tick;
    std::optional<int> resume_tick;
};

struct WorldConfig {
    Box ego;
    std::vector<Polygon> lanes;
    std::vector<AgentSpec> traffic;
    double friction{1};
};

struct TrafficState {
    Box box;
    bool active{};
};

State bicycle_step(State state, Action action, double wheelbase, double friction);
double distance(Vec2 first, Vec2 second);
double target_steering(const State& state, Vec2 target, double wheelbase);
bool overlaps(const Box& first, const Box& second);
double clearance(const Box& first, const Box& second);
double time_to_collision(const Box& first, const Box& second);
bool on_road(const Box& box, const std::vector<Polygon>& lanes);

class World {
public:
    explicit World(WorldConfig config);
    void step(Action action);
    const Box& ego() const { return config_.ego; }
    const std::vector<TrafficState>& traffic() const { return traffic_; }
    int tick() const { return tick_; }

private:
    WorldConfig config_;
    std::vector<TrafficState> traffic_;
    std::vector<std::size_t> waypoint_indices_;
    int tick_{};
};

}