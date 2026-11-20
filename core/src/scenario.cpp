#include "drivebench/scenario.hpp"

#include <cmath>
#include <fstream>
#include <set>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace drivebench {
namespace {

using Json = nlohmann::json;

void require(bool valid, const std::string& message) {
    if (!valid) throw std::invalid_argument(message);
}

double number(const Json& value, double minimum, double maximum, const std::string& field) {
    require(value.is_number(), field + " must be numeric");
    const double result = value.get<double>();
    require(std::isfinite(result) && result >= minimum && result <= maximum, field + " out of range");
    return result;
}

int count(const Json& value, int minimum, int maximum, const std::string& field) {
    require(value.is_number_integer(), field + " must be an integer");
    return static_cast<int>(number(value, minimum, maximum, field));
}

Vec2 point(const Json& value) {
    require(value.is_array() && value.size() == 2, "point must contain x and y");
    return {number(value[0], -10000, 10000, "x"), number(value[1], -10000, 10000, "y")};
}

std::vector<Vec2> points(const Json& value, std::size_t minimum) {
    require(value.is_array() && value.size() >= minimum, "not enough points in array");
    std::vector<Vec2> result;
    for (const auto& entry : value) {
        const auto next = point(entry);
        require(result.empty() || distance(result.back(), next) > 1e-8, "consecutive points must differ");
        result.push_back(next);
    }
    return result;
}

Box box(const Json& value) {
    return {{point(value.at("position")), number(value.at("heading"), -3.142, 3.142, "heading"),
             number(value.at("speed"), 0, 60, "speed")},
            number(value.at("length"), 0.1, 25, "length"), number(value.at("width"), 0.1, 10, "width")};
}

AgentClass agent_class(const std::string& name) {
    if (name == "vehicle") return AgentClass::vehicle;
    if (name == "pedestrian") return AgentClass::pedestrian;
    if (name == "cyclist") return AgentClass::cyclist;
    throw std::invalid_argument("unknown agent class: " + name);
}

}

double route_length(const std::vector<Vec2>& route) {
    double result = 0;
    for (std::size_t index = 1; index < route.size(); ++index) result += distance(route[index - 1], route[index]);
    return result;
}

Scenario load_scenario(const std::filesystem::path& path) {
    try {
        std::ifstream input(path);
        require(input.is_open(), "cannot open scenario");
        const auto root = Json::parse(input);
        Scenario scenario;
        scenario.name = root.at("name").get<std::string>();
        require(!scenario.name.empty(), "name must not be empty");
        scenario.max_ticks = count(root.at("max_ticks"), 1, 6000, "max_ticks");
        scenario.goal_radius = number(root.at("goal_radius"), 0.1, 10, "goal_radius");
        scenario.perception_noise = number(root.at("perception_noise"), 0, 1, "perception_noise");
        if (root.contains("friction") && root.contains("weather")) {
            throw std::invalid_argument(path.string() + ": use weather.friction or friction, not both");
        }
        if (root.contains("weather")) {
            require(root.at("weather").is_object(), "weather must be an object");
            require(root.at("weather").contains("friction"), "weather.friction is required");
            scenario.world.friction = number(root.at("weather").at("friction"), 0.1, 1, "weather.friction");
        } else {
            scenario.world.friction = number(root.at("friction"), 0.1, 1, "friction");
        }
        scenario.world.ego = box(root.at("ego"));
        scenario.route = points(root.at("route"), 2);
        const auto& road = root.at("road");
        const auto& lanes = road.at("lanes");
        require(lanes.is_array() && !lanes.empty(), "road.lanes must be a nonempty array");
        for (const auto& ring : lanes) {
            auto lane = points(ring, 3);
            if (distance(lane.front(), lane.back()) <= 1e-8) lane.pop_back();
            require(lane.size() >= 3, "lane needs at least three vertices");
            double area = 0;
            for (std::size_t index = 0; index < lane.size(); ++index) {
                const auto first = lane[index];
                const auto second = lane[(index + 1) % lane.size()];
                area += first.x * second.y - second.x * first.y;
            }
            require(std::abs(area) > 1e-8, "lane polygon must have nonzero area");
            scenario.world.lanes.push_back(std::move(lane));
        }
        scenario.rules.speed_limit = number(road.at("speed_limit"), 0.1, 60, "speed_limit");
        scenario.rules.lane_tolerance = number(road.at("lane_tolerance"), 0.1, 20, "lane_tolerance");
        const auto& signs = road.at("stop_signs");
        require(signs.is_array(), "road.stop_signs must be an array");
        const double length = route_length(scenario.route);
        for (const auto& entry : signs) {
            const double sign = number(entry, 1, length, "stop_signs");
            require(scenario.rules.stop_signs.empty() || scenario.rules.stop_signs.back() < sign,
                    "stop signs must be strictly increasing");
            scenario.rules.stop_signs.push_back(sign);
        }
        const auto& agents = root.at("agents");
        require(agents.is_array(), "agents must be an array");
        std::set<std::string> ids;
        for (const auto& entry : agents) {
            AgentSpec agent;
            agent.id = entry.at("id").get<std::string>();
            require(!agent.id.empty() && ids.insert(agent.id).second, "agent ids must be nonempty and unique");
            agent.kind = agent_class(entry.at("class").get<std::string>());
            agent.spawn = box(entry);
            agent.waypoints = points(entry.at("waypoints"), 0);
            agent.cruise_speed = number(entry.at("cruise_speed"), 0, 60, "cruise_speed");
            agent.start_tick = count(entry.at("start_tick"), 0, scenario.max_ticks, "start_tick");
            agent.yields_to_ego = entry.value("yields_to_ego", false);
            if (entry.contains("brake_tick")) agent.brake_tick = count(entry.at("brake_tick"), 0, scenario.max_ticks, "brake_tick");
            if (entry.contains("resume_tick")) {
                require(agent.brake_tick.has_value(), "resume_tick requires brake_tick");
                agent.resume_tick = count(entry.at("resume_tick"), *agent.brake_tick + 1, scenario.max_ticks, "resume_tick");
            }
            scenario.world.traffic.push_back(std::move(agent));
        }
        const auto& criteria = root.at("criteria");
        const auto optional_rule = [&](const char* field) { return criteria.value(field, false); };
        scenario.criteria = {criteria.at("no_collision").get<bool>(), criteria.at("goal_reached").get<bool>(),
                             criteria.at("no_off_road").get<bool>(), optional_rule("obey_stop_signs"),
                             optional_rule("obey_speed_limit"), optional_rule("keep_lane"), std::nullopt};
        if (criteria.contains("max_harsh_brakes")) {
            scenario.criteria.max_harsh_brakes = count(criteria.at("max_harsh_brakes"), 0, scenario.max_ticks, "max_harsh_brakes");
        }
        for (const auto& [key, value] : criteria.items()) {
            static const std::set<std::string> allowed{"no_collision", "goal_reached", "no_off_road",
                "obey_stop_signs", "obey_speed_limit", "keep_lane", "max_harsh_brakes"};
            require(allowed.contains(key), "unknown criterion: " + key);
        }
        return scenario;
    } catch (const std::exception& error) {
        throw std::invalid_argument(path.string() + ": " + error.what());
    }
}

}