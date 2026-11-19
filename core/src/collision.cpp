#include "drivebench/world.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace drivebench {
namespace {

Vec2 subtract(Vec2 first, Vec2 second) { return {first.x - second.x, first.y - second.y}; }
double dot(Vec2 first, Vec2 second) { return first.x * second.x + first.y * second.y; }

Vec2 forward(const Box& box) {
    return {std::cos(box.state.heading), std::sin(box.state.heading)};
}

Vec2 velocity(const Box& box) {
    const auto direction = forward(box);
    return {direction.x * box.state.speed, direction.y * box.state.speed};
}

std::array<Vec2, 2> axes(const Box& box) {
    const auto direction = forward(box);
    return {direction, Vec2{-direction.y, direction.x}};
}

std::array<Vec2, 4> corners(const Box& box) {
    const auto [longitudinal, lateral] = axes(box);
    const auto point = [&](double front, double side) {
        return Vec2{box.state.position.x + longitudinal.x * front + lateral.x * side,
                    box.state.position.y + longitudinal.y * front + lateral.y * side};
    };
    const double length = box.length / 2;
    const double width = box.width / 2;
    return {point(length, width), point(-length, width), point(-length, -width), point(length, -width)};
}

double radius(const Box& box, Vec2 axis) {
    const auto [longitudinal, lateral] = axes(box);
    return (std::abs(dot(axis, longitudinal)) * box.length + std::abs(dot(axis, lateral)) * box.width) / 2;
}

double point_segment(Vec2 point, Vec2 start, Vec2 end) {
    const auto edge = subtract(end, start);
    const auto offset = subtract(point, start);
    const double fraction = std::clamp(dot(offset, edge) / dot(edge, edge), 0.0, 1.0);
    return distance(point, {start.x + fraction * edge.x, start.y + fraction * edge.y});
}

bool contains(const Polygon& polygon, Vec2 point) {
    bool inside = false;
    for (std::size_t index = 0, previous = polygon.size() - 1; index < polygon.size(); previous = index++) {
        const auto start = polygon[previous];
        const auto end = polygon[index];
        if (point_segment(point, start, end) <= 1e-9) return true;
        if ((start.y > point.y) != (end.y > point.y) &&
            point.x < start.x + (end.x - start.x) * (point.y - start.y) / (end.y - start.y)) {
            inside = !inside;
        }
    }
    return inside;
}

}

double time_to_collision(const Box& first, const Box& second) {
    const auto [first_forward, first_side] = axes(first);
    const auto [second_forward, second_side] = axes(second);
    const auto offset = subtract(second.state.position, first.state.position);
    const auto relative_velocity = subtract(velocity(second), velocity(first));
    double entry = 0;
    double exit = std::numeric_limits<double>::infinity();
    for (const auto axis : {first_forward, first_side, second_forward, second_side}) {
        const double reach = radius(first, axis) + radius(second, axis);
        const double separation = dot(offset, axis);
        const double rate = dot(relative_velocity, axis);
        if (rate == 0) {
            if (std::abs(separation) > reach) return std::numeric_limits<double>::infinity();
        } else {
            const double lower = (-reach - separation) / rate;
            const double upper = (reach - separation) / rate;
            entry = std::max(entry, std::min(lower, upper));
            exit = std::min(exit, std::max(lower, upper));
            if (entry > exit) return std::numeric_limits<double>::infinity();
        }
    }
    return entry;
}

bool overlaps(const Box& first, const Box& second) {
    return time_to_collision(first, second) == 0;
}

double clearance(const Box& first, const Box& second) {
    if (overlaps(first, second)) return 0;
    const auto first_corners = corners(first);
    const auto second_corners = corners(second);
    double result = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < first_corners.size(); ++index) {
        const auto next = (index + 1) % first_corners.size();
        for (const auto point : first_corners) {
            result = std::min(result, point_segment(point, second_corners[index], second_corners[next]));
        }
        for (const auto point : second_corners) {
            result = std::min(result, point_segment(point, first_corners[index], first_corners[next]));
        }
    }
    return result;
}

bool on_road(const Box& box, const std::vector<Polygon>& lanes) {
    const auto covered = [&](Vec2 point) {
        return std::any_of(lanes.begin(), lanes.end(), [&](const auto& lane) { return contains(lane, point); });
    };
    const auto footprint = corners(box);
    for (std::size_t index = 0; index < footprint.size(); ++index) {
        const auto start = footprint[index];
        const auto end = footprint[(index + 1) % footprint.size()];
        if (!covered(start) || !covered({(start.x + end.x) / 2, (start.y + end.y) / 2})) return false;
    }
    return covered(box.state.position);
}

}