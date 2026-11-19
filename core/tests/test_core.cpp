#include "drivebench/world.hpp"

#include <cmath>
#include <iostream>

int main() {
    const auto state = drivebench::bicycle_step({{0, 0}, 0, 5}, {2, 0}, 2.7, 1);
    if (std::abs(state.position.x - 0.51) > 1e-12 || state.position.y != 0 ||
        state.heading != 0 || std::abs(state.speed - 5.2) > 1e-12) {
        std::cerr << "Unexpected fixed-step kinematics\n";
        return 1;
    }
    const drivebench::Box moving{{{0, 0}, 0, 5}, 4, 2};
    const drivebench::Box parked{{{10, 0}, 0, 0}, 4, 2};
    if (drivebench::overlaps(moving, parked) || drivebench::clearance(moving, parked) != 6 ||
        std::abs(drivebench::time_to_collision(moving, parked) - 1.2) > 1e-12) {
        std::cerr << "Unexpected oriented-box geometry\n";
        return 1;
    }
}