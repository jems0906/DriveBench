#include "drivebench/run_result.hpp"

#include <climits>
#include <cstring>

namespace drivebench {

std::string trajectory_bytes(const std::vector<Frame>& frames) {
    static_assert(CHAR_BIT == 8 && sizeof(double) == 8 && std::numeric_limits<double>::is_iec559);
    std::string result{"DBR1"};
    const auto integer = [&](std::uint64_t value) {
        for (int shift = 0; shift < 64; shift += 8) result.push_back(static_cast<char>((value >> shift) & 255U));
    };
    const auto real = [&](double value) {
        std::uint64_t encoded{};
        std::memcpy(&encoded, &value, sizeof(encoded));
        integer(encoded);
    };
    const auto box = [&](const Box& value) {
        real(value.state.position.x);
        real(value.state.position.y);
        real(value.state.heading);
        real(value.state.speed);
        real(value.length);
        real(value.width);
    };
    const auto actors = [&](const std::vector<ActorBox>& values) {
        integer(values.size());
        for (const auto& actor : values) {
            integer(actor.id.size());
            result.append(actor.id);
            integer(static_cast<std::uint64_t>(actor.kind));
            box(actor.box);
        }
    };
    integer(frames.size());
    for (const auto& frame : frames) {
        integer(frame.tick);
        box(frame.ego);
        actors(frame.actors);
        actors(frame.perceived);
        real(frame.action.acceleration);
        real(frame.action.steering);
        integer(frame.on_road);
        integer(frame.goal);
        real(frame.route.progress);
        real(frame.route.deviation);
    }
    return result;
}

}