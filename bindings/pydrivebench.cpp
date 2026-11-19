#include "drivebench/evaluator.hpp"

#include <cmath>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

namespace py = pybind11;
using namespace drivebench;

namespace {

py::object finite(double value) {
    if (!std::isfinite(value)) return py::none();
    return py::float_(value);
}

py::dict metrics_dict(const Metrics& metrics) {
    using namespace py::literals;
    return py::dict("collision"_a=metrics.collision, "min_ttc"_a=finite(metrics.min_ttc),
                    "min_distance"_a=py::dict("vehicle"_a=finite(metrics.min_distance[0]),
                        "pedestrian"_a=finite(metrics.min_distance[1]), "cyclist"_a=finite(metrics.min_distance[2])),
                    "goal_reached"_a=metrics.goal_reached, "off_road_events"_a=metrics.off_road_events,
                    "distance_traveled"_a=metrics.distance_traveled, "route_length"_a=metrics.route_length,
                    "stop_sign_violations"_a=metrics.stop_sign_violations, "speed_limit_violations"_a=metrics.speed_limit_violations,
                    "lane_departures"_a=metrics.lane_departures, "max_acceleration"_a=metrics.max_acceleration,
                    "max_jerk"_a=metrics.max_jerk, "harsh_brakes"_a=metrics.harsh_brakes);
}

py::dict box_dict(const Box& box) {
    using namespace py::literals;
    return py::dict("position"_a=py::make_tuple(box.state.position.x, box.state.position.y),
                    "heading"_a=box.state.heading, "speed"_a=box.state.speed,
                    "length"_a=box.length, "width"_a=box.width);
}

py::list actor_list(const std::vector<ActorBox>& actors) {
    constexpr const char* names[]{"vehicle", "pedestrian", "cyclist"};
    py::list result;
    for (const auto& actor : actors) {
        auto entry = box_dict(actor.box);
        entry["id"] = actor.id;
        entry["class"] = names[static_cast<std::size_t>(actor.kind)];
        result.append(std::move(entry));
    }
    return result;
}

py::list trajectory_list(const std::vector<Frame>& frames) {
    using namespace py::literals;
    py::list result;
    for (const auto& frame : frames) {
        result.append(py::dict("tick"_a=frame.tick, "ego"_a=box_dict(frame.ego),
                              "actors"_a=actor_list(frame.actors), "perceived"_a=actor_list(frame.perceived),
                              "action"_a=py::dict("acceleration"_a=frame.action.acceleration, "steering"_a=frame.action.steering),
                              "on_road"_a=frame.on_road, "goal"_a=frame.goal,
                              "route_progress"_a=frame.route.progress, "lane_deviation"_a=frame.route.deviation));
    }
    return result;
}

}

PYBIND11_MODULE(pydrivebench, module) {
    module.doc() = "DriveBench closed-loop simulation and evaluation";
    module.attr("timestep") = timestep;
    py::class_<Verdict>(module, "Verdict")
        .def_readonly("passed", &Verdict::passed)
        .def_readonly("failures", &Verdict::failures);
    py::class_<RunResult>(module, "RunResult")
        .def_readonly("scenario", &RunResult::scenario)
        .def_readonly("driver_version", &RunResult::driver_version)
        .def_readonly("seed", &RunResult::seed)
        .def_readonly("verdict", &RunResult::verdict)
        .def_property_readonly("metrics", [](const RunResult& result) { return metrics_dict(result.metrics); })
        .def_property_readonly("trajectory", [](const RunResult& result) { return trajectory_list(result.trajectory); })
        .def("trajectory_bytes", [](const RunResult& result) { return py::bytes(trajectory_bytes(result.trajectory)); });
    module.def("run_scenario", &run_scenario, py::arg("scenario_path"), py::arg("driver_version"), py::arg("seed"),
               py::call_guard<py::gil_scoped_release>());
    module.def("summarize", [](const std::vector<Verdict>& verdicts) {
        using namespace py::literals;
        const auto result = summarize(verdicts);
        return py::dict("runs"_a=result.runs, "passed"_a=result.passed, "pass_rate"_a=result.pass_rate, "failures"_a=result.failures);
    });
}