#pragma once

#include "drivebench/run_result.hpp"

#include <map>

namespace drivebench {

struct SuiteSummary {
    std::size_t runs{};
    std::size_t passed{};
    double pass_rate{};
    std::map<std::string, std::size_t> failures;
};

Metrics measure(const std::vector<Frame>& frames, const RoadRules& rules, double planned_length);
Verdict score(const Metrics& metrics, const Criteria& criteria);
SuiteSummary summarize(const std::vector<Verdict>& verdicts);

}