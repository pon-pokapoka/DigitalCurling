// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

#include <cmath>
#include <memory>
#include <stdexcept>
#include <nlohmann/json.hpp>
#include "digitalcurling/common.hpp"
#include "simulator_fcv1event_factory.hpp"
#include "simulator_fcv1event.hpp"

namespace digitalcurling::simulators {

nlohmann::json SimulatorFCV1EventFactory::ToJson() const {
    nlohmann::json j;
    to_json(j, *this);
    return j;
}

std::unique_ptr<ISimulator> SimulatorFCV1EventFactory::CreateSimulator() const {
    return std::make_unique<SimulatorFCV1Event>(*this);
}

std::unique_ptr<ISimulatorFactory> SimulatorFCV1EventFactory::Clone() const {
    return std::make_unique<SimulatorFCV1EventFactory>(*this);
}

void SimulatorFCV1EventFactory::Validate() const {
    // Written so that NaN fails too.
    if (!(std::isfinite(seconds_per_frame) && seconds_per_frame > 0.f))
        throw std::invalid_argument("SimulatorFCV1Event: seconds_per_frame must be finite and positive.");
    if (!(max_step_seconds == 0.f || (std::isfinite(max_step_seconds) && max_step_seconds >= kMinStepSeconds)))
        throw std::invalid_argument("SimulatorFCV1Event: max_step_seconds must be 0, or finite and at least 0.001.");
    if (!(std::isfinite(sheet_width) && sheet_width >= 0.f))
        throw std::invalid_argument("SimulatorFCV1Event: sheet_width must be finite and not negative.");
}

// json
void to_json(nlohmann::json & j, SimulatorFCV1EventFactory const& v) {
    j["type"] = DIGITALCURLING_PLUGIN_NAME;
    j["seconds_per_frame"] = v.seconds_per_frame;
    j["max_step_seconds"] = v.max_step_seconds;
    j["sheet_width"] = v.sheet_width;
}
void from_json(nlohmann::json const& j, SimulatorFCV1EventFactory & v) {
    SimulatorFCV1EventFactory const defaults;
    j.at("seconds_per_frame").get_to(v.seconds_per_frame);
    v.max_step_seconds = j.value("max_step_seconds", defaults.max_step_seconds);
    v.sheet_width = j.value("sheet_width", defaults.sheet_width);
    v.Validate();
}

} // namespace digitalcurling::simulators
