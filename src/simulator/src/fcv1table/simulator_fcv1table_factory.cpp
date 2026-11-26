// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

#include <cmath>
#include <memory>
#include <stdexcept>
#include <nlohmann/json.hpp>
#include "digitalcurling/common.hpp"
#include "simulator_fcv1table_factory.hpp"
#include "simulator_fcv1table.hpp"

namespace digitalcurling::simulators {

nlohmann::json SimulatorFCV1TableFactory::ToJson() const {
    nlohmann::json j;
    to_json(j, *this);
    return j;
}

std::unique_ptr<ISimulator> SimulatorFCV1TableFactory::CreateSimulator() const {
    return std::make_unique<SimulatorFCV1Table>(*this);
}

std::unique_ptr<ISimulatorFactory> SimulatorFCV1TableFactory::Clone() const {
    return std::make_unique<SimulatorFCV1TableFactory>(*this);
}

void SimulatorFCV1TableFactory::Validate() const {
    // Written so that NaN fails too.
    if (!(std::isfinite(seconds_per_frame) && seconds_per_frame > 0.f))
        throw std::invalid_argument("SimulatorFCV1Table: seconds_per_frame must be finite and positive.");
    if (!(step_seconds >= kMinStepSeconds && step_seconds <= kMaxStepSeconds))
        throw std::invalid_argument("SimulatorFCV1Table: step_seconds must be between 0.001 and 10.");
    if (!(std::isfinite(sheet_width) && sheet_width >= 0.f))
        throw std::invalid_argument("SimulatorFCV1Table: sheet_width must be finite and not negative.");
}

// json
void to_json(nlohmann::json & j, SimulatorFCV1TableFactory const& v) {
    j["type"] = DIGITALCURLING_PLUGIN_NAME;
    j["seconds_per_frame"] = v.seconds_per_frame;
    j["step_seconds"] = v.step_seconds;
    j["sheet_width"] = v.sheet_width;
}
void from_json(nlohmann::json const& j, SimulatorFCV1TableFactory & v) {
    SimulatorFCV1TableFactory const defaults;
    j.at("seconds_per_frame").get_to(v.seconds_per_frame);
    v.step_seconds = j.value("step_seconds", defaults.step_seconds);
    v.sheet_width = j.value("sheet_width", defaults.sheet_width);
    v.Validate();
}

} // namespace digitalcurling::simulators
