// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

#include <memory>
#include <nlohmann/json.hpp>
#include "digitalcurling/common.hpp"
#include "simulator_fcv1light_factory.hpp"
#include "simulator_fcv1light.hpp"

namespace digitalcurling::simulators {

nlohmann::json SimulatorFCV1LightFactory::ToJson() const {
    nlohmann::json j;
    to_json(j, *this);
    return j;
}

std::unique_ptr<ISimulator> SimulatorFCV1LightFactory::CreateSimulator() const {
    return std::make_unique<SimulatorFCV1Light>(*this);
}

std::unique_ptr<ISimulatorFactory> SimulatorFCV1LightFactory::Clone() const {
    return std::make_unique<SimulatorFCV1LightFactory>(*this);
}

// json
void to_json(nlohmann::json & j, SimulatorFCV1LightFactory const& v) {
    j["type"] = DIGITALCURLING_PLUGIN_NAME;
    j["seconds_per_frame"] = v.seconds_per_frame;
}
void from_json(nlohmann::json const& j, SimulatorFCV1LightFactory & v) {
    j.at("seconds_per_frame").get_to(v.seconds_per_frame);
}

} // namespace digitalcurling::simulators
