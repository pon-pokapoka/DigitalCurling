// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

#include <memory>
#include <nlohmann/json.hpp>
#include "digitalcurling/common.hpp"
#include "simulator_fcv1table.hpp"
#include "simulator_fcv1table_storage.hpp"

namespace digitalcurling::simulators {

SimulatorFCV1TableStorage::SimulatorFCV1TableStorage(SimulatorFCV1TableFactory const& factory)
    : factory(factory)
    , stones()
    , collisions()
{}

nlohmann::json SimulatorFCV1TableStorage::ToJson() const {
    nlohmann::json j;
    to_json(j, *this);
    return j;
}

std::unique_ptr<ISimulator> SimulatorFCV1TableStorage::CreateSimulator() const {
    return std::make_unique<SimulatorFCV1Table>(*this);
}

// json
void to_json(nlohmann::json & j, SimulatorFCV1TableStorage const& v) {
    j["type"] = DIGITALCURLING_PLUGIN_NAME;
    j["factory"] = v.factory;
    j["stones"] = v.stones;
    j["collisions"] = v.collisions;
}
void from_json(nlohmann::json const& j, SimulatorFCV1TableStorage & v) {
    j.at("factory").get_to(v.factory);
    j.at("stones").get_to(v.stones);
    j.at("collisions").get_to(v.collisions);
}

} // namespace digitalcurling::simulators
