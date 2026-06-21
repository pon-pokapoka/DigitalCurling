// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

#include "simulator_fcv1light.hpp"
#include "digitalcurling/plugins/simulator_plugin_export.hpp"

DIGITALCURLING_EXPORT_INVERTIBLE_SIMULATOR_PLUGIN(
    digitalcurling::simulators::SimulatorFCV1LightFactory,
    digitalcurling::simulators::SimulatorFCV1LightStorage,
    digitalcurling::simulators::SimulatorFCV1Light,
    &digitalcurling::simulators::SimulatorFCV1Light::CalculateShot
)
