// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

#include "simulator_fcv1table.hpp"
#include "digitalcurling/plugins/simulator_plugin_export.hpp"

DIGITALCURLING_EXPORT_INVERTIBLE_SIMULATOR_PLUGIN(
    digitalcurling::simulators::SimulatorFCV1TableFactory,
    digitalcurling::simulators::SimulatorFCV1TableStorage,
    digitalcurling::simulators::SimulatorFCV1Table,
    &digitalcurling::simulators::SimulatorFCV1Table::CalculateShot
)
