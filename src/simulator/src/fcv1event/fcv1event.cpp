// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

#include "simulator_fcv1event.hpp"
#include "digitalcurling/plugins/simulator_plugin_export.hpp"

DIGITALCURLING_EXPORT_INVERTIBLE_SIMULATOR_PLUGIN(
    digitalcurling::simulators::SimulatorFCV1EventFactory,
    digitalcurling::simulators::SimulatorFCV1EventStorage,
    digitalcurling::simulators::SimulatorFCV1Event,
    &digitalcurling::simulators::SimulatorFCV1Event::CalculateShot
)
