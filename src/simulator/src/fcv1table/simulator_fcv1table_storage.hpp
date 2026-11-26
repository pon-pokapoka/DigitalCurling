// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

/// @file
/// @brief Defines SimulatorFCV1TableStorage

#pragma once

#include <memory>
#include <vector>
#include <nlohmann/json.hpp>
#include "digitalcurling/simulators/i_simulator_storage.hpp"
#include "simulator_fcv1table_factory.hpp"

namespace digitalcurling::simulators {

/// @brief Storage of the simulator FCV1Table
///
/// Holds the simulator's whole state, so a loaded simulator continues exactly as
/// the saved one would have.
class SimulatorFCV1TableStorage : public ISimulatorStorage {
public:
    /// @brief Default constructor
    SimulatorFCV1TableStorage() = default;
    /// @brief Copy constructor
    SimulatorFCV1TableStorage(SimulatorFCV1TableStorage const&) = default;
    /// @brief Copy assignment operator
    SimulatorFCV1TableStorage & operator = (SimulatorFCV1TableStorage const&) = default;
    /// @brief Constructor
    /// @param factory Factory to initialize the storage with
    SimulatorFCV1TableStorage(SimulatorFCV1TableFactory const& factory);
    virtual ~SimulatorFCV1TableStorage() override = default;

    virtual const char* GetId() const noexcept override { return DIGITALCURLING_PLUGIN_NAME; }
    virtual nlohmann::json ToJson() const override;

    virtual std::unique_ptr<ISimulator> CreateSimulator() const override;

    /// @brief Factory of the saved simulator
    SimulatorFCV1TableFactory factory;
    /// @brief Positions and velocities of all stones
    ISimulator::AllStones stones;
    /// @brief Collisions of the last step
    std::vector<ISimulator::Collision> collisions;
};


/// @cond Doxygen_Suppress
// json
void to_json(nlohmann::json &, SimulatorFCV1TableStorage const&);
void from_json(nlohmann::json const&, SimulatorFCV1TableStorage &);
/// @endcond

} // namespace digitalcurling::simulators
