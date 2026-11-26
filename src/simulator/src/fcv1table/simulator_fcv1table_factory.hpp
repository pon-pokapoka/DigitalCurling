// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

/// @file
/// @brief Defines SimulatorFCV1TableFactory

#pragma once

#include <memory>
#include <nlohmann/json.hpp>
#include "digitalcurling/simulators/i_simulator.hpp"
#include "digitalcurling/simulators/i_simulator_factory.hpp"
#include "fcv1table.hpp"

namespace digitalcurling::simulators {

/// @brief Factory of the simulator FCV1Table
class SimulatorFCV1TableFactory : public ISimulatorFactory {
public:
    /// @brief Shortest allowed step_seconds [s]
    static constexpr float kMinStepSeconds = 0.001f;
    /// @brief Longest allowed step_seconds [s]
    static constexpr float kMaxStepSeconds = 10.f;

    /// @brief Frame length of the underlying fcv1 model [s]
    ///
    /// The table holds fcv1's motion for 0.001. With any other value the
    /// simulator doesn't use the table: each `Step()` runs one Box2D frame of this
    /// length, as fcv1 does, and step_seconds is ignored. Must be finite and
    /// positive.
    float seconds_per_frame = 0.001f;

    /// @brief Longest time one `Step()` advances [s]
    ///
    /// Rounded to whole frames; must be in [kMinStepSeconds, kMaxStepSeconds].
    /// A `Step()` covers less time when it ends early; see `SimulatorFCV1Table::Step()`.
    float step_seconds = 1.f;

    /// @brief Width of the sheet at which stones leave play [m], or 0 for no side limit
    ///
    /// Only decides where a `Step()` ends early, so that a caller that removes
    /// stones leaving the sheet between steps does it in the same frame as it
    /// would with fcv1. The simulator itself removes nothing. Must be finite and
    /// not negative.
    float sheet_width = 4.75f;

    /// @brief Default constructor
    SimulatorFCV1TableFactory() = default;
    /// @brief Copy constructor
    SimulatorFCV1TableFactory(SimulatorFCV1TableFactory const&) = default;
    /// @brief Copy assignment operator
    SimulatorFCV1TableFactory & operator = (SimulatorFCV1TableFactory const&) = default;
    virtual ~SimulatorFCV1TableFactory() override = default;

    virtual const char* GetId() const noexcept override { return DIGITALCURLING_PLUGIN_NAME; }
    virtual nlohmann::json ToJson() const override;

    virtual std::unique_ptr<ISimulator> CreateSimulator() const override;
    virtual std::unique_ptr<ISimulatorFactory> Clone() const override;

    /// @brief Check the settings
    /// @exception std::invalid_argument a setting is out of range
    void Validate() const;
};


/// @cond Doxygen_Suppress
// json
void to_json(nlohmann::json &, SimulatorFCV1TableFactory const&);
void from_json(nlohmann::json const&, SimulatorFCV1TableFactory &);
/// @endcond

} // namespace digitalcurling::simulators
