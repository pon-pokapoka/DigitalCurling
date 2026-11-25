// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

/// @file
/// @brief Defines SimulatorFCV1EventFactory

#pragma once

#include <memory>
#include <nlohmann/json.hpp>
#include "digitalcurling/simulators/i_simulator.hpp"
#include "digitalcurling/simulators/i_simulator_factory.hpp"
#include "fcv1event.hpp"

namespace digitalcurling::simulators {

/// @brief Factory of the simulator FCV1Event
class SimulatorFCV1EventFactory : public ISimulatorFactory {
public:
    /// @brief Shortest allowed max_step_seconds other than 0 [s]
    static constexpr float kMinStepSeconds = 0.001f;

    /// @brief Frame length of the underlying fcv1 model [s]
    ///
    /// The curve holds fcv1's motion for 0.001. With any other value the simulator
    /// doesn't look for events: each `Step()` runs one Box2D frame of this length,
    /// as fcv1 does, and max_step_seconds is ignored. Must be finite and positive.
    float seconds_per_frame = 0.001f;

    /// @brief Longest time one `Step()` advances [s], or 0 for no limit
    ///
    /// Rounded to whole frames; must be 0, or finite and at least kMinStepSeconds.
    /// Without a limit a `Step()` goes on to the next event; see `SimulatorFCV1Event::Step()`.
    float max_step_seconds = 0.f;

    /// @brief Width of the sheet at which stones leave play [m], or 0 for no side limit
    ///
    /// Only decides where a `Step()` ends early, so that a caller that removes
    /// stones leaving the sheet between steps does it in the same frame as it
    /// would with fcv1. The simulator itself removes nothing. Must be finite and
    /// not negative.
    float sheet_width = 4.75f;

    /// @brief Default constructor
    SimulatorFCV1EventFactory() = default;
    /// @brief Copy constructor
    SimulatorFCV1EventFactory(SimulatorFCV1EventFactory const&) = default;
    /// @brief Copy assignment operator
    SimulatorFCV1EventFactory & operator = (SimulatorFCV1EventFactory const&) = default;
    virtual ~SimulatorFCV1EventFactory() override = default;

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
void to_json(nlohmann::json &, SimulatorFCV1EventFactory const&);
void from_json(nlohmann::json const&, SimulatorFCV1EventFactory &);
/// @endcond

} // namespace digitalcurling::simulators
