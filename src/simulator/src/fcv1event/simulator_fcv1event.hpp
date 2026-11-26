// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

/// @file
/// @brief Defines SimulatorFCV1Event

#pragma once

#include <array>
#include <memory>
#include <vector>
#include "digitalcurling/moves/shot.hpp"
#include "digitalcurling/simulators/i_simulator.hpp"

#include "event_search.hpp"
#include "free_path.hpp"
#include "simulator_fcv1event_factory.hpp"
#include "simulator_fcv1event_storage.hpp"
#include "stone_body.hpp"

namespace digitalcurling::simulators {

/// @brief Fast approximation of fcv1 for search that goes from event to event
///
/// Between events, stones move along a curve of fcv1's own motion
/// (`fcv1event::FreeMotionCurve`). The next event is searched for on that curve,
/// frame by frame in effect, so a contact starts in the frame in which it would in
/// fcv1. Only frames that start with a contact are solved (`fcv1event::AdvanceContactFrame`).
///
/// The state between steps is exactly what `SimulatorFCV1EventStorage` holds.
class SimulatorFCV1Event : public ISimulator {
public:
    /// @brief Constructor
    /// @param factory Factory of this simulator
    /// @exception std::invalid_argument the factory's settings are out of range
    explicit SimulatorFCV1Event(SimulatorFCV1EventFactory const& factory);
    /// @brief Constructor
    /// @param storage Storage to start from
    /// @exception std::invalid_argument a setting is out of range or a stone is not finite
    explicit SimulatorFCV1Event(SimulatorFCV1EventStorage const& storage);
    virtual ~SimulatorFCV1Event() = default;

    virtual const char* GetId() const noexcept override { return DIGITALCURLING_PLUGIN_NAME; }

    virtual ISimulator::AllStones const& GetStones() const override;
    /// @brief Set all stones
    /// @exception std::invalid_argument a stone is not finite; the state doesn't change then
    virtual void SetStones(ISimulator::AllStones const& stones) override;

    /// @brief Advance to the next event
    ///
    /// Events are
    /// - the end of a contact involving a moving stone: the stones are moving apart
    ///   or at rest;
    /// - the frame in which a stone crosses the back line (by its radius), the back
    ///   board, y = 0, or the side of a sheet of sheet_width;
    /// - all stones having stopped;
    /// - max_step_seconds having passed, if that is set.
    virtual void Step() override;
    /// @brief Contacts of the last step
    ///
    /// Contacts of moving stones are listed once per frame. Pairs that touch at rest
    /// at the end of the step are listed once, with impulse 0. Stones that overlap
    /// slightly while moving apart are not listed.
    virtual std::vector<Collision> const& GetCollisions() const override;
    /// @brief Whether all stones have stopped moving and spinning
    ///
    /// Unlike fcv1's, this takes spin in both directions into account.
    virtual bool AreAllStonesStopped() const override;
    /// @brief The factory's seconds_per_frame, not the length of a step
    virtual float GetSecondsPerFrame() const override;

    virtual ISimulatorFactory const& GetFactory() const override;

    virtual std::unique_ptr<ISimulatorStorage> CreateStorage() const override;
    virtual void Save(ISimulatorStorage & storage) const override;
    virtual void Load(ISimulatorStorage const& storage) override;

    /// @brief Estimate the shot that passes a target position at a target speed
    /// @param target_position Target position
    /// @param target_speed Speed at the target position
    /// @param shot_angular_velocity Spin of the shot; only its sign is used
    /// @return The estimated shot
    /// @note The speed comes from the same regression as fcv1's, so that both
    /// simulators give the same shot. The direction is read from the curve.
    virtual moves::Shot CalculateShot(Vector2 const& target_position, float const target_speed, float const shot_angular_velocity) const;

private:
    using StoneBody = fcv1event::StoneBody;
    using FreePath = fcv1event::FreePath;

    SimulatorFCV1EventStorage storage_;

    // Used only during a step.
    std::array<StoneBody, StoneCoordinate::kStoneMax> bodies_;
    int body_count_;
    std::array<FreePath, StoneCoordinate::kStoneMax> paths_;  // same order as bodies_
    fcv1event::OutLines out_lines_;

    static void Validate(SimulatorFCV1EventStorage const& storage);
    static void Validate(ISimulator::AllStones const& stones);

    void ReadStones();
    void WriteStones();
    bool IsAnyActive() const;
    // Update out_lines of the stones; returns whether one has just crossed a line.
    bool UpdateOutLines();
    // Whether the stones are in a state that only Box2D's frames cover
    bool NeedsBox2D() const;
    // Advance along the free paths to the next event, by max_frames at most.
    // Returns the number of frames.
    int AdvanceToEvent(int max_frames);
    void AddRestingContacts();
};

} // namespace digitalcurling::simulators
