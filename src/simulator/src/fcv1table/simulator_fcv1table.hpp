// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

/// @file
/// @brief Defines SimulatorFCV1Table

#pragma once

#include <array>
#include <memory>
#include <optional>
#include <vector>
#include <box2d/box2d.h>
#include "digitalcurling/moves/shot.hpp"
#include "digitalcurling/simulators/i_simulator.hpp"

#include "simulator_fcv1table_factory.hpp"
#include "simulator_fcv1table_storage.hpp"

namespace digitalcurling::simulators {

/// @brief Fast approximation of fcv1 for search
///
/// Stones that can't touch another stone move along a table of fcv1's own motion
/// (`fcv1table::FreeMotionTable`). Where a contact can't be ruled out, the
/// simulator runs fcv1's 1 ms Box2D frames until it can be again.
///
/// The state between steps is exactly what `SimulatorFCV1TableStorage` holds: every
/// run of Box2D frames starts from a new world, so nothing is hidden in Box2D.
class SimulatorFCV1Table : public ISimulator {
public:
    /// @brief Mass of a stone [kg]
    static constexpr float kStoneMass = 19.96f;

    /// @brief Constructor
    /// @param factory Factory of this simulator
    /// @exception std::invalid_argument the factory's settings are out of range
    explicit SimulatorFCV1Table(SimulatorFCV1TableFactory const& factory);
    /// @brief Constructor
    /// @param storage Storage to start from
    /// @exception std::invalid_argument a setting is out of range or a stone is not finite
    explicit SimulatorFCV1Table(SimulatorFCV1TableStorage const& storage);
    virtual ~SimulatorFCV1Table() = default;

    virtual const char* GetId() const noexcept override { return DIGITALCURLING_PLUGIN_NAME; }

    virtual ISimulator::AllStones const& GetStones() const override;
    /// @brief Set all stones
    /// @exception std::invalid_argument a stone is not finite; the state doesn't change then
    virtual void SetStones(ISimulator::AllStones const& stones) override;

    /// @brief Advance by step_seconds, or less if the step ends early
    ///
    /// A step ends early
    /// - when a contact involving a moving stone is over: the stones are moving
    ///   apart or at rest;
    /// - in the frame in which a stone crosses the back line (by its radius), the
    ///   back board, y = 0, or the side of a sheet of sheet_width;
    /// - when all stones have stopped.
    ///
    /// Stones that keep pressing on each other for longer than step_seconds are
    /// reported in every step until they are done.
    virtual void Step() override;
    /// @brief Contacts of the last step
    ///
    /// Contacts of moving stones are listed once per frame, as Box2D reports them.
    /// Pairs that touch at rest at the end of the step are listed once, with
    /// impulse 0. Stones that overlap slightly while moving apart are not listed.
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
    /// simulators give the same shot. The direction is read from the table.
    virtual moves::Shot CalculateShot(Vector2 const& target_position, float const target_speed, float const shot_angular_velocity) const;

private:
    /// @brief A stone during a step
    struct Body {
        int id;
        double x, y;
        double angle;
        double index;        // position on the table's curve, or < 0 if not looked up yet
        float dx, dy;        // unit heading, if speed > 0
        float speed;
        float spin;
        unsigned out_lines;  // lines that end a step and that the stone is past

        bool IsActive() const;
    };

    class ContactListener : public b2ContactListener {
    public:
        explicit ContactListener(SimulatorFCV1Table * instance) : instance_(instance) {}
        virtual void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) override;
    private:
        SimulatorFCV1Table * const instance_;
    };

    SimulatorFCV1TableStorage storage_;

    // Used only during a step.
    std::array<Body, StoneCoordinate::kStoneMax> bodies_;
    int body_count_;
    std::optional<b2World> world_;
    std::array<b2Body*, StoneCoordinate::kStoneMax> world_bodies_;  // same order as bodies_
    ContactListener contact_listener_;

    static void Validate(SimulatorFCV1TableStorage const& storage);
    static void Validate(ISimulator::AllStones const& stones);

    void ReadStones();
    void WriteStones();
    bool IsAnyActive() const;
    unsigned GetOutLines(Body const& body) const;
    // Update out_lines; returns whether a stone has just crossed a line.
    bool UpdateOutLines();

    // Frames, at most max_frames, in which no moving stone can touch another; 0 if it can in the next one.
    int CountContactFreeFrames(int max_frames);
    // Frames, at most max_frames and at least 1, in which no stone can cross a line except in the last.
    int CountInsideFrames(int max_frames);

    static void AdvanceFree(Body & body, int frames);
    void BuildWorld();
    void AdvanceWorld(float seconds);
    void AddRestingContacts();
};

} // namespace digitalcurling::simulators
