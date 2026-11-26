// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

/// @file
/// @brief Defines Box2DFrames

#pragma once

#include <array>
#include <vector>
#include <box2d/box2d.h>
#include "digitalcurling/simulators/i_simulator.hpp"
#include "stone_body.hpp"

namespace digitalcurling::simulators::fcv1event {

/// @brief A run of consecutive fcv1 frames in a Box2D world made for it
///
/// For states that the faster ways don't cover: other frame lengths, very fast
/// stones, and stones that overlap deeply.
class Box2DFrames : private b2ContactListener {
public:
    /// @brief Constructor
    /// @param bodies the stones
    /// @param count number of stones
    /// @param collisions contacts of moving stones are added to this
    Box2DFrames(StoneBody const* bodies, int count, std::vector<ISimulator::Collision> & collisions);

    /// @brief Advance by one frame
    /// @param bodies the stones, as this object left them
    /// @param seconds length of the frame
    void Advance(StoneBody * bodies, float seconds);

private:
    b2World world_;
    std::array<b2Body*, StoneCoordinate::kStoneMax> bodies_;
    int count_;
    std::vector<ISimulator::Collision> & collisions_;

    virtual void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) override;
};

} // namespace digitalcurling::simulators::fcv1event
