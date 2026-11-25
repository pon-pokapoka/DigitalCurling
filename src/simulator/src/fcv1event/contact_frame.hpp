// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

/// @file
/// @brief A frame in which stones touch

#pragma once

#include <vector>
#include "digitalcurling/simulators/i_simulator.hpp"
#include "stone_body.hpp"

namespace digitalcurling::simulators::fcv1event {

/// @brief Largest overlap of stones that AdvanceContactFrame() allows [m]
///
/// Box2D's slop, up to which it doesn't push stones apart.
constexpr float kMaxOverlap = 0.005f;

/// @brief Whether an active stone overlaps another by more than kMaxOverlap
bool IsOverlapTooLarge(StoneBody const* bodies, int count);

/// @brief Advance by one frame of fcv1 with contacts, without Box2D
///
/// Does what Box2D's solver does in fcv1's frames for stones, which is little: they
/// are equal circles with one contact point per pair. Left out is what doesn't
/// matter for them: position correction and continuous collision, which act on
/// larger overlaps only (see IsOverlapTooLarge()), and starting from the impulses of
/// the previous frame, which end up the same.
/// @param bodies the stones
/// @param count number of stones
/// @param collisions contacts of moving stones are added to this
void AdvanceContactFrame(StoneBody * bodies, int count, std::vector<ISimulator::Collision> & collisions);

/// @brief Whether a moving stone touches another and closes in on it in the next frame
bool IsContactLeft(StoneBody const* bodies, int count);

} // namespace digitalcurling::simulators::fcv1event
