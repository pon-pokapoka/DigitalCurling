// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

/// @file
/// @brief Defines FreePath

#pragma once

#include "free_motion_curve.hpp"
#include "stone_body.hpp"

namespace digitalcurling::simulators::fcv1event {

/// @brief The contact-less motion of one stone from its present state until it is at rest
class FreePath {
public:
    /// @brief State of the stone after some frames
    struct Node {
        int frame;
        double x, y;
        float vx, vy;
        float speed;
        float turn;  ///< change of heading since the start of the path, without its sign
    };

    /// @brief An empty path
    FreePath() = default;
    /// @brief Constructor
    /// @param body the stone; its speed must be at most FreeMotionCurve::kMaxSpeed
    explicit FreePath(StoneBody const& body);

    /// @brief Frames until the stone neither moves nor spins
    int GetFrames() const { return moving_frames_ + resting_frames_; }
    /// @brief Length of the whole path [m]
    double GetLength() const { return length_; }

    /// @brief State at the start
    Node const& GetStart() const { return start_; }
    /// @brief State when the stone stops
    Node const& GetStop() const { return stop_; }

    /// @brief State after a number of frames
    Node At(int frame) const;

    /// @brief Advance the stone this path was made from
    void Advance(StoneBody & body, int frames) const;

private:
    double x_, y_;
    double xx_, xy_, yx_, yy_;  // rotation (and reflection) from the curve to the sheet
    double index_;              // the start, on the curve
    int moving_frames_;         // until the stone stops
    int resting_frames_;        // after that, until it doesn't spin
    int curling_frames_;        // the path curls until then and is straight after it
    bool spin_ends_;            // whether the spin ends before the stone stops
    double length_;
    FreeMotionCurve::Point from_;
    FreeMotionCurve::Point curl_end_;
    Node start_;
    Node stop_;
};

} // namespace digitalcurling::simulators::fcv1event
