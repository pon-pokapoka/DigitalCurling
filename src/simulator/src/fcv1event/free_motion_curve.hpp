// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

/// @file
/// @brief Defines FreeMotionCurve

#pragma once

#include <vector>

namespace digitalcurling::simulators::fcv1event {

/// @brief Length of one fcv1 frame [s]; the time unit of the curve
constexpr double kFrameSeconds = 0.001;

/// @brief Free (contact-less) motion of one stone, tabulated from fcv1's 1 ms recurrence
///
/// Deceleration and curl depend only on speed, and curl only on the sign of the
/// spin, so a single curve that starts at kMaxSpeed describes every free motion:
/// a stone moves along the part of the curve below its own speed, rotated to its
/// heading and mirrored for clockwise spin. Spin loss is additive, so it is
/// tabulated too. Positions on the curve are frame counts ("index"), possibly
/// fractional.
class FreeMotionCurve {
public:
    /// @brief Highest speed the curve covers [m/s]
    static constexpr float kMaxSpeed = 6.f;

    /// @brief A point on the curve, which starts at the origin along +x and curls counter-clockwise
    struct Point {
        double x, y;       ///< position
        double arc;        ///< path length so far
        double spin_sum;   ///< sum of spin_loss over the frames so far, for the rotation angle
        float ux, uy;      ///< heading as a vector
        float heading;     ///< heading angle
        float spin_loss;   ///< loss of |spin| so far
        float speed;       ///< speed
    };

    /// @brief The curve, built on first use (thread-safe)
    static FreeMotionCurve const& Get();

    /// @brief Index at which the curve has the given speed
    /// @param speed speed in [0, kMaxSpeed]
    double Find(float speed) const;

    /// @brief Point at an index; indices past the end give the stopped point
    Point At(double index) const;

    /// @brief Index of the first frame at which the stone is stopped
    int Last() const { return last_; }

private:
    FreeMotionCurve();

    static constexpr int kStride = 8;  // frames between points; keeps the curve at about 0.6 MB
    static constexpr int kGridSize = 6000;

    std::vector<Point> points_;  // one per kStride frames
    std::vector<int> grid_;      // grid_[g]: last point whose speed is >= g * kMaxSpeed / kGridSize
    int last_;
};

} // namespace digitalcurling::simulators::fcv1event
