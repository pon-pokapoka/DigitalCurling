// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

/// @file
/// @brief Defines StoneBody

#pragma once

#include <cmath>
#include <limits>
#include "digitalcurling/stone.hpp"

namespace digitalcurling::simulators::fcv1event {

/// @brief Mass of a stone [kg]
constexpr float kStoneMass = 19.96f;

/// @brief Distance between the centres of touching stones [m]
constexpr float kDiameter = 2.f * Stone::kRadius;

/// @brief Speeds and spins up to this count as zero, as in fcv1
constexpr float kEpsilon = std::numeric_limits<float>::epsilon();

/// @brief A stone during a step
struct StoneBody {
    int id;
    double x, y;
    double angle;
    float vx, vy;
    float speed;         ///< |(vx, vy)|, or 0 if the stone counts as stopped
    float spin;
    unsigned out_lines;  ///< lines that end a step and that the stone is past

    /// @brief Whether the stone moves or spins
    bool IsActive() const { return speed > 0.f || std::abs(spin) > kEpsilon; }

    /// @brief Set the velocity
    void SetVelocity(float new_vx, float new_vy)
    {
        vx = new_vx;
        vy = new_vy;
        speed = std::sqrt(vx * vx + vy * vy);
        if (!(speed > kEpsilon)) vx = vy = speed = 0.f;
    }
};

/// @brief Whether stones at these places touch (Box2D's test, which rounds to float first)
inline bool Touches(double ax, double ay, double bx, double by)
{
    float const ex = static_cast<float>(bx) - static_cast<float>(ax);
    float const ey = static_cast<float>(by) - static_cast<float>(ay);
    return ex * ex + ey * ey <= kDiameter * kDiameter;
}

/// @brief Whether two stones touch
inline bool Touches(StoneBody const& a, StoneBody const& b) { return Touches(a.x, a.y, b.x, b.y); }

} // namespace digitalcurling::simulators::fcv1event
