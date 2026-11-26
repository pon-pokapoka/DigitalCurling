// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

#include <algorithm>
#include <cmath>
#include "free_motion_curve.hpp"

namespace digitalcurling::simulators::fcv1event {

FreeMotionCurve const& FreeMotionCurve::Get()
{
    static FreeMotionCurve const curve;
    return curve;
}

FreeMotionCurve::FreeMotionCurve()
{
    // fcv1's frame, in double precision. Starts above kMaxSpeed to leave room for
    // dropping the first few frames.
    std::vector<Point> frames;
    double speed = kMaxSpeed + 0.01;
    double x = 0., y = 0., arc = 0., heading = 0., spin_loss = 0., spin_sum = 0.;
    while (true) {
        frames.push_back({
            x, y, arc, spin_sum,
            static_cast<float>(std::cos(heading)), static_cast<float>(std::sin(heading)),
            static_cast<float>(heading), static_cast<float>(spin_loss), static_cast<float>(speed)
        });
        if (speed <= 0.) break;

        double const next = speed - (0.00200985 / (speed + 0.06385782) + 0.00626286) * 9.80665 * kFrameSeconds;
        spin_loss += 0.025 / std::max(speed, 0.001) * kFrameSeconds;
        spin_sum += spin_loss;
        if (next > 0.) {
            heading += 0.00820 * std::pow(speed, -0.8) * kFrameSeconds;
            speed = next;
        } else {
            speed = 0.;
        }
        x += speed * std::cos(heading) * kFrameSeconds;
        y += speed * std::sin(heading) * kFrameSeconds;
        arc += speed * kFrameSeconds;
    }

    // Keep every kStride-th frame so that the last one is the stopped stone.
    std::size_t const first = (frames.size() - 1) % kStride;
    for (std::size_t n = first; n < frames.size(); n += kStride) points_.push_back(frames[n]);
    last_ = static_cast<int>(points_.size() - 1) * kStride;

    grid_.resize(kGridSize + 1);
    int const last_point = static_cast<int>(points_.size()) - 1;
    int p = 0;
    for (int g = kGridSize; g >= 0; --g) {
        float const speed_at_grid = static_cast<float>(g) * kMaxSpeed / static_cast<float>(kGridSize);
        while (p < last_point && points_[p + 1].speed >= speed_at_grid) ++p;
        grid_[g] = p;
    }
}

double FreeMotionCurve::Find(float speed) const
{
    int const last_point = static_cast<int>(points_.size()) - 1;
    int const g = std::clamp(static_cast<int>(speed * (kGridSize / kMaxSpeed)) + 1, 0, kGridSize);
    int p = grid_[g];
    while (p < last_point && points_[p + 1].speed >= speed) ++p;
    if (p >= last_point) return last_;
    Point const& a = points_[p];
    Point const& b = points_[p + 1];
    return kStride * (p + std::clamp(static_cast<double>(a.speed - speed) / static_cast<double>(a.speed - b.speed), 0., 1.));
}

FreeMotionCurve::Point FreeMotionCurve::At(double index) const
{
    if (index >= last_) return points_.back();
    double const position = index * (1. / kStride);
    int const p = static_cast<int>(position);
    double const f = position - p;
    float const g = static_cast<float>(f);
    Point const& a = points_[p];
    Point const& b = points_[p + 1];
    return {
        a.x + f * (b.x - a.x),
        a.y + f * (b.y - a.y),
        a.arc + f * (b.arc - a.arc),
        a.spin_sum + f * (b.spin_sum - a.spin_sum),
        a.ux + g * (b.ux - a.ux),
        a.uy + g * (b.uy - a.uy),
        a.heading + g * (b.heading - a.heading),
        a.spin_loss + g * (b.spin_loss - a.spin_loss),
        a.speed + g * (b.speed - a.speed)
    };
}

} // namespace digitalcurling::simulators::fcv1event
