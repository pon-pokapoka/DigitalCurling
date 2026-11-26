// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

#include <algorithm>
#include <cmath>
#include "free_path.hpp"

namespace digitalcurling::simulators::fcv1event {

namespace {

// Spin lost per frame by a stone that doesn't move (fcv1's rule at its lowest speed)
constexpr float kRestingSpinLoss = 0.025f;

} // unnamed namespace


FreePath::FreePath(StoneBody const& body)
    : x_(body.x)
    , y_(body.y)
    , xx_(0.), xy_(0.), yx_(0.), yy_(0.)
    , index_(0.)
    , moving_frames_(0)
    , resting_frames_(0)
    , curling_frames_(0)
    , spin_ends_(false)
    , length_(0.)
    , from_()
    , curl_end_()
    , start_()
    , stop_()
{
    float spin = std::abs(body.spin);
    if (body.speed > 0.f) {
        auto const& curve = FreeMotionCurve::Get();
        index_ = curve.Find(body.speed);
        from_ = curve.At(index_);
        auto const end = curve.At(curve.Last());
        moving_frames_ = static_cast<int>(std::ceil(curve.Last() - index_));
        length_ = end.arc - from_.arc;

        float const total_loss = end.spin_loss - from_.spin_loss;
        if (spin > kEpsilon) {
            spin_ends_ = total_loss >= spin;
            curling_frames_ = moving_frames_;
            if (spin_ends_) {
                int low = 0;  // the first frame count at which the spin is gone is in (low, curling_frames_]
                while (curling_frames_ - low > 1) {
                    int const middle = (low + curling_frames_) / 2;
                    if (curve.At(index_ + middle).spin_loss - from_.spin_loss >= spin) curling_frames_ = middle;
                    else low = middle;
                }
            }
            curl_end_ = curve.At(index_ + curling_frames_);
        } else {
            curl_end_ = from_;
        }
        spin = std::max(0.f, spin - total_loss);

        // Rotates by the difference of the headings; clockwise spin mirrors the curve.
        double const direction = body.spin > 0.f ? 1. : -1.;
        double const norm = 1. / std::sqrt(
            (static_cast<double>(body.vx) * body.vx + static_cast<double>(body.vy) * body.vy)
            * (static_cast<double>(from_.ux) * from_.ux + static_cast<double>(from_.uy) * from_.uy));
        double const cx = norm * body.vx * from_.ux;
        double const cy = norm * body.vy * from_.ux;
        double const sx = norm * body.vx * from_.uy;
        double const sy = norm * body.vy * from_.uy;
        xx_ = cx + direction * sy;
        xy_ = sx - direction * cy;
        yx_ = cy - direction * sx;
        yy_ = sy + direction * cx;
    }
    if (spin > kEpsilon) resting_frames_ = static_cast<int>(std::ceil(spin / kRestingSpinLoss));

    start_ = { 0, x_, y_, body.vx, body.vy, body.speed, 0.f };
    stop_ = At(moving_frames_);
}

FreePath::Node FreePath::At(int frame) const
{
    if (moving_frames_ == 0) return { frame, x_, y_, 0.f, 0.f, 0.f, 0.f };

    auto const point = FreeMotionCurve::Get().At(index_ + std::min(frame, moving_frames_));
    bool const curling = frame <= curling_frames_;
    auto const& curl = curling ? point : curl_end_;
    double cx = curl.x - from_.x;
    double cy = curl.y - from_.y;
    if (!curling) {
        double const straight = point.arc - curl.arc;
        cx += straight * curl.ux;
        cy += straight * curl.uy;
    }
    return {
        frame,
        x_ + xx_ * cx + xy_ * cy,
        y_ + yx_ * cx + yy_ * cy,
        point.speed * static_cast<float>(xx_ * curl.ux + xy_ * curl.uy),
        point.speed * static_cast<float>(yx_ * curl.ux + yy_ * curl.uy),
        point.speed,
        curl.heading - from_.heading
    };
}

void FreePath::Advance(StoneBody & body, int frames) const
{
    float const direction = body.spin > 0.f ? 1.f : -1.f;
    float spin = std::abs(body.spin);
    int const moved_frames = std::min(frames, moving_frames_);

    if (moved_frames > 0) {
        Node const node = At(moved_frames);
        body.x = node.x;
        body.y = node.y;
        // The interpolated heading is a little shorter than a unit vector.
        float const length = std::sqrt(node.vx * node.vx + node.vy * node.vy);
        float const scale = length > 0.f ? node.speed / length : 0.f;
        body.SetVelocity(scale * node.vx, scale * node.vy);

        if (spin > kEpsilon) {
            // The stone turns by its spin after each frame's loss; in the frame in which the spin ends, not at all.
            bool const spin_ended = spin_ends_ && moved_frames >= curling_frames_;
            int const turning_frames = spin_ended ? curling_frames_ - 1 : moved_frames;
            auto const end = FreeMotionCurve::Get().At(index_ + turning_frames);
            double const loss_sum = end.spin_sum - from_.spin_sum - turning_frames * static_cast<double>(from_.spin_loss);
            body.angle += direction * kFrameSeconds * (turning_frames * static_cast<double>(spin) - loss_sum);
            spin = spin_ended ? 0.f : std::max(0.f, spin - (end.spin_loss - from_.spin_loss));
        }
    }

    int const rested_frames = frames - moved_frames;
    if (rested_frames > 0 && spin > kEpsilon) {
        int const turning_frames = std::min(rested_frames, static_cast<int>(spin / kRestingSpinLoss));
        body.angle += direction * kFrameSeconds
            * (turning_frames * static_cast<double>(spin) - kRestingSpinLoss * 0.5 * turning_frames * (turning_frames + 1.));
        spin = std::max(0.f, spin - kRestingSpinLoss * static_cast<float>(rested_frames));
    }
    body.spin = direction * spin;
}

} // namespace digitalcurling::simulators::fcv1event
