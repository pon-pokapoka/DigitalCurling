// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>
#include "digitalcurling/coordinate.hpp"
#include "box2d_frames.hpp"
#include "contact_frame.hpp"
#include "free_motion_curve.hpp"
#include "simulator_fcv1event.hpp"

namespace digitalcurling::simulators {

namespace {

using namespace fcv1event;

constexpr float kFrame = 0.001f;  // the seconds_per_frame the curve is made for

inline bool IsFinite(ISimulator::StoneState const& s)
{
    return std::isfinite(s.position.x) && std::isfinite(s.position.y) && std::isfinite(s.angle)
        && std::isfinite(s.translational_velocity.SquaredLength())  // also rejects speeds that overflow
        && std::isfinite(s.angular_velocity);
}

} // unnamed namespace


SimulatorFCV1Event::SimulatorFCV1Event(SimulatorFCV1EventFactory const& factory)
    : SimulatorFCV1Event(SimulatorFCV1EventStorage(factory))
{}

SimulatorFCV1Event::SimulatorFCV1Event(SimulatorFCV1EventStorage const& storage)
    : storage_(storage)
    , bodies_()
    , body_count_(0)
    , paths_()
    , out_lines_()
{
    Validate(storage);
}

void SimulatorFCV1Event::Validate(SimulatorFCV1EventStorage const& storage)
{
    storage.factory.Validate();
    Validate(storage.stones);
}

void SimulatorFCV1Event::Validate(ISimulator::AllStones const& stones)
{
    for (auto const& stone : stones) {
        if (stone && !IsFinite(*stone))
            throw std::invalid_argument("SimulatorFCV1Event: stone states must be finite.");
    }
}

void SimulatorFCV1Event::SetStones(ISimulator::AllStones const& stones)
{
    Validate(stones);
    storage_.stones = stones;
}

ISimulator::AllStones const& SimulatorFCV1Event::GetStones() const
{
    return storage_.stones;
}

std::vector<ISimulator::Collision> const& SimulatorFCV1Event::GetCollisions() const
{
    return storage_.collisions;
}

bool SimulatorFCV1Event::AreAllStonesStopped() const
{
    return std::none_of(storage_.stones.begin(), storage_.stones.end(), [](auto const& stone) {
        return stone && (stone->translational_velocity.SquaredLength() > kEpsilon
            || std::abs(stone->angular_velocity) > kEpsilon);
    });
}

float SimulatorFCV1Event::GetSecondsPerFrame() const
{
    return storage_.factory.seconds_per_frame;
}

ISimulatorFactory const& SimulatorFCV1Event::GetFactory() const
{
    return storage_.factory;
}

std::unique_ptr<ISimulatorStorage> SimulatorFCV1Event::CreateStorage() const
{
    return std::make_unique<SimulatorFCV1EventStorage>(storage_);
}

void SimulatorFCV1Event::Save(ISimulatorStorage & storage) const
{
    static_cast<SimulatorFCV1EventStorage &>(storage) = storage_;
}

void SimulatorFCV1Event::Load(ISimulatorStorage const& storage)
{
    auto const& new_storage = static_cast<SimulatorFCV1EventStorage const&>(storage);
    Validate(new_storage);
    storage_ = new_storage;
}

void SimulatorFCV1Event::Step()
{
    storage_.collisions.clear();
    ReadStones();

    auto const& factory = storage_.factory;
    if (factory.seconds_per_frame != kFrame) {
        // The curve doesn't apply; run one frame as fcv1 would.
        if (IsAnyActive())
            Box2DFrames(bodies_.data(), body_count_, storage_.collisions).Advance(bodies_.data(), factory.seconds_per_frame);
    } else {
        constexpr float kMaxFrames = 1e9f;
        int remaining = static_cast<int>(kMaxFrames);
        if (factory.max_step_seconds > 0.f)
            remaining = std::max(1, static_cast<int>(std::lround(std::min(kMaxFrames, factory.max_step_seconds / kFrame))));

        bool out = false;
        while (remaining > 0 && !out && storage_.collisions.empty() && IsAnyActive()) {
            if (NeedsBox2D()) {
                Box2DFrames frames(bodies_.data(), body_count_, storage_.collisions);
                do {
                    frames.Advance(bodies_.data(), kFrame);
                    --remaining;
                    out = UpdateOutLines();
                } while (remaining > 0 && !out && IsAnyActive() && NeedsBox2D());
                continue;
            }

            remaining -= AdvanceToEvent(remaining);
            out = UpdateOutLines();
            if (remaining <= 0 || out || !IsAnyActive()) break;

            do {
                AdvanceContactFrame(bodies_.data(), body_count_, storage_.collisions);
                --remaining;
                out = UpdateOutLines();
            } while (remaining > 0 && !out && IsContactLeft(bodies_.data(), body_count_) && !NeedsBox2D());
        }
    }

    AddRestingContacts();
    WriteStones();
}

void SimulatorFCV1Event::ReadStones()
{
    // Box2D limits the spin to this; without it a huge spin would never decay.
    constexpr float kMaxSpin = b2_maxRotation / kFrame;

    float const width = storage_.factory.sheet_width;
    out_lines_.x_limit = width > 0.f ? width / 2.f - Stone::kRadius : std::numeric_limits<float>::infinity();
    out_lines_.y_back_line = coordinate::kBackLineY + Stone::kRadius;
    out_lines_.y_back_board = coordinate::kBackBoardY - Stone::kRadius;

    body_count_ = 0;
    for (int i = 0; i < StoneCoordinate::kStoneMax; ++i) {
        auto const& stone = storage_.stones[i];
        if (!stone) continue;
        StoneBody & body = bodies_[body_count_++];
        body.id = i;
        body.x = stone->position.x;
        body.y = stone->position.y;
        body.angle = stone->angle;
        body.SetVelocity(stone->translational_velocity.x, stone->translational_velocity.y);
        body.spin = std::clamp(stone->angular_velocity, -kMaxSpin, kMaxSpin);
        body.out_lines = out_lines_.Get(body.x, body.y);
    }
}

void SimulatorFCV1Event::WriteStones()
{
    for (int i = 0; i < body_count_; ++i) {
        StoneBody const& body = bodies_[i];
        auto & stone = *storage_.stones[body.id];
        stone.position = Vector2(static_cast<float>(body.x), static_cast<float>(body.y));
        stone.angle = static_cast<float>(body.angle);
        stone.translational_velocity = Vector2(body.vx, body.vy);
        stone.angular_velocity = std::abs(body.spin) > kEpsilon ? body.spin : 0.f;
    }
}

bool SimulatorFCV1Event::IsAnyActive() const
{
    return std::any_of(bodies_.begin(), bodies_.begin() + body_count_, [](StoneBody const& b) { return b.IsActive(); });
}

bool SimulatorFCV1Event::UpdateOutLines()
{
    bool crossed = false;
    for (int i = 0; i < body_count_; ++i) {
        unsigned const lines = out_lines_.Get(bodies_[i].x, bodies_[i].y);
        crossed |= (lines & ~bodies_[i].out_lines) != 0;
        bodies_[i].out_lines = lines;
    }
    return crossed;
}

bool SimulatorFCV1Event::NeedsBox2D() const
{
    return std::any_of(bodies_.begin(), bodies_.begin() + body_count_, [](StoneBody const& b) { return b.speed > FreeMotionCurve::kMaxSpeed; })
        || IsOverlapTooLarge(bodies_.data(), body_count_);
}

int SimulatorFCV1Event::AdvanceToEvent(int max_frames)
{
    int last = 0;
    for (int i = 0; i < body_count_; ++i) {
        paths_[i] = FreePath(bodies_[i]);
        last = std::max(last, paths_[i].GetFrames());
    }

    int frames = std::min(max_frames, last);
    for (int i = 0; i < body_count_; ++i)
        frames = std::min(frames, FindCrossing(paths_[i], bodies_[i].out_lines, out_lines_, frames));
    for (int i = 0; i < body_count_; ++i) {
        if (bodies_[i].speed <= 0.f) continue;
        for (int j = 0; j < body_count_; ++j) {
            if (j == i || (j < i && bodies_[j].speed > 0.f)) continue;
            frames = std::min(frames, FindContact(paths_[i], paths_[j], frames));
        }
    }

    if (frames > 0) {
        for (int i = 0; i < body_count_; ++i) paths_[i].Advance(bodies_[i], frames);
    }
    return frames;
}

void SimulatorFCV1Event::AddRestingContacts()
{
    for (int i = 0; i < body_count_; ++i) {
        StoneBody const& a = bodies_[i];
        if (a.IsActive()) continue;
        for (int j = i + 1; j < body_count_; ++j) {
            StoneBody const& b = bodies_[j];
            if (b.IsActive()) continue;
            if (!Touches(a, b)) continue;

            Collision collision;
            auto const set = [](Collision::CollisionStone & stone, StoneBody const& body) {
                stone.id = static_cast<std::uint8_t>(body.id);
                stone.stone.position = Vector2(static_cast<float>(body.x), static_cast<float>(body.y));
                stone.stone.angle = static_cast<float>(body.angle);
            };
            set(collision.a, a);
            set(collision.b, b);
            storage_.collisions.emplace_back(std::move(collision));
        }
    }
}

moves::Shot SimulatorFCV1Event::CalculateShot(Vector2 const& target_position, float const target_speed, float const shot_angular_velocity) const
{
    if (!(target_speed >= 0.f))
        throw std::invalid_argument("SimulatorFCV1Event::CalculateShot: target_speed must be non-negative.");
    if (target_speed > 4.f)
        throw std::invalid_argument("SimulatorFCV1Event::CalculateShot: target_speed is too large.");

    auto const target_r = target_position.Length();
    if (!(target_r >= 0.1f))
        throw std::invalid_argument("SimulatorFCV1Event::CalculateShot: target_position is too close to the origin.");

    // fcv1's regression
    float const v0_speed = [target_r, target_speed] {
        if (target_speed <= 0.05f) {
            float constexpr kC0[] = { 0.0005048122574925176,0.2756242531609261 };
            float constexpr kC1[] = { 0.00046669575066030805,-29.898958358378636,-0.0014030973174948508 };
            float constexpr kC2[] = { 0.13968687866736632,0.41120940058777616 };

            float const c0 = kC0[0] * target_r + kC0[1];
            float const c1 = -kC1[0] * std::log(target_r + kC1[1]) + kC1[2];
            float const c2 = kC2[0] * target_r + kC2[1];

            return std::sqrt(c0 * target_speed * target_speed + c1 * target_speed + c2);
        } else if (target_speed <= 1.f) {
            float constexpr kC0[] = { -0.0014309170115803444,0.9858457898438147 };
            float constexpr kC1[] = { -0.0008339331735471273,-29.86751291726946,-0.19811799977982522 };
            float constexpr kC2[] = { 0.13967323742978,0.42816312110477517 };

            float const c0 = kC0[0] * target_r + kC0[1];
            float const c1 = -kC1[0] * std::log(target_r + kC1[1]) + kC1[2];
            float const c2 = kC2[0] * target_r + kC2[1];

            return std::sqrt(c0 * target_speed * target_speed + c1 * target_speed + c2);
        } else {
            float constexpr kC0[] = { 1.0833113118071224e-06,-0.00012132851917870833,0.004578093297561233,0.9767006869364527 };
            float constexpr kC1[] = { 0.07950648211492622,-8.228225657195706,-0.05601306077702578 };
            float constexpr kC2[] = { 0.14140440186382008,0.3875782508767419 };

            float const c0 = kC0[0] * target_r * target_r * target_r + kC0[1] * target_r * target_r + kC0[2] * target_r + kC0[3];
            float const c1 = -kC1[0] * std::log(target_r + kC1[1]) + kC1[2];
            float const c2 = kC2[0] * target_r + kC2[1];

            return std::sqrt(c0 * target_speed * target_speed + c1 * target_speed + c2);
        }
    }();

    if (target_speed >= v0_speed)
        throw std::invalid_argument("SimulatorFCV1Event::CalculateShot: target_speed is too large for the target_position.");
    // Also rejects NaN, which the regression gives for distances below its range.
    if (!(v0_speed <= FreeMotionCurve::kMaxSpeed))
        throw std::invalid_argument("SimulatorFCV1Event::CalculateShot: target_position is out of range.");

    float const angular_velocity = static_cast<float>(std::acos(-1.0) / 2.0 * (shot_angular_velocity > 0.f ? 1. : -1.));

    // Where the shot, aimed along +y, first has the target speed
    auto const& curve = FreeMotionCurve::Get();
    StoneBody body{};
    body.SetVelocity(0.f, v0_speed);
    body.spin = angular_velocity;
    auto const end = FreePath(body).At(static_cast<int>(std::ceil(curve.Find(target_speed) - curve.Find(v0_speed))));

    float const delta_angle = static_cast<float>(std::atan2(end.x, end.y));
    float const target_angle = std::atan2(target_position.y, target_position.x);
    return moves::Shot { v0_speed, angular_velocity, target_angle + delta_angle };
}

} // namespace digitalcurling::simulators
