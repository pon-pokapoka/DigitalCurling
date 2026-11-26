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
#include "free_motion_table.hpp"
#include "simulator_fcv1table.hpp"

namespace digitalcurling::simulators {

namespace {

using fcv1table::FreeMotionTable;
using fcv1table::kFrameSeconds;

constexpr float kEpsilon = std::numeric_limits<float>::epsilon();
constexpr float kFrame = 0.001f;  // the seconds_per_frame the table is made for
constexpr float kDiameter = 2.f * Stone::kRadius;

// Spin lost per frame by a stone that doesn't move (fcv1's rule at its lowest speed)
constexpr float kRestingSpinLoss = 0.025f;

// A stone this close to another starts a run of Box2D frames awake [m]
constexpr float kWakeMargin = 0.01f;

enum OutLine : unsigned { kBackLine = 1, kBackBoard = 2, kFront = 4, kRightSide = 8, kLeftSide = 16 };
constexpr float kBackLineLimit = coordinate::kBackLineY + Stone::kRadius;
constexpr float kBackBoardLimit = coordinate::kBackBoardY - Stone::kRadius;

inline b2Vec2 ToB2(Vector2 v) { return { v.x, v.y }; }
inline Vector2 ToVector2(b2Vec2 v) { return { v.x, v.y }; }

inline bool IsMoving(b2Body const& body)
{
    return body.GetLinearVelocity().LengthSquared() > kEpsilon
        || std::abs(body.GetAngularVelocity()) > kEpsilon;
}

inline bool IsFinite(ISimulator::StoneState const& s)
{
    return std::isfinite(s.position.x) && std::isfinite(s.position.y) && std::isfinite(s.angle)
        && std::isfinite(s.translational_velocity.SquaredLength())  // also rejects speeds that overflow
        && std::isfinite(s.angular_velocity);
}

// Squared distance from (px, py) to the segment from the origin along the unit vector (dx, dy)
inline float SquaredDistanceToSegment(float px, float py, float dx, float dy, float length)
{
    float const t = std::clamp(px * dx + py * dy, 0.f, length);
    float const x = px - t * dx;
    float const y = py - t * dy;
    return x * x + y * y;
}

} // unnamed namespace


bool SimulatorFCV1Table::Body::IsActive() const
{
    return speed > 0.f || std::abs(spin) > kEpsilon;
}

SimulatorFCV1Table::SimulatorFCV1Table(SimulatorFCV1TableFactory const& factory)
    : SimulatorFCV1Table(SimulatorFCV1TableStorage(factory))
{}

SimulatorFCV1Table::SimulatorFCV1Table(SimulatorFCV1TableStorage const& storage)
    : storage_(storage)
    , bodies_()
    , body_count_(0)
    , world_()
    , world_bodies_()
    , contact_listener_(this)
{
    Validate(storage);
}

void SimulatorFCV1Table::Validate(SimulatorFCV1TableStorage const& storage)
{
    storage.factory.Validate();
    Validate(storage.stones);
}

void SimulatorFCV1Table::Validate(ISimulator::AllStones const& stones)
{
    for (auto const& stone : stones) {
        if (stone && !IsFinite(*stone))
            throw std::invalid_argument("SimulatorFCV1Table: stone states must be finite.");
    }
}

void SimulatorFCV1Table::SetStones(ISimulator::AllStones const& stones)
{
    Validate(stones);
    storage_.stones = stones;
}

ISimulator::AllStones const& SimulatorFCV1Table::GetStones() const
{
    return storage_.stones;
}

std::vector<ISimulator::Collision> const& SimulatorFCV1Table::GetCollisions() const
{
    return storage_.collisions;
}

bool SimulatorFCV1Table::AreAllStonesStopped() const
{
    return std::none_of(storage_.stones.begin(), storage_.stones.end(), [](auto const& stone) {
        return stone && (stone->translational_velocity.SquaredLength() > kEpsilon
            || std::abs(stone->angular_velocity) > kEpsilon);
    });
}

float SimulatorFCV1Table::GetSecondsPerFrame() const
{
    return storage_.factory.seconds_per_frame;
}

ISimulatorFactory const& SimulatorFCV1Table::GetFactory() const
{
    return storage_.factory;
}

std::unique_ptr<ISimulatorStorage> SimulatorFCV1Table::CreateStorage() const
{
    return std::make_unique<SimulatorFCV1TableStorage>(storage_);
}

void SimulatorFCV1Table::Save(ISimulatorStorage & storage) const
{
    static_cast<SimulatorFCV1TableStorage &>(storage) = storage_;
}

void SimulatorFCV1Table::Load(ISimulatorStorage const& storage)
{
    auto const& new_storage = static_cast<SimulatorFCV1TableStorage const&>(storage);
    Validate(new_storage);
    storage_ = new_storage;
}

void SimulatorFCV1Table::Step()
{
    storage_.collisions.clear();
    ReadStones();

    if (storage_.factory.seconds_per_frame != kFrame) {
        // The table doesn't apply; run one frame as fcv1 would.
        if (IsAnyActive()) {
            BuildWorld();
            AdvanceWorld(storage_.factory.seconds_per_frame);
        }
    } else {
        int remaining = std::max(1, static_cast<int>(std::lround(storage_.factory.step_seconds / kFrameSeconds)));
        while (remaining > 0 && IsAnyActive()) {
            int frames = CountContactFreeFrames(remaining);
            if (frames > 0) {
                frames = CountInsideFrames(frames);
                for (int i = 0; i < body_count_; ++i) {
                    if (bodies_[i].IsActive()) AdvanceFree(bodies_[i], frames);
                }
                remaining -= frames;
                if (UpdateOutLines()) break;
                continue;
            }

            BuildWorld();
            bool out = false;
            do {
                AdvanceWorld(kFrame);
                --remaining;
                out = UpdateOutLines();
            } while (remaining > 0 && !out && IsAnyActive() && CountContactFreeFrames(1) == 0);
            if (out || !storage_.collisions.empty()) break;
        }
    }

    AddRestingContacts();
    WriteStones();
}

void SimulatorFCV1Table::ReadStones()
{
    // Box2D limits the spin to this; without it a huge spin would never decay.
    constexpr float kMaxSpin = b2_maxRotation / kFrame;

    body_count_ = 0;
    for (int i = 0; i < StoneCoordinate::kStoneMax; ++i) {
        auto const& stone = storage_.stones[i];
        if (!stone) continue;
        Body & body = bodies_[body_count_++];
        b2Vec2 heading = ToB2(stone->translational_velocity);
        body.id = i;
        body.x = stone->position.x;
        body.y = stone->position.y;
        body.angle = stone->angle;
        body.index = -1.;
        body.speed = heading.Normalize();
        if (!(body.speed > kEpsilon)) body.speed = 0.f;
        body.dx = heading.x;
        body.dy = heading.y;
        body.spin = std::clamp(stone->angular_velocity, -kMaxSpin, kMaxSpin);
        body.out_lines = GetOutLines(body);
    }
}

void SimulatorFCV1Table::WriteStones()
{
    for (int i = 0; i < body_count_; ++i) {
        Body const& body = bodies_[i];
        auto & stone = *storage_.stones[body.id];
        stone.position = Vector2(static_cast<float>(body.x), static_cast<float>(body.y));
        stone.angle = static_cast<float>(body.angle);
        stone.translational_velocity = Vector2(body.speed * body.dx, body.speed * body.dy);
        stone.angular_velocity = std::abs(body.spin) > kEpsilon ? body.spin : 0.f;
    }
}

bool SimulatorFCV1Table::IsAnyActive() const
{
    return std::any_of(bodies_.begin(), bodies_.begin() + body_count_, [](Body const& b) { return b.IsActive(); });
}

unsigned SimulatorFCV1Table::GetOutLines(Body const& body) const
{
    float const x = static_cast<float>(body.x);
    float const y = static_cast<float>(body.y);
    unsigned lines = 0;
    if (y > kBackLineLimit) lines |= kBackLine;
    if (y > kBackBoardLimit) lines |= kBackBoard;
    if (y < 0.f) lines |= kFront;
    if (storage_.factory.sheet_width > 0.f) {
        float const x_limit = storage_.factory.sheet_width / 2.f - Stone::kRadius;
        if (x > x_limit) lines |= kRightSide;
        if (x < -x_limit) lines |= kLeftSide;
    }
    return lines;
}

bool SimulatorFCV1Table::UpdateOutLines()
{
    bool crossed = false;
    for (int i = 0; i < body_count_; ++i) {
        unsigned const lines = GetOutLines(bodies_[i]);
        crossed |= (lines & ~bodies_[i].out_lines) != 0;
        bodies_[i].out_lines = lines;
    }
    return crossed;
}

int SimulatorFCV1Table::CountContactFreeFrames(int max_frames)
{
    auto const& table = FreeMotionTable::Get();
    constexpr float kPadding = 5e-5f;  // covers rounding
    constexpr float kLimit = kDiameter + kPadding;

    for (int i = 0; i < body_count_; ++i) {
        if (bodies_[i].speed > FreeMotionTable::kMaxSpeed) return 0;
    }

    int result = max_frames;
    for (int i = 0; i < body_count_; ++i) {
        Body & a = bodies_[i];
        if (a.speed <= 0.f) continue;
        for (int j = 0; j < body_count_; ++j) {
            Body & b = bodies_[j];
            if (j == i || (j < i && b.speed > 0.f)) continue;

            // 1. Both at full speed towards each other
            float const ex = static_cast<float>(b.x - a.x);
            float const ey = static_cast<float>(b.y - a.y);
            float const distance_squared = ex * ex + ey * ey;
            float const reach = (a.speed + b.speed) * kFrame;
            float const far = kLimit + reach * static_cast<float>(result);
            if (distance_squared >= far * far) continue;

            float const distance = std::sqrt(distance_squared);
            int const lower_bound = std::max(0, static_cast<int>((distance - kLimit) / reach));
            float const opening_speed =
                ((b.speed * b.dx - a.speed * a.dx) * ex + (b.speed * b.dy - a.speed * a.dy) * ey) / distance;

            int frames = result;
            for (; frames > lower_bound; frames /= 2) {
                // Path length and change of heading of each stone, and a bound on their change of velocity
                auto const get_path = [&table, frames](Body & body, float & length, float & turn) -> float {
                    if (body.speed <= 0.f) {
                        length = turn = 0.f;
                        return 0.f;
                    }
                    if (body.index < 0.) body.index = table.Find(body.speed);
                    auto const from = table.At(body.index);
                    auto const to = table.At(body.index + frames + 1);
                    length = static_cast<float>(to.arc - from.arc);
                    turn = std::abs(body.spin) > kEpsilon ? to.heading - from.heading : 0.f;
                    return body.speed - to.speed + body.speed * turn;
                };
                float a_length, a_turn, b_length, b_turn;
                float velocity_change = get_path(a, a_length, a_turn);
                velocity_change += get_path(b, b_length, b_turn);

                // 2. Moving apart, or not closing fast enough
                float const lowest_opening_speed = opening_speed - velocity_change;
                if (lowest_opening_speed >= 0.f
                    || distance + lowest_opening_speed * static_cast<float>(frames) * kFrame >= kLimit) break;

                // 3. Straight paths, padded by how far each can curl off its heading
                float path_distance_squared = SquaredDistanceToSegment(ex, ey, a.dx, a.dy, a_length);
                if (b_length > 0.f) {
                    float const b_end_x = ex + b_length * b.dx;
                    float const b_end_y = ey + b_length * b.dy;
                    path_distance_squared = std::min({
                        path_distance_squared,
                        SquaredDistanceToSegment(b_end_x, b_end_y, a.dx, a.dy, a_length),
                        SquaredDistanceToSegment(-ex, -ey, b.dx, b.dy, b_length),
                        SquaredDistanceToSegment(a_length * a.dx - ex, a_length * a.dy - ey, b.dx, b.dy, b_length)
                    });
                    float const cross = a.dx * b.dy - a.dy * b.dx;
                    if (std::abs(cross) > 1e-9f) {
                        float const a_at = (ex * b.dy - ey * b.dx) / cross;
                        float const b_at = (ex * a.dy - ey * a.dx) / cross;
                        if (a_at >= 0.f && a_at <= a_length && b_at >= 0.f && b_at <= b_length)
                            path_distance_squared = 0.f;
                    }
                }
                float const needed = kLimit + a_length * a_turn + b_length * b_turn;
                if (path_distance_squared >= needed * needed) break;
            }
            result = std::min(result, std::max(frames, lower_bound));
            if (result == 0) return 0;
        }
    }
    return result;
}

int SimulatorFCV1Table::CountInsideFrames(int max_frames)
{
    auto const& table = FreeMotionTable::Get();
    float const width = storage_.factory.sheet_width;
    double const x_limit = width > 0.f ? width / 2.f - Stone::kRadius : std::numeric_limits<double>::infinity();

    int result = max_frames;
    for (int i = 0; i < body_count_ && result > 1; ++i) {
        Body & body = bodies_[i];
        if (body.speed <= 0.f) continue;

        // Distance to each line and the component of the heading towards it
        struct { unsigned line; double distance; float towards; } const lines[] = {
            { kBackLine, kBackLineLimit - body.y, body.dy },
            { kBackBoard, kBackBoardLimit - body.y, body.dy },
            { kFront, body.y, -body.dy },
            { kRightSide, x_limit - body.x, body.dx },
            { kLeftSide, x_limit + body.x, -body.dx },
        };
        for (auto const& l : lines) {
            if (body.out_lines & l.line) continue;
            while (result > 1 && body.speed * kFrame * static_cast<float>(result) > l.distance) {
                if (body.index < 0.) body.index = table.Find(body.speed);
                auto const from = table.At(body.index);
                auto const to = table.At(body.index + result);
                float const turn = std::abs(body.spin) > kEpsilon ? to.heading - from.heading : 0.f;
                if ((to.arc - from.arc) * std::max(0.f, l.towards + turn) <= l.distance) break;
                result /= 2;
            }
        }
    }
    return result;
}

void SimulatorFCV1Table::AdvanceFree(Body & body, int frames)
{
    float const direction = body.spin > 0.f ? 1.f : -1.f;
    float spin = std::abs(body.spin);
    int resting_frames = frames;

    if (body.speed > 0.f) {
        auto const& table = FreeMotionTable::Get();
        if (body.index < 0.) body.index = table.Find(body.speed);
        int const moving_frames = std::min(frames, static_cast<int>(std::ceil(table.Last() - body.index)));
        auto const from = table.At(body.index);
        auto const to = table.At(body.index + moving_frames);

        if (spin > kEpsilon) {
            // The path curls until the spin is gone and is straight after that.
            int curling_frames = moving_frames;
            auto curl_end = to;
            bool const spin_ends = to.spin_loss - from.spin_loss >= spin;
            if (spin_ends) {
                int low = 0;  // first frame count at which the spin is gone is in (low, curling_frames]
                while (curling_frames - low > 1) {
                    int const middle = (low + curling_frames) / 2;
                    if (table.At(body.index + middle).spin_loss - from.spin_loss >= spin) curling_frames = middle;
                    else low = middle;
                }
                curl_end = table.At(body.index + curling_frames);
            }

            // Displacement and heading in the frame of the stone's heading
            double const cx = curl_end.x - from.x;
            double const cy = curl_end.y - from.y;
            double const local_x = cx * from.ux + cy * from.uy;
            double const local_y = (-cx * from.uy + cy * from.ux) * direction;
            float heading_x = curl_end.ux * from.ux + curl_end.uy * from.uy;
            float heading_y = (-curl_end.ux * from.uy + curl_end.uy * from.ux) * direction;
            float const norm = 1.f / std::sqrt(heading_x * heading_x + heading_y * heading_y);
            heading_x *= norm;
            heading_y *= norm;

            float const dx = heading_x * body.dx - heading_y * body.dy;
            float const dy = heading_x * body.dy + heading_y * body.dx;
            double const straight = to.arc - curl_end.arc;
            body.x += local_x * body.dx - local_y * body.dy + straight * dx;
            body.y += local_x * body.dy + local_y * body.dx + straight * dy;
            body.dx = dx;
            body.dy = dy;

            // The stone turns by its spin after each frame's loss; in the frame in which the spin ends, not at all.
            int const turning_frames = spin_ends ? curling_frames - 1 : curling_frames;
            auto const turn_end = spin_ends ? table.At(body.index + turning_frames) : curl_end;
            double const loss_sum = turn_end.spin_sum - from.spin_sum - turning_frames * static_cast<double>(from.spin_loss);
            body.angle += direction * kFrameSeconds * (turning_frames * static_cast<double>(spin) - loss_sum);
            spin = spin_ends ? 0.f : spin - (to.spin_loss - from.spin_loss);
        } else {
            double const length = to.arc - from.arc;
            body.x += length * body.dx;
            body.y += length * body.dy;
        }

        body.index += moving_frames;
        body.speed = to.speed;
        resting_frames = frames - moving_frames;
    }

    if (resting_frames > 0 && spin > kEpsilon) {
        int const turning_frames = std::min(resting_frames, static_cast<int>(spin / kRestingSpinLoss));
        body.angle += direction * kFrameSeconds
            * (turning_frames * static_cast<double>(spin) - kRestingSpinLoss * 0.5 * turning_frames * (turning_frames + 1.));
        spin = std::max(0.f, spin - kRestingSpinLoss * static_cast<float>(resting_frames));
    }
    body.spin = direction * spin;
}

void SimulatorFCV1Table::BuildWorld()
{
    world_.emplace(b2Vec2_zero);
    world_->SetContactListener(&contact_listener_);

    // As in fcv1
    b2CircleShape shape;
    shape.m_radius = Stone::kRadius;

    b2FixtureDef fixture_def;
    fixture_def.shape = &shape;
    fixture_def.friction = 0.2f;
    fixture_def.restitution = 1.f;
    fixture_def.restitutionThreshold = 0.f;
    fixture_def.density = kStoneMass / (b2_pi * Stone::kRadius * Stone::kRadius);

    b2BodyDef body_def;
    body_def.type = b2_dynamicBody;
    body_def.bullet = true;

    for (int i = 0; i < body_count_; ++i) {
        Body const& body = bodies_[i];
        // Resting stones sleep, which saves most of the solver's time. Those that
        // touch another must not: contacts between sleeping stones aren't registered.
        body_def.awake = body.IsActive();
        for (int j = 0; j < body_count_ && !body_def.awake; ++j) {
            double const ex = bodies_[j].x - body.x;
            double const ey = bodies_[j].y - body.y;
            body_def.awake = j != i && ex * ex + ey * ey < (kDiameter + kWakeMargin) * (kDiameter + kWakeMargin);
        }
        body_def.position.Set(static_cast<float>(body.x), static_cast<float>(body.y));
        body_def.angle = static_cast<float>(body.angle);
        body_def.linearVelocity.Set(body.speed * body.dx, body.speed * body.dy);
        body_def.angularVelocity = body.spin;
        body_def.userData.pointer = static_cast<uintptr_t>(body.id);
        world_bodies_[i] = world_->CreateBody(&body_def);
        world_bodies_[i]->CreateFixture(&fixture_def);
    }
}

void SimulatorFCV1Table::AdvanceWorld(float seconds)
{
    // fcv1's frame
    for (int i = 0; i < body_count_; ++i) {
        b2Body * const body = world_bodies_[i];
        b2Vec2 const velocity = body->GetLinearVelocity();
        b2Vec2 heading = velocity;
        float const speed = heading.Normalize();
        float const spin = body->GetAngularVelocity();

        if (speed > kEpsilon) {
            constexpr float kGravity = 9.80665f;
            float const acceleration = -(0.00200985f / (speed + 0.06385782f) + 0.00626286f) * kGravity;
            float const new_speed = speed + acceleration * seconds;
            if (new_speed <= 0.f) {
                body->SetLinearVelocity(b2Vec2_zero);
            } else {
                float yaw = 0.f;
                if (std::abs(spin) > kEpsilon)
                    yaw = (spin > 0.f ? 1.f : -1.f) * 0.00820f * std::pow(speed, -0.8f) * seconds;
                body->SetLinearVelocity(new_speed * std::cos(yaw) * heading + new_speed * std::sin(yaw) * heading.Skew());
            }
        }

        if (std::abs(spin) > kEpsilon) {
            float const loss = 0.025f / std::max(speed, 0.001f) * seconds;
            body->SetAngularVelocity(std::abs(spin) <= loss ? 0.f : spin - (spin > 0.f ? loss : -loss));
        }
    }

    world_->Step(
        seconds,
        8,  // velocityIterations
        3   // positionIterations
    );

    for (int i = 0; i < body_count_; ++i) {
        b2Body const* const from = world_bodies_[i];
        if (!from->IsAwake()) continue;
        Body & body = bodies_[i];
        b2Vec2 heading = from->GetLinearVelocity();
        body.x = from->GetPosition().x;
        body.y = from->GetPosition().y;
        body.angle = from->GetAngle();
        body.index = -1.;
        body.speed = heading.Normalize();
        if (!(body.speed > kEpsilon)) body.speed = 0.f;
        body.dx = heading.x;
        body.dy = heading.y;
        body.spin = from->GetAngularVelocity();
    }
}

void SimulatorFCV1Table::AddRestingContacts()
{
    for (int i = 0; i < body_count_; ++i) {
        Body const& a = bodies_[i];
        if (a.IsActive()) continue;
        for (int j = i + 1; j < body_count_; ++j) {
            Body const& b = bodies_[j];
            if (b.IsActive()) continue;
            // Box2D's test for touching circles
            float const ex = static_cast<float>(b.x) - static_cast<float>(a.x);
            float const ey = static_cast<float>(b.y) - static_cast<float>(a.y);
            if (ex * ex + ey * ey > kDiameter * kDiameter) continue;

            Collision collision;
            auto const set = [](Collision::CollisionStone & stone, Body const& body) {
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

void SimulatorFCV1Table::ContactListener::PostSolve(b2Contact* contact, const b2ContactImpulse* impulse)
{
    b2Body * const a = contact->GetFixtureA()->GetBody();
    b2Body * const b = contact->GetFixtureB()->GetBody();
    if (!IsMoving(*a) && !IsMoving(*b)) return;  // see AddRestingContacts

    Collision collision;
    collision.a.id = static_cast<std::uint8_t>(a->GetUserData().pointer);
    collision.b.id = static_cast<std::uint8_t>(b->GetUserData().pointer);
    collision.a.stone.position = ToVector2(a->GetWorldCenter());
    collision.b.stone.position = ToVector2(b->GetWorldCenter());
    collision.a.stone.angle = a->GetAngle();
    collision.b.stone.angle = b->GetAngle();
    collision.normal_impulse = impulse->normalImpulses[0];
    collision.tangent_impulse = impulse->tangentImpulses[0];
    instance_->storage_.collisions.emplace_back(std::move(collision));
}

moves::Shot SimulatorFCV1Table::CalculateShot(Vector2 const& target_position, float const target_speed, float const shot_angular_velocity) const
{
    if (!(target_speed >= 0.f))
        throw std::invalid_argument("SimulatorFCV1Table::CalculateShot: target_speed must be non-negative.");
    if (target_speed > 4.f)
        throw std::invalid_argument("SimulatorFCV1Table::CalculateShot: target_speed is too large.");

    auto const target_r = target_position.Length();
    if (!(target_r >= 0.1f))
        throw std::invalid_argument("SimulatorFCV1Table::CalculateShot: target_position is too close to the origin.");

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
        throw std::invalid_argument("SimulatorFCV1Table::CalculateShot: target_speed is too large for the target_position.");
    // Also rejects NaN, which the regression gives for distances below its range.
    if (!(v0_speed <= FreeMotionTable::kMaxSpeed))
        throw std::invalid_argument("SimulatorFCV1Table::CalculateShot: target_position is out of range.");

    float const angular_velocity = static_cast<float>(std::acos(-1.0) / 2.0 * (shot_angular_velocity > 0.f ? 1. : -1.));

    // Where the shot, aimed along +y, first has the target speed
    auto const& table = FreeMotionTable::Get();
    Body body{};
    body.dy = 1.f;
    body.speed = v0_speed;
    body.spin = angular_velocity;
    body.index = table.Find(v0_speed);
    AdvanceFree(body, static_cast<int>(std::ceil(table.Find(target_speed) - body.index)));

    float const delta_angle = static_cast<float>(std::atan2(body.x, body.y));
    float const target_angle = std::atan2(target_position.y, target_position.x);
    return moves::Shot { v0_speed, angular_velocity, target_angle + delta_angle };
}

} // namespace digitalcurling::simulators
