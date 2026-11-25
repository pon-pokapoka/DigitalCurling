// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include "contact_frame.hpp"
#include "free_motion_curve.hpp"

namespace digitalcurling::simulators::fcv1event {

namespace {

constexpr float kFrame = static_cast<float>(kFrameSeconds);
constexpr float kInverseMass = 1.f / kStoneMass;
constexpr float kInverseInertia = 1.f / (0.5f * kStoneMass * Stone::kRadius * Stone::kRadius);

// Of fcv1's Box2D world
constexpr float kFriction = 0.2f;
constexpr int kVelocityIterations = 8;

struct Motion { float vx, vy, spin; };

struct Contact {
    int a, b;
    float normal_impulse, tangent_impulse;
    float nx, ny;  // from a to b
    float half_distance;
    float tangent_mass;
    float bounce_speed;
};

// fcv1's change of velocity and spin in a frame
Motion GetNextMotion(StoneBody const& body)
{
    Motion next{ body.vx, body.vy, body.spin };
    if (body.speed > 0.f) {
        constexpr float kGravity = 9.80665f;
        float const acceleration = -(0.00200985f / (body.speed + 0.06385782f) + 0.00626286f) * kGravity;
        float const new_speed = std::max(0.f, body.speed + acceleration * kFrame);
        float yaw = 0.f;
        if (std::abs(body.spin) > kEpsilon)
            yaw = (body.spin > 0.f ? 1.f : -1.f) * 0.00820f * std::pow(body.speed, -0.8f) * kFrame;
        float const longitudinal = new_speed / body.speed * std::cos(yaw);
        float const transverse = new_speed / body.speed * std::sin(yaw);
        next.vx = longitudinal * body.vx - transverse * body.vy;
        next.vy = longitudinal * body.vy + transverse * body.vx;
    }
    if (std::abs(body.spin) > kEpsilon) {
        float const loss = 0.025f / std::max(body.speed, 0.001f) * kFrame;
        next.spin = std::abs(body.spin) <= loss ? 0.f : body.spin - (body.spin > 0.f ? loss : -loss);
    }
    return next;
}

} // unnamed namespace


void AdvanceContactFrame(StoneBody * bodies, int count, std::vector<ISimulator::Collision> & collisions)
{
    constexpr int kMax = StoneCoordinate::kStoneMax;
    Motion motions[kMax];
    bool was_active[kMax];
    bool was_moving[kMax];
    int island_of[kMax];  // by seed, or -1
    std::fill_n(island_of, kMax, -1);
    for (int i = 0; i < count; ++i) {
        was_active[i] = bodies[i].IsActive();
        was_moving[i] = bodies[i].speed > 0.f;
        motions[i] = was_active[i] ? GetNextMotion(bodies[i]) : Motion{ 0.f, 0.f, 0.f };
    }

    Contact contacts[kMax * (kMax - 1) / 2];
    int contact_count = 0;

    // Stones connected by contacts are solved together, found in the order in which Box2D searches.
    for (int seed = count - 1; seed >= 0; --seed) {
        if (island_of[seed] >= 0 || !was_active[seed]) continue;

        int island[kMax];
        int island_size = 0;
        int stack[kMax];
        int stack_size = 0;
        bool done[kMax] = {};
        Contact * const begin = contacts + contact_count;
        stack[stack_size++] = seed;
        island_of[seed] = seed;
        while (stack_size > 0) {
            int const i = stack[--stack_size];
            island[island_size++] = i;
            done[i] = true;
            for (int j = count - 1; j >= 0; --j) {
                // Stones of earlier islands don't touch any other, and have already moved on.
                if (done[j] || (island_of[j] >= 0 && island_of[j] != seed) || !Touches(bodies[i], bodies[j])) continue;
                Contact & contact = contacts[contact_count++];
                contact.a = std::min(i, j);
                contact.b = std::max(i, j);
                if (island_of[j] >= 0) continue;
                island_of[j] = seed;
                stack[stack_size++] = j;
            }
        }
        Contact * const end = contacts + contact_count;

        for (Contact * c = begin; c != end; ++c) {
            float const ex = static_cast<float>(bodies[c->b].x - bodies[c->a].x);
            float const ey = static_cast<float>(bodies[c->b].y - bodies[c->a].y);
            float const distance = std::sqrt(ex * ex + ey * ey);
            c->normal_impulse = c->tangent_impulse = 0.f;
            c->nx = distance > kEpsilon ? ex / distance : 1.f;
            c->ny = distance > kEpsilon ? ey / distance : 0.f;
            c->half_distance = 0.5f * distance;
            c->tangent_mass = 0.5f / (kInverseMass + kInverseInertia * c->half_distance * c->half_distance);
            Motion const& a = motions[c->a];
            Motion const& b = motions[c->b];
            c->bounce_speed = std::max(0.f, (a.vx - b.vx) * c->nx + (a.vy - b.vy) * c->ny);  // restitution 1
        }

        auto const apply_normal = [](Contact const& c, Motion & a, Motion & b, float impulse) {
            float const x = kInverseMass * impulse * c.nx;
            float const y = kInverseMass * impulse * c.ny;
            a.vx -= x;
            a.vy -= y;
            b.vx += x;
            b.vy += y;
        };
        auto const apply_tangent = [](Contact const& c, Motion & a, Motion & b, float impulse) {
            float const x = kInverseMass * impulse * c.ny;
            float const y = kInverseMass * impulse * -c.nx;
            float const spin = kInverseInertia * impulse * c.half_distance;
            a.vx -= x;
            a.vy -= y;
            a.spin += spin;
            b.vx += x;
            b.vy += y;
            b.spin += spin;
        };

        for (int iteration = 0; iteration < kVelocityIterations; ++iteration) {
            for (Contact * c = begin; c != end; ++c) {
                Motion & a = motions[c->a];
                Motion & b = motions[c->b];

                float const slip = (b.vx - a.vx) * c->ny - (b.vy - a.vy) * c->nx + (a.spin + b.spin) * c->half_distance;
                float const max_tangent = kFriction * c->normal_impulse;
                float const tangent = std::clamp(c->tangent_impulse - c->tangent_mass * slip, -max_tangent, max_tangent);
                apply_tangent(*c, a, b, tangent - c->tangent_impulse);
                c->tangent_impulse = tangent;

                float const opening = (b.vx - a.vx) * c->nx + (b.vy - a.vy) * c->ny;
                float const normal = std::max(0.f, c->normal_impulse - 0.5f * kStoneMass * (opening - c->bounce_speed));
                apply_normal(*c, a, b, normal - c->normal_impulse);
                c->normal_impulse = normal;
            }
        }

        for (int k = 0; k < island_size; ++k) {
            StoneBody & body = bodies[island[k]];
            Motion const& motion = motions[island[k]];
            body.x += kFrameSeconds * motion.vx;
            body.y += kFrameSeconds * motion.vy;
            body.angle += kFrameSeconds * motion.spin;
            body.SetVelocity(motion.vx, motion.vy);
            body.spin = motion.spin;
        }

        for (Contact * c = begin; c != end; ++c) {
            StoneBody const& a = bodies[c->a];
            StoneBody const& b = bodies[c->b];
            if (!was_moving[c->a] && !was_moving[c->b] && a.speed <= 0.f && b.speed <= 0.f) continue;
            ISimulator::Collision collision;
            auto const set = [](ISimulator::Collision::CollisionStone & stone, StoneBody const& body) {
                stone.id = static_cast<std::uint8_t>(body.id);
                stone.stone.position = Vector2(static_cast<float>(body.x), static_cast<float>(body.y));
                stone.stone.angle = static_cast<float>(body.angle);
            };
            set(collision.a, a);
            set(collision.b, b);
            collision.normal_impulse = c->normal_impulse;
            collision.tangent_impulse = c->tangent_impulse;
            collisions.emplace_back(std::move(collision));
        }
    }
}

bool IsContactLeft(StoneBody const* bodies, int count)
{
    for (int i = 0; i < count; ++i) {
        if (bodies[i].speed <= 0.f) continue;
        for (int j = 0; j < count; ++j) {
            if (j == i || (j < i && bodies[j].speed > 0.f) || !Touches(bodies[i], bodies[j])) continue;
            Motion const a = GetNextMotion(bodies[i]);
            Motion const b = GetNextMotion(bodies[j]);
            float const ex = static_cast<float>(bodies[j].x - bodies[i].x);
            float const ey = static_cast<float>(bodies[j].y - bodies[i].y);
            if ((b.vx - a.vx) * ex + (b.vy - a.vy) * ey < 0.f) return true;
        }
    }
    return false;
}

bool IsOverlapTooLarge(StoneBody const* bodies, int count)
{
    constexpr double kMinDistance = kDiameter - kMaxOverlap;
    for (int i = 0; i < count; ++i) {
        for (int j = i + 1; j < count; ++j) {
            if (!bodies[i].IsActive() && !bodies[j].IsActive()) continue;
            double const ex = bodies[j].x - bodies[i].x;
            double const ey = bodies[j].y - bodies[i].y;
            if (ex * ex + ey * ey < kMinDistance * kMinDistance) return true;
        }
    }
    return false;
}

} // namespace digitalcurling::simulators::fcv1event
