// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include "box2d_frames.hpp"

namespace digitalcurling::simulators::fcv1event {

namespace {

// A stone this close to another starts awake [m]
constexpr float kWakeMargin = 0.01f;

inline bool IsMoving(b2Body const& body)
{
    return body.GetLinearVelocity().LengthSquared() > kEpsilon
        || std::abs(body.GetAngularVelocity()) > kEpsilon;
}

} // unnamed namespace


Box2DFrames::Box2DFrames(StoneBody const* bodies, int count, std::vector<ISimulator::Collision> & collisions)
    : world_(b2Vec2_zero)
    , bodies_()
    , count_(count)
    , collisions_(collisions)
{
    world_.SetContactListener(this);

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

    for (int i = 0; i < count; ++i) {
        StoneBody const& body = bodies[i];
        // Contacts between sleeping stones aren't registered, so stones that may touch another are awake.
        body_def.awake = body.IsActive();
        for (int j = 0; j < count && !body_def.awake; ++j) {
            double const ex = bodies[j].x - body.x;
            double const ey = bodies[j].y - body.y;
            body_def.awake = j != i && ex * ex + ey * ey < (kDiameter + kWakeMargin) * (kDiameter + kWakeMargin);
        }
        body_def.position.Set(static_cast<float>(body.x), static_cast<float>(body.y));
        body_def.angle = static_cast<float>(body.angle);
        body_def.linearVelocity.Set(body.vx, body.vy);
        body_def.angularVelocity = body.spin;
        body_def.userData.pointer = static_cast<uintptr_t>(body.id);
        bodies_[i] = world_.CreateBody(&body_def);
        bodies_[i]->CreateFixture(&fixture_def);
    }
}

void Box2DFrames::Advance(StoneBody * bodies, float seconds)
{
    for (int i = 0; i < count_; ++i) {
        b2Body * const body = bodies_[i];
        b2Vec2 heading = body->GetLinearVelocity();
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

    world_.Step(
        seconds,
        8,  // velocityIterations
        3   // positionIterations
    );

    for (int i = 0; i < count_; ++i) {
        b2Body const* const from = bodies_[i];
        if (!from->IsAwake()) continue;
        StoneBody & body = bodies[i];
        body.x = from->GetPosition().x;
        body.y = from->GetPosition().y;
        body.angle = from->GetAngle();
        body.SetVelocity(from->GetLinearVelocity().x, from->GetLinearVelocity().y);
        body.spin = from->GetAngularVelocity();
    }
}

void Box2DFrames::PostSolve(b2Contact* contact, const b2ContactImpulse* impulse)
{
    b2Body * const a = contact->GetFixtureA()->GetBody();
    b2Body * const b = contact->GetFixtureB()->GetBody();
    if (!IsMoving(*a) && !IsMoving(*b)) return;

    ISimulator::Collision collision;
    auto const set = [](ISimulator::Collision::CollisionStone & stone, b2Body & body) {
        stone.id = static_cast<std::uint8_t>(body.GetUserData().pointer);
        stone.stone.position = Vector2(body.GetWorldCenter().x, body.GetWorldCenter().y);
        stone.stone.angle = body.GetAngle();
    };
    set(collision.a, *a);
    set(collision.b, *b);
    collision.normal_impulse = impulse->normalImpulses[0];
    collision.tangent_impulse = impulse->tangentImpulses[0];
    collisions_.emplace_back(std::move(collision));
}

} // namespace digitalcurling::simulators::fcv1event
