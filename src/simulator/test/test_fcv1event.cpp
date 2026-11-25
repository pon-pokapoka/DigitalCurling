#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>
#include "common.hpp"
#include "digitalcurling/coordinate.hpp"
#include "../src/fcv1/simulator_fcv1.hpp"
#include "../src/fcv1/simulator_fcv1_factory.hpp"
#undef DIGITALCURLING_PLUGIN_NAME
#include "../src/fcv1event/simulator_fcv1event.hpp"
#include "../src/fcv1event/simulator_fcv1event_factory.hpp"
#include "../src/fcv1event/simulator_fcv1event_storage.hpp"
#include "digitalcurling/plugins/simulator_plugin_export.hpp"

namespace dct = digitalcurling::test;

namespace {

using AllStones = dcs::ISimulator::AllStones;
using StoneState = dcs::ISimulator::StoneState;

constexpr float kDiameter = 2.f * dc::Stone::kRadius;
constexpr float kPi = 3.14159265f;
constexpr int kMaxSteps = 200000;

std::unique_ptr<dcs::ISimulator> CreateEvent(float max_step_seconds = 0.f)
{
    dcs::SimulatorFCV1EventFactory factory;
    factory.max_step_seconds = max_step_seconds;
    return factory.CreateSimulator();
}

std::unique_ptr<dcs::ISimulator> CreateFCV1()
{
    return dcs::SimulatorFCV1Factory().CreateSimulator();
}

// Step until everything stops. Returns the number of steps.
int RunToStop(dcs::ISimulator & simulator, AllStones const& stones)
{
    simulator.SetStones(stones);
    int steps = 0;
    while (!simulator.AreAllStonesStopped() && steps < kMaxSteps) {
        simulator.Step();
        ++steps;
    }
    EXPECT_LT(steps, kMaxSteps);
    return steps;
}

float Distance(dc::Vector2 const& a, dc::Vector2 const& b) { return (a - b).Length(); }

// Largest distance between the same stone in both
float Difference(AllStones const& a, AllStones const& b)
{
    float result = 0.f;
    for (std::size_t i = 0; i < a.size(); ++i) {
        EXPECT_EQ(a[i].has_value(), b[i].has_value());
        if (a[i] && b[i]) result = std::max(result, Distance(a[i]->position, b[i]->position));
    }
    return result;
}

// The pairs of stones in the collisions with an impulse
std::set<std::pair<int, int>> GetPairs(dcs::ISimulator const& simulator)
{
    std::set<std::pair<int, int>> result;
    for (auto const& c : simulator.GetCollisions())
        if (c.normal_impulse > 0.f) result.emplace(std::min(c.a.id, c.b.id), std::max(c.a.id, c.b.id));
    return result;
}

// Differences of where the stones stop, sorted
std::vector<float> CompareWithFCV1(std::vector<AllStones> const& shots, float max_step_seconds = 0.f)
{
    auto fcv1 = CreateFCV1();
    auto event = CreateEvent(max_step_seconds);
    std::vector<float> result;
    for (auto const& shot : shots) {
        RunToStop(*fcv1, shot);
        RunToStop(*event, shot);
        result.push_back(Difference(fcv1->GetStones(), event->GetStones()));
    }
    std::sort(result.begin(), result.end());
    return result;
}

float Median(std::vector<float> const& sorted) { return sorted[sorted.size() / 2]; }

StoneState Moving(float x, float y, float speed, float direction, float spin)
{
    return StoneState(dc::Vector2(x, y), 0.f,
        dc::Vector2(-speed * std::sin(direction), speed * std::cos(direction)), spin);
}

StoneState Resting(float x, float y)
{
    return StoneState(dc::Vector2(x, y), 0.f, dc::Vector2(), 0.f);
}

class Random {
public:
    float operator()(float low, float high) { return low + (high - low) * (engine_() / 4294967296.f); }
    float Sign() { return (engine_() & 1) ? 1.f : -1.f; }
private:
    std::mt19937 engine_{ 20220401 };
};

} // unnamed namespace


TEST(SimulatorFCV1Event, InitialState)
{
    auto simulator = CreateEvent();
    EXPECT_TRUE(simulator->AreAllStonesStopped());
    simulator->Step();
    EXPECT_TRUE(simulator->AreAllStonesStopped());
    EXPECT_TRUE(simulator->GetCollisions().empty());
}

// --- settings ---

TEST(SimulatorFCV1Event, FactoryToJson)
{
    dcs::SimulatorFCV1EventFactory factory;
    factory.seconds_per_frame = 0.24f;
    factory.max_step_seconds = 0.5f;
    factory.sheet_width = 5.f;

    nlohmann::json const j = factory;
    EXPECT_EQ(j.at("type").get<std::string>(), "fcv1event");
    EXPECT_EQ(j.at("seconds_per_frame").get<float>(), 0.24f);
    EXPECT_EQ(j.at("max_step_seconds").get<float>(), 0.5f);
    EXPECT_EQ(j.at("sheet_width").get<float>(), 5.f);
}

TEST(SimulatorFCV1Event, FactoryFromJson)
{
    nlohmann::json const j = {
        { "type", "fcv1event" }, { "seconds_per_frame", 0.25f }, { "max_step_seconds", 0.1f }, { "sheet_width", 0.f }
    };
    auto const factory = j.get<dcs::SimulatorFCV1EventFactory>();
    EXPECT_EQ(factory.seconds_per_frame, 0.25f);
    EXPECT_EQ(factory.max_step_seconds, 0.1f);
    EXPECT_EQ(factory.sheet_width, 0.f);

    // A missing setting has its default, also when the target held another value.
    nlohmann::json const j_default = { { "type", "fcv1event" }, { "seconds_per_frame", 0.001f } };
    dcs::SimulatorFCV1EventFactory target;
    target.max_step_seconds = 3.f;
    target.sheet_width = 3.f;
    j_default.get_to(target);
    EXPECT_EQ(target.max_step_seconds, 0.f);
    EXPECT_EQ(target.sheet_width, 4.75f);
}

TEST(SimulatorFCV1Event, InvalidSettings)
{
    float const nan = std::numeric_limits<float>::quiet_NaN();
    float const inf = std::numeric_limits<float>::infinity();
    for (float const max_step_seconds : { -1.f, 0.0005f, nan, inf }) {
        dcs::SimulatorFCV1EventFactory factory;
        factory.max_step_seconds = max_step_seconds;
        EXPECT_THROW(factory.CreateSimulator(), std::invalid_argument) << max_step_seconds;
    }
    for (float const seconds_per_frame : { 0.f, -0.001f, nan, inf }) {
        dcs::SimulatorFCV1EventFactory factory;
        factory.seconds_per_frame = seconds_per_frame;
        EXPECT_THROW(factory.CreateSimulator(), std::invalid_argument) << seconds_per_frame;
    }
    for (float const sheet_width : { -1.f, nan, inf }) {
        dcs::SimulatorFCV1EventFactory factory;
        factory.sheet_width = sheet_width;
        EXPECT_THROW(factory.CreateSimulator(), std::invalid_argument) << sheet_width;
    }
    nlohmann::json const j = { { "type", "fcv1event" }, { "seconds_per_frame", 0.001f }, { "max_step_seconds", -1.f } };
    EXPECT_THROW(j.get<dcs::SimulatorFCV1EventFactory>(), std::invalid_argument);
}

TEST(SimulatorFCV1Event, MaxStepSeconds)
{
    AllStones stones;
    stones[0] = Moving(0.f, 0.f, 2.4f, 0.f, 1.5f);
    auto fcv1 = CreateFCV1();

    for (float const max_step_seconds : { 0.001f, 0.01f, 0.1f, 1.f, 10.f }) {
        auto event = CreateEvent(max_step_seconds);
        event->SetStones(stones);
        event->Step();

        fcv1->SetStones(stones);
        int const frames = static_cast<int>(std::lround(max_step_seconds * 1000.f));
        for (int i = 0; i < frames; ++i) fcv1->Step();

        auto const& expected = *fcv1->GetStones()[0];
        auto const& actual = *event->GetStones()[0];
        EXPECT_LT((expected.position - actual.position).Length(), 0.002f) << max_step_seconds;
        EXPECT_NEAR(expected.translational_velocity.Length(), actual.translational_velocity.Length(), 1e-3f) << max_step_seconds;
        EXPECT_NEAR(expected.angular_velocity, actual.angular_velocity, 1e-3f) << max_step_seconds;
        EXPECT_NEAR(expected.angle, actual.angle, 1e-2f) << max_step_seconds;
    }
}

TEST(SimulatorFCV1Event, WithoutALimitAStepReachesTheNextEvent)
{
    auto event = CreateEvent();
    AllStones stones;
    stones[0] = Moving(0.f, 0.f, 2.2f, 0.f, 1.5f);
    stones[1] = Resting(1.5f, 37.f);
    EXPECT_EQ(RunToStop(*event, stones), 1);  // nothing happens on the way

    // A long limit, even one far beyond the frames that fit in an int, is no different.
    auto limited = CreateEvent(1e30f);
    EXPECT_EQ(RunToStop(*limited, stones), 1);
    EXPECT_TRUE(dct::EqualsSimulatorStones(event->GetStones(), limited->GetStones()));
}

TEST(SimulatorFCV1Event, OtherFrameLengthFollowsFCV1)
{
    // Without its 1 ms frames the curve doesn't apply, and the simulator runs plain frames.
    AllStones stones;
    stones[0] = Moving(0.f, 30.f, 2.f, 0.f, 1.5f);
    stones[1] = Resting(0.05f, 33.f);

    dcs::SimulatorFCV1Factory fcv1_factory;
    fcv1_factory.seconds_per_frame = 0.002f;
    auto fcv1 = fcv1_factory.CreateSimulator();
    dcs::SimulatorFCV1EventFactory event_factory;
    event_factory.seconds_per_frame = 0.002f;
    auto event = event_factory.CreateSimulator();
    EXPECT_EQ(event->GetSecondsPerFrame(), 0.002f);

    fcv1->SetStones(stones);
    event->SetStones(stones);
    for (int i = 0; i < 100; ++i) {
        fcv1->Step();
        event->Step();
    }
    EXPECT_LT(Difference(fcv1->GetStones(), event->GetStones()), 1e-4f);

    RunToStop(*fcv1, stones);
    RunToStop(*event, stones);
    EXPECT_LT(Difference(fcv1->GetStones(), event->GetStones()), 0.05f);
}

// --- invalid and extreme states ---

TEST(SimulatorFCV1Event, NonFiniteStonesAreRejected)
{
    AllStones good;
    good[3] = Moving(0.f, 10.f, 2.f, 0.f, 1.f);
    auto simulator = CreateEvent();
    simulator->SetStones(good);

    for (float const bad : { std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity() }) {
        for (int field = 0; field < 6; ++field) {
            AllStones stones = good;
            auto & s = *stones[3];
            float * const fields[] = {
                &s.position.x, &s.position.y, &s.angle,
                &s.translational_velocity.x, &s.translational_velocity.y, &s.angular_velocity
            };
            *fields[field] = bad;
            EXPECT_THROW(simulator->SetStones(stones), std::invalid_argument) << field;

            dcs::SimulatorFCV1EventStorage storage;
            storage.stones = stones;
            EXPECT_THROW(simulator->Load(storage), std::invalid_argument) << field;
            EXPECT_THROW(storage.CreateSimulator(), std::invalid_argument) << field;
        }
    }
    AllStones stones = good;
    stones[3]->translational_velocity.x = 1e30f;  // its square overflows
    EXPECT_THROW(simulator->SetStones(stones), std::invalid_argument);

    EXPECT_TRUE(dct::EqualsSimulatorStones(simulator->GetStones(), good));
}

TEST(SimulatorFCV1Event, HugeSpinDecays)
{
    AllStones stones;
    stones[0] = StoneState(dc::Vector2(0.f, 10.f), 0.f, dc::Vector2(), -1e30f);
    auto simulator = CreateEvent();
    EXPECT_EQ(RunToStop(*simulator, stones), 1);
    EXPECT_EQ(simulator->GetStones()[0]->angular_velocity, 0.f);
}

TEST(SimulatorFCV1Event, OverlappingStonesTerminate)
{
    AllStones stones;
    stones[0] = Moving(0.f, 30.f, 1.f, 0.3f, 1.f);
    stones[1] = Moving(0.f, 30.f, 1.f, 0.3f, 1.f);
    stones[2] = Resting(0.1f, 30.1f);
    auto simulator = CreateEvent();
    RunToStop(*simulator, stones);
    for (int i = 0; i < 3; ++i) EXPECT_TRUE(std::isfinite(simulator->GetStones()[i]->position.Length()));
}

TEST(SimulatorFCV1Event, DeepOverlapFollowsFCV1)
{
    // Box2D's continuous collision acts on these, so they take the Box2D frames.
    for (float const overlap : { 0.012f, 0.016f, 0.03f }) {
        AllStones stones;
        stones[0] = Moving(0.f, 30.f, 1.f, 0.f, 0.f);
        stones[1] = Resting(0.f, 30.f + kDiameter - overlap);
        EXPECT_LT(CompareWithFCV1({ stones })[0], 0.002f) << overlap;
    }
}

TEST(SimulatorFCV1Event, VeryFastHeadOn)
{
    // They overlap by more than Box2D's slop in the first frame in which they touch.
    for (float const spin : { 0.f, 1.5f }) {
        AllStones stones;
        stones[0] = Moving(0.f, 20.f, 5.5f, 0.f, spin);
        stones[1] = Moving(0.02f, 22.f, 5.5f, kPi, spin);
        auto fcv1 = CreateFCV1();
        fcv1->SetStones(stones);
        while (fcv1->GetCollisions().empty()) fcv1->Step();
        auto event = CreateEvent();
        event->SetStones(stones);
        event->Step();
        ASSERT_FALSE(event->GetCollisions().empty());
        EXPECT_LT(Difference(fcv1->GetStones(), event->GetStones()), 0.03f);  // a frame is 1.1 cm
        for (int i = 0; i < 2; ++i) {
            auto const& v = fcv1->GetStones()[i]->translational_velocity;
            EXPECT_LT(Distance(v, event->GetStones()[i]->translational_velocity), 0.1f);
        }
    }
}

TEST(SimulatorFCV1Event, SpeedAboveTheCurve)
{
    AllStones stones;
    stones[0] = Moving(0.f, 0.f, 6.5f, 0.f, 1.f);
    stones[1] = Resting(0.05f, 8.f);
    auto fcv1 = CreateFCV1();
    auto event = CreateEvent();
    fcv1->SetStones(stones);
    for (int i = 0; i < 1000; ++i) fcv1->Step();
    event = CreateEvent(1.f);
    event->SetStones(stones);
    event->Step();
    EXPECT_LT(Difference(fcv1->GetStones(), event->GetStones()), 1e-3f);

    // Slows down into the curve's range and goes on with it.
    stones[1].reset();
    auto const differences = CompareWithFCV1({ stones });
    EXPECT_LT(differences[0], 0.15f);  // after about 280 m
}

// --- stopping ---

TEST(SimulatorFCV1Event, NegativeSpinIsNotStopped)
{
    auto simulator = CreateEvent();
    for (float const spin : { 1.f, -1.f }) {
        AllStones stones;
        stones[0] = StoneState(dc::Vector2(0.f, 10.f), 0.f, dc::Vector2(), spin);
        simulator->SetStones(stones);
        EXPECT_FALSE(simulator->AreAllStonesStopped());
        RunToStop(*simulator, stones);
        EXPECT_EQ(simulator->GetStones()[0]->angular_velocity, 0.f);
    }
}

TEST(SimulatorFCV1Event, NegativeSpinMirrorsPositiveSpin)
{
    auto simulator = CreateEvent();
    AllStones stones;
    stones[0] = Moving(0.f, 0.f, 2.3f, 0.f, 1.2f);
    RunToStop(*simulator, stones);
    auto const positive = *simulator->GetStones()[0];
    stones[0]->angular_velocity = -1.2f;
    RunToStop(*simulator, stones);
    auto const negative = *simulator->GetStones()[0];
    EXPECT_NE(positive.position.x, 0.f);
    EXPECT_FLOAT_EQ(negative.position.x, -positive.position.x);
    EXPECT_FLOAT_EQ(negative.position.y, positive.position.y);
    EXPECT_FLOAT_EQ(negative.angle, -positive.angle);
}

TEST(SimulatorFCV1Event, StoppedStonesAreAtRest)
{
    Random random;
    auto simulator = CreateEvent();
    for (int k = 0; k < 30; ++k) {
        AllStones stones;
        stones[0] = Moving(0.f, 20.f, random(0.1f, 2.f), random(-kPi, kPi), random(-5.f, 5.f));
        RunToStop(*simulator, stones);
        EXPECT_EQ(simulator->GetStones()[0]->angular_velocity, 0.f);
        EXPECT_LT(simulator->GetStones()[0]->translational_velocity.Length(), 4e-4f);
    }
}

TEST(SimulatorFCV1Event, SpinRunsOut)
{
    // Spin 0.3 is gone long before the stone stops; the path is straight from then on.
    std::vector<AllStones> shots;
    for (float const spin : { 0.1f, -0.3f, 0.3f, -0.6f }) {
        AllStones stones;
        stones[0] = Moving(0.f, 0.f, 2.4f, 0.f, spin);
        shots.push_back(stones);
    }
    EXPECT_LT(CompareWithFCV1(shots).back(), 0.01f);
}

// --- accuracy against fcv1 ---

TEST(SimulatorFCV1Event, Draws)
{
    Random random;
    std::vector<AllStones> shots;
    for (int k = 0; k < 40; ++k) {
        AllStones stones;
        stones[0] = Moving(0.f, 0.f, random(1.8f, 2.5f), random(-0.06f, 0.06f), random.Sign() * random(0.2f, 2.f));
        shots.push_back(stones);
    }
    for (float const max_step_seconds : { 0.f, 0.1f }) {
        auto const differences = CompareWithFCV1(shots, max_step_seconds);
        EXPECT_LT(Median(differences), 0.004f) << max_step_seconds;
        EXPECT_LT(differences.back(), 0.015f) << max_step_seconds;
    }
}

TEST(SimulatorFCV1Event, Hits)
{
    // Short paths, so that fcv1's own rounding doesn't blur the comparison
    Random random;
    std::vector<AllStones> shots;
    for (int k = 0; k < 60; ++k) {
        AllStones stones;
        stones[0] = Moving(random(-0.15f, 0.15f), 33.f, random(0.8f, 2.5f), random(-0.01f, 0.01f), random.Sign() * 1.5f);
        stones[1] = Resting(0.f, 36.f);
        stones[2] = Resting(0.5f, 37.5f);
        stones[3] = Resting(-0.4f, 38.f);
        shots.push_back(stones);
    }
    auto const differences = CompareWithFCV1(shots);
    EXPECT_LT(Median(differences), 0.005f);
    EXPECT_LT(differences[differences.size() * 9 / 10], 0.05f);
}

TEST(SimulatorFCV1Event, TwoMovingStones)
{
    Random random;
    std::vector<AllStones> shots;
    for (int k = 0; k < 40; ++k) {
        // Both reach about (0, 36) after about t
        float const t = random(0.5f, 1.2f);
        AllStones stones;
        for (int i = 0; i < 2; ++i) {
            float const direction = random(-kPi, kPi);
            float const speed = random(0.8f, 2.f);
            float const length = speed * t - 0.05f * t * t;
            stones[i] = Moving(length * std::sin(direction) + random(-0.1f, 0.1f), 36.f - length * std::cos(direction),
                speed, direction, random(-1.5f, 1.5f));
        }
        if (Distance(stones[0]->position, stones[1]->position) < kDiameter + 0.02f) continue;
        shots.push_back(stones);
    }
    ASSERT_GT(shots.size(), 25u);
    auto const differences = CompareWithFCV1(shots);
    EXPECT_LT(Median(differences), 0.015f);
    EXPECT_LT(differences[differences.size() * 8 / 10], 0.1f);
}

TEST(SimulatorFCV1Event, ManyMovingStones)
{
    Random random;
    std::vector<AllStones> shots;
    for (int k = 0; k < 20; ++k) {
        AllStones stones;
        for (int i = 0; i < 5; ++i) {
            float const a = 2.f * kPi * (static_cast<float>(i) + random(0.f, 0.3f)) / 5.f;
            stones[i] = Moving(1.2f * std::sin(a), 36.f - 1.2f * std::cos(a), random(0.3f, 1.f), a + random(-0.2f, 0.2f), random(-1.5f, 1.5f));
        }
        shots.push_back(stones);
    }
    // Several collisions in a row amplify any difference, so only the median is checked.
    EXPECT_LT(Median(CompareWithFCV1(shots)), 0.05f);
}

TEST(SimulatorFCV1Event, FrozenStones)
{
    Random random;
    std::vector<AllStones> shots;
    for (int k = 0; k < 30; ++k) {
        AllStones stones;
        stones[0] = Moving(random(-0.1f, 0.1f), 34.f, random(0.8f, 2.f), 0.f, random.Sign() * 1.5f);
        stones[1] = Resting(0.f, 36.f);
        float const a = random(-0.8f, 0.8f);
        stones[2] = Resting(kDiameter * std::sin(a), 36.f + kDiameter * std::cos(a));
        shots.push_back(stones);
    }
    auto const differences = CompareWithFCV1(shots);
    EXPECT_LT(Median(differences), 0.01f);
    EXPECT_LT(differences[differences.size() * 9 / 10], 0.1f);
}

TEST(SimulatorFCV1Event, StonesTravellingTogether)
{
    // Stone 1 follows stone 2 at less than the way they go in a frame when stone 0
    // hits it. Contacts must be looked for where all stones are at the same time.
    std::vector<AllStones> shots;
    for (float const gap : { 0.0003f, 0.0006f, 0.0009f }) {
        for (bool const reverse : { false, true }) {
            AllStones stones;
            int const order[] = { reverse ? 8 : 2, 5, reverse ? 2 : 8 };
            stones[order[0]] = Moving(0.f, 34.f, 2.f, 0.f, 0.5f);
            stones[order[1]] = Moving(0.05f, 34.f + kDiameter + 0.2f, 1.2f, 0.f, 0.5f);
            stones[order[2]] = Moving(0.05f, 34.f + 2.f * kDiameter + 0.2f + gap, 1.2f, 0.f, 0.5f);
            shots.push_back(stones);
        }
    }
    EXPECT_LT(CompareWithFCV1(shots).back(), 0.01f);
}

TEST(SimulatorFCV1Event, SearchFindsTheFirstFrame)
{
    // The search skips most frames; going through all of them must give the same one.
    namespace dce = dcs::fcv1event;
    Random random;
    int contacts = 0, returns = 0;
    for (int k = 0; k < 600; ++k) {
        int const kind = k % 4;
        dce::StoneBody bodies[2] = {};
        dc::Vector2 meeting;
        if (kind == 0) {
            // Side by side, moving apart at first and curling back
            for (int i = 0; i < 2; ++i) {
                float const side = i == 0 ? -1.f : 1.f;
                float const direction = -side * random(0.f, 0.03f);
                float const speed = random(0.9f, 1.f);
                bodies[i].SetVelocity(-speed * std::sin(direction), speed * std::cos(direction));
                bodies[i].spin = side * 1.5f;
            }
            bodies[1].x = kDiameter + random(1e-4f, 0.01f);
        } else {
            for (int i = 0; i < 2; ++i) {
                float const direction = random(-kPi, kPi);
                float const speed = i == 1 && kind == 1 ? 0.f : random(0.1f, 1.5f);
                bodies[i].SetVelocity(-speed * std::sin(direction), speed * std::cos(direction));
                bodies[i].spin = random(-1.5f, 1.5f);
            }
            // Place the second so that they are about a diameter apart after some frames, often just missing.
            dce::FreePath const at_origin[] = { dce::FreePath(bodies[0]), dce::FreePath(bodies[1]) };
            int const frame = static_cast<int>(random(0.1f, 0.9f) * static_cast<float>(at_origin[0].GetFrames()));
            float const angle = random(-kPi, kPi);
            float const distance = kDiameter + random(-0.004f, 0.002f);
            auto const p = at_origin[0].At(frame);
            auto const q = at_origin[1].At(frame);
            bodies[1].x = p.x - q.x + distance * std::cos(angle);
            bodies[1].y = p.y - q.y + distance * std::sin(angle);
            if (std::hypot(bodies[1].x, bodies[1].y) < kDiameter) continue;
            meeting = dc::Vector2(static_cast<float>(p.x), static_cast<float>(p.y));
        }

        dce::FreePath const paths[] = { dce::FreePath(bodies[0]), dce::FreePath(bodies[1]) };
        int const limit = std::max(paths[0].GetFrames(), paths[1].GetFrames());
        int expected = 0;
        for (; expected <= limit; ++expected) {
            auto const m = paths[0].At(expected);
            auto const n = paths[1].At(expected);
            if (dce::Touches(m.x, m.y, n.x, n.y) && (n.x - m.x) * (n.vx - m.vx) + (n.y - m.y) * (n.vy - m.vy) < 0.) break;
        }
        if (expected <= limit) ++(kind == 0 ? returns : contacts);
        EXPECT_EQ(dce::FindContact(paths[0], paths[1], limit), expected) << k;
        EXPECT_EQ(dce::FindContact(paths[1], paths[0], limit), expected) << k;
        EXPECT_EQ(dce::FindContact(paths[0], paths[1], expected - 1), expected) << k;

        // The same for a line
        dce::OutLines const lines{ std::abs(meeting.x) + random(0.f, 0.01f), 1e3f, 1e3f };
        unsigned const past = lines.Get(0., 0.) | 16u;
        int crossing = 0;
        while (crossing <= limit && (lines.Get(paths[0].At(crossing).x, 0.) & ~past) == 0) ++crossing;
        EXPECT_EQ(dce::FindCrossing(paths[0], past, lines, limit), crossing) << k;
    }
    EXPECT_GT(contacts, 100);
    EXPECT_GT(returns, 20);
}

TEST(SimulatorFCV1Event, NoContactIsMissedOrInvented)
{
    // Walk fcv1 through every step of crowded shots, from the state before it. Where
    // the steps report a contact, fcv1 must have one of the same stones, and where
    // they don't, it must have none.
    Random random;
    auto fcv1 = CreateFCV1();
    int steps = 0, hits = 0, missed = 0, invented = 0;
    for (int k = 0; k < 40; ++k) {
        AllStones stones;
        for (int i = 0; i < 10; ++i) {
            for (int attempt = 0; attempt < 100; ++attempt) {
                dc::Vector2 const p(random(-1.5f, 1.5f), random(34.f, 37.f));
                bool const free = std::none_of(stones.begin(), stones.end(),
                    [p](auto const& s) { return s && Distance(p, s->position) < kDiameter + 0.02f; });
                if (!free) continue;
                stones[i] = i < 4 ? Moving(p.x, p.y, random(0.2f, 1.5f), random(-kPi, kPi), random(-2.f, 2.f)) : Resting(p.x, p.y);
                break;
            }
        }
        auto event = CreateEvent(k % 2 == 0 ? 0.1f : 0.5f);
        int const max_frames = k % 2 == 0 ? 100 : 500;
        event->SetStones(stones);
        while (!event->AreAllStonesStopped()) {
            fcv1->SetStones(event->GetStones());
            event->Step();
            ++steps;
            auto const pairs = GetPairs(*event);
            std::set<std::pair<int, int>> expected;
            for (int f = 0; f < max_frames && expected.empty(); ++f) {
                fcv1->Step();
                expected = GetPairs(*fcv1);
                // The step may have ended before max_frames, at a line or with all stones at rest.
                if (pairs.empty() && Difference(fcv1->GetStones(), event->GetStones()) < 2e-4f) break;
            }
            if (!pairs.empty()) ++hits;
            if (pairs.empty() && !expected.empty()) ++missed;
            for (auto const& pair : expected)
                if (!pairs.empty() && pairs.count(pair) == 0) ++missed;
            if (!pairs.empty() && expected.empty()) ++invented;
        }
    }
    EXPECT_GT(steps, 500);
    EXPECT_GT(hits, 50);
    EXPECT_EQ(missed, 0);
    EXPECT_EQ(invented, 0);
}

TEST(SimulatorFCV1Event, NearMissesAreNotCollisions)
{
    auto simulator = CreateEvent();
    for (float const gap : { 0.0005f, 0.002f, 0.01f }) {
        AllStones stones;
        stones[0] = Moving(0.f, 30.f, 2.f, 0.f, 0.f);
        stones[1] = Resting(kDiameter + gap, 32.f);
        simulator->SetStones(stones);
        while (!simulator->AreAllStonesStopped()) {
            simulator->Step();
            EXPECT_TRUE(simulator->GetCollisions().empty());
        }
        EXPECT_EQ(simulator->GetStones()[1]->position.y, 32.f);
    }
}

// --- contacts ---

namespace {

// Slide stone 0 along +y until everything stops, with stones 1 and 2 at rest at
// x = 3, `gap` apart, well away from its path. Returns where stone 0 stops.
dc::Vector2 SlideNextToRestingPair(float gap, std::vector<dcs::ISimulator::Collision>* collisions = nullptr)
{
    auto simulator = CreateEvent();
    AllStones stones;
    stones[0] = StoneState(dc::Vector2(0.f, 0.f), 0.f, dc::Vector2(0.f, 2.5f), 1.f);
    stones[1] = Resting(3.f, 20.f);
    stones[2] = Resting(3.f + gap, 20.f);
    simulator->SetStones(stones);
    while (!simulator->AreAllStonesStopped()) {
        simulator->Step();
        if (collisions) {
            auto const& c = simulator->GetCollisions();
            collisions->insert(collisions->end(), c.begin(), c.end());
        }
    }
    return simulator->GetStones()[0]->position;
}

AllStones HeadOnHit()
{
    AllStones stones;
    stones[0] = Moving(0.f, 30.f, 2.f, 0.f, 0.f);
    stones[1] = Resting(0.f, 33.f);
    return stones;
}

} // unnamed namespace

TEST(SimulatorFCV1Event, RestingContactDoesNotChangeOtherStones)
{
    float const touching = kDiameter - 0.0005f;
    auto const apart = SlideNextToRestingPair(1.f);
    auto const near = SlideNextToRestingPair(touching);
    EXPECT_FLOAT_EQ(near.x, apart.x);
    EXPECT_FLOAT_EQ(near.y, apart.y);
}

TEST(SimulatorFCV1Event, RestingContactIsStillReported)
{
    std::vector<dcs::ISimulator::Collision> collisions;
    SlideNextToRestingPair(kDiameter - 0.0005f, &collisions);
    ASSERT_FALSE(collisions.empty());
    for (auto const& c : collisions) {
        EXPECT_EQ(std::min(c.a.id, c.b.id), 1);
        EXPECT_EQ(std::max(c.a.id, c.b.id), 2);
    }
}

TEST(SimulatorFCV1Event, HitStillMovesTheOtherStone)
{
    auto simulator = CreateEvent();
    AllStones stones;
    stones[0] = StoneState(dc::Vector2(0.f, 0.f), 0.f, dc::Vector2(0.f, 2.f), 0.f);
    stones[1] = Resting(0.f, 1.f);
    RunToStop(*simulator, stones);

    auto const& result = simulator->GetStones();
    EXPECT_GT(result[1]->position.y, 1.5f);
    EXPECT_LT(result[0]->position.y, result[1]->position.y);
}

TEST(SimulatorFCV1Event, StepEndsWhenAContactIsOver)
{
    auto simulator = CreateEvent();
    simulator->SetStones(HeadOnHit());
    simulator->Step();

    auto const& collisions = simulator->GetCollisions();
    ASSERT_FALSE(collisions.empty());
    EXPECT_EQ(std::min(collisions[0].a.id, collisions[0].b.id), 0);
    EXPECT_EQ(std::max(collisions[0].a.id, collisions[0].b.id), 1);
    EXPECT_GT(collisions[0].normal_impulse, 30.f);  // about 20 kg * 1.8 m/s
    EXPECT_LT(collisions.size(), 10u);

    auto const& stones = simulator->GetStones();
    EXPECT_NEAR(stones[1]->position.y, 33.f, 0.02f);
    EXPECT_GT(stones[1]->translational_velocity.y, 1.5f);

    // The next step has no contact to report.
    simulator->Step();
    EXPECT_TRUE(simulator->GetCollisions().empty());
}

TEST(SimulatorFCV1Event, NoContactIsLeftAfterAStep)
{
    // In a tight rack, the stones still close in on each other after the first frame.
    auto simulator = CreateEvent();
    AllStones stones;
    stones[15] = Moving(0.02f, 33.f, 2.5f, 0.f, 1.f);
    float const pitch = kDiameter - 0.0002f;
    int count = 0;
    for (int row = 0; row < 4; ++row) {
        for (int i = 0; i <= row; ++i)
            stones[count++] = Resting((static_cast<float>(i) - 0.5f * static_cast<float>(row)) * pitch, 35.f + static_cast<float>(row) * 0.866f * pitch);
    }
    simulator->SetStones(stones);
    int repeated_contacts = 0;
    while (!simulator->AreAllStonesStopped()) {
        simulator->Step();
        std::set<std::pair<int, int>> seen;
        for (auto const& c : simulator->GetCollisions())
            if (!seen.emplace(c.a.id, c.b.id).second) ++repeated_contacts;

        auto const& s = simulator->GetStones();
        for (int i = 0; i < 16; ++i) {
            for (int j = i + 1; j < 16; ++j) {
                if (!s[i] || !s[j] || Distance(s[i]->position, s[j]->position) > kDiameter) continue;
                auto const& a = *s[i];
                auto const& b = *s[j];
                float const opening = ((b.position.x - a.position.x) * (b.translational_velocity.x - a.translational_velocity.x)
                    + (b.position.y - a.position.y) * (b.translational_velocity.y - a.translational_velocity.y)) / kDiameter;
                EXPECT_GT(opening, -1e-3f) << i << " " << j;
            }
        }
    }
    EXPECT_GT(repeated_contacts, 0);
}

TEST(SimulatorFCV1Event, EveryContactIsReported)
{
    // Stone 0 pushes 1, which hits 2, which hits 3
    AllStones stones;
    stones[0] = Moving(0.f, 30.f, 2.5f, 0.f, 0.f);
    for (int i = 1; i < 4; ++i) stones[i] = Resting(0.f, 30.f + static_cast<float>(i));

    auto const pairs = [&stones](dcs::ISimulator & simulator) {
        std::vector<std::pair<int, int>> result;
        simulator.SetStones(stones);
        while (!simulator.AreAllStonesStopped()) {
            simulator.Step();
            for (auto const& c : simulator.GetCollisions()) {
                std::pair<int, int> const pair(std::min(c.a.id, c.b.id), std::max(c.a.id, c.b.id));
                if (c.normal_impulse > 0.f && (result.empty() || result.back() != pair)) result.push_back(pair);
            }
        }
        return result;
    };
    auto const expected = pairs(*CreateFCV1());
    EXPECT_GE(expected.size(), 3u);
    EXPECT_EQ(pairs(*CreateEvent()), expected);
}

// --- the loop that the plugin API runs ---

namespace {

void Simulate(dcs::ISimulator & simulator, DigitalCurling_SimulateModeFlag mode, float sheet_width)
{
    dc::plugins::detail::SimulatorSimulateLoopBody(&simulator, mode, 0, sheet_width);
}

} // unnamed namespace

TEST(SimulatorFCV1Event, CollisionModeStopsAtTheHit)
{
    auto fcv1 = CreateFCV1();
    auto event = CreateEvent();
    fcv1->SetStones(HeadOnHit());
    event->SetStones(HeadOnHit());
    Simulate(*fcv1, DIGITALCURLING_SIMULATE_MODE_COLLISION, 4.75f);
    Simulate(*event, DIGITALCURLING_SIMULATE_MODE_COLLISION, 4.75f);

    EXPECT_FALSE(event->GetCollisions().empty());
    EXPECT_FALSE(event->AreAllStonesStopped());
    EXPECT_LT(Difference(fcv1->GetStones(), event->GetStones()), 0.01f);
    EXPECT_NEAR(fcv1->GetStones()[1]->translational_velocity.y, event->GetStones()[1]->translational_velocity.y, 0.02f);
}

TEST(SimulatorFCV1Event, CollisionModeReachesTheEndWithStonesTouchingAtRest)
{
    // As with fcv1, a resting contact makes the loop return before any hit; calling it again goes on.
    AllStones stones = HeadOnHit();
    stones[2] = Resting(2.f, 20.f);
    stones[3] = Resting(2.f, 20.f + kDiameter - 0.0005f);
    auto event = CreateEvent();
    event->SetStones(stones);
    int calls = 0;
    bool hit = false;
    while (!event->AreAllStonesStopped() && calls < 100) {
        Simulate(*event, DIGITALCURLING_SIMULATE_MODE_COLLISION, 4.75f);
        ++calls;
        for (auto const& c : event->GetCollisions()) hit |= std::min(c.a.id, c.b.id) == 0;
    }
    EXPECT_LT(calls, 100);
    EXPECT_TRUE(hit);
}

TEST(SimulatorFCV1Event, OutStoneModeStopsAtTheLine)
{
    struct Case { StoneState stone; float line; bool is_x; } const cases[] = {
        { Moving(1.5f, 20.f, 2.f, -0.5f, 1.f), 4.75f / 2.f - dc::Stone::kRadius, true },
        { Moving(-1.5f, 20.f, 2.f, 0.5f, -1.f), -4.75f / 2.f + dc::Stone::kRadius, true },
        { Moving(0.f, 38.f, 2.5f, 0.f, 1.f), dc::coordinate::kBackBoardY - dc::Stone::kRadius, false },
        { Moving(0.f, 2.f, 2.5f, kPi, 1.f), 0.f, false },
    };
    for (auto const& c : cases) {
        // The stone has to leave in the frame in which it crosses the line.
        auto simulator = CreateEvent();
        AllStones stones;
        stones[0] = c.stone;
        simulator->SetStones(stones);
        auto const value = [&] { auto const& p = simulator->GetStones()[0]->position; return c.is_x ? p.x : p.y; };
        float const start = value();
        float const side = start < c.line ? 1.f : -1.f;
        while (!simulator->AreAllStonesStopped() && side * (value() - c.line) <= 0.f) simulator->Step();
        EXPECT_GT(side * (value() - c.line), 0.f);
        EXPECT_LT(side * (value() - c.line), 0.003f);

        simulator->SetStones(stones);
        Simulate(*simulator, DIGITALCURLING_SIMULATE_MODE_OUT_STONE, 4.75f);
        EXPECT_FALSE(simulator->GetStones()[0].has_value());
    }
}

TEST(SimulatorFCV1Event, FullModeMatchesFCV1NearTheEdges)
{
    Random random;
    auto fcv1 = CreateFCV1();
    auto event = CreateEvent();
    std::vector<float> differences;
    for (int k = 0; k < 60; ++k) {
        float const side = random.Sign();
        AllStones stones;
        stones[0] = Moving(side * random(0.f, 1.f), 34.f, random(1.f, 3.f), -side * random(0.f, 0.4f), random.Sign() * 1.5f);
        stones[1] = Resting(side * 1.7f, 38.f);
        stones[2] = Resting(side * random(2.f, 2.2f), random(38.5f, 39.5f));
        stones[3] = Resting(side * 2.1f, 43.f);
        for (auto * const simulator : { fcv1.get(), event.get() }) {
            simulator->SetStones(stones);
            Simulate(*simulator, DIGITALCURLING_SIMULATE_MODE_FULL, 4.75f);
        }
        differences.push_back(Difference(fcv1->GetStones(), event->GetStones()));
    }
    std::sort(differences.begin(), differences.end());
    EXPECT_LT(Median(differences), 0.005f);
    EXPECT_LT(differences[differences.size() * 9 / 10], 0.05f);
}

// --- CalculateShot ---

TEST(SimulatorFCV1Event, CalculateShotMatchesFCV1)
{
    dcs::SimulatorFCV1 const fcv1{ dcs::SimulatorFCV1Factory() };
    dcs::SimulatorFCV1Event const event{ dcs::SimulatorFCV1EventFactory() };
    int shots = 0;
    for (float const speed : { 0.f, 0.02f, 0.05f, 0.5f, 1.f, 2.f, 3.f }) {
        for (float const r : { 33.f, 38.405f, 43.f }) {
            for (float const x : { -1.f, 0.f, 1.f }) {
                for (float const spin : { 1.57f, -1.57f }) {
                    dc::Vector2 const target(x, std::sqrt(r * r - x * x));
                    auto const expected = fcv1.CalculateShot(target, speed, spin);
                    auto const actual = event.CalculateShot(target, speed, spin);
                    EXPECT_EQ(actual.translational_velocity, expected.translational_velocity);
                    EXPECT_EQ(actual.angular_velocity, expected.angular_velocity);
                    EXPECT_NEAR(actual.release_angle, expected.release_angle, 2e-5f) << speed << " " << r;
                    ++shots;
                }
            }
        }
    }
    EXPECT_EQ(shots, 126);
}

TEST(SimulatorFCV1Event, CalculateShotRejectsWhatItCannotDo)
{
    dcs::SimulatorFCV1Event const event{ dcs::SimulatorFCV1EventFactory() };
    float const nan = std::numeric_limits<float>::quiet_NaN();
    dc::Vector2 const tee(0.f, 38.405f);
    EXPECT_THROW(event.CalculateShot(tee, -1.f, 1.f), std::invalid_argument);
    EXPECT_THROW(event.CalculateShot(tee, 4.5f, 1.f), std::invalid_argument);
    EXPECT_THROW(event.CalculateShot(tee, nan, 1.f), std::invalid_argument);
    EXPECT_THROW(event.CalculateShot(dc::Vector2(0.f, 0.05f), 1.f, 1.f), std::invalid_argument);
    EXPECT_THROW(event.CalculateShot(dc::Vector2(nan, 38.f), 1.f, 1.f), std::invalid_argument);
    // Too close for the regression, for which fcv1 returns NaN
    EXPECT_THROW(event.CalculateShot(dc::Vector2(0.f, 20.f), 0.5f, 1.f), std::invalid_argument);
}

// --- save and load ---

namespace {

AllStones BusyStones()
{
    AllStones stones;
    stones[0] = Moving(0.02f, 32.f, 2.4f, 0.f, 1.5f);
    stones[2] = Resting(0.f, 36.f);
    stones[4] = Resting(0.05f, 36.f + kDiameter);
    stones[7] = Resting(0.3f, 36.8f);
    stones[9] = Moving(-1.f, 35.f, 0.8f, -0.7f, -1.f);
    stones[15] = Resting(-0.3f, 38.f);
    return stones;
}

} // unnamed namespace

TEST(SimulatorFCV1Event, SaveLoadContinuesExactly)
{
    // From every step of a busy shot, continuing, loading into a used simulator,
    // into a new one, and from JSON must all end bit-identically.
    for (float const max_step_seconds : { 0.f, 0.001f, 0.02f, 0.1f, 1.f }) {
        auto simulator = CreateEvent(max_step_seconds);
        auto other = CreateEvent(max_step_seconds);
        RunToStop(*other, BusyStones());  // leave something behind in it

        int const total_steps = RunToStop(*simulator, BusyStones());
        auto const expected = simulator->GetStones();

        simulator->SetStones(BusyStones());
        int const interval = std::max(1, total_steps / 40);
        for (int step = 0; !simulator->AreAllStonesStopped(); ++step) {
            if (step % interval == 0) {
                std::shared_ptr<dcs::ISimulatorStorage> const storage = simulator->CreateStorage();
                nlohmann::json const j = *std::dynamic_pointer_cast<dcs::SimulatorFCV1EventStorage>(storage);

                other->Load(*storage);
                auto created = storage->CreateSimulator();
                auto parsed = j.get<dcs::SimulatorFCV1EventStorage>().CreateSimulator();
                for (auto * const s : { other.get(), created.get(), parsed.get() }) {
                    while (!s->AreAllStonesStopped()) s->Step();
                    EXPECT_TRUE(dct::EqualsSimulatorStones(s->GetStones(), expected)) << max_step_seconds << " " << step;
                }
            }
            simulator->Step();
        }
        EXPECT_TRUE(dct::EqualsSimulatorStones(simulator->GetStones(), expected));
    }
}

TEST(SimulatorFCV1Event, SaveRestoresCollisions)
{
    auto simulator = CreateEvent();
    simulator->SetStones(HeadOnHit());
    while (simulator->GetCollisions().empty()) simulator->Step();
    auto const storage = simulator->CreateStorage();
    auto const count = simulator->GetCollisions().size();

    simulator->Step();
    EXPECT_TRUE(simulator->GetCollisions().empty());
    simulator->Load(*storage);
    EXPECT_EQ(simulator->GetCollisions().size(), count);
}
