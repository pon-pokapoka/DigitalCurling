#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
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
#include "../src/fcv1table/simulator_fcv1table.hpp"
#include "../src/fcv1table/simulator_fcv1table_factory.hpp"
#include "../src/fcv1table/simulator_fcv1table_storage.hpp"
#include "digitalcurling/plugins/simulator_plugin_export.hpp"

namespace dct = digitalcurling::test;

namespace {

using AllStones = dcs::ISimulator::AllStones;
using StoneState = dcs::ISimulator::StoneState;

constexpr float kDiameter = 2.f * dc::Stone::kRadius;
constexpr float kPi = 3.14159265f;
constexpr int kMaxSteps = 200000;

std::unique_ptr<dcs::ISimulator> CreateTable(float step_seconds = 1.f)
{
    dcs::SimulatorFCV1TableFactory factory;
    factory.step_seconds = step_seconds;
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

// Differences of where the stones stop, sorted
std::vector<float> CompareWithFCV1(std::vector<AllStones> const& shots, float step_seconds = 1.f)
{
    auto fcv1 = CreateFCV1();
    auto table = CreateTable(step_seconds);
    std::vector<float> result;
    for (auto const& shot : shots) {
        RunToStop(*fcv1, shot);
        RunToStop(*table, shot);
        result.push_back(Difference(fcv1->GetStones(), table->GetStones()));
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


TEST(SimulatorFCV1Table, InitialState)
{
    auto simulator = CreateTable();
    EXPECT_TRUE(simulator->AreAllStonesStopped());
    simulator->Step();
    EXPECT_TRUE(simulator->AreAllStonesStopped());
    EXPECT_TRUE(simulator->GetCollisions().empty());
}

// --- settings ---

TEST(SimulatorFCV1Table, FactoryToJson)
{
    dcs::SimulatorFCV1TableFactory factory;
    factory.seconds_per_frame = 0.24f;
    factory.step_seconds = 0.5f;
    factory.sheet_width = 5.f;

    nlohmann::json const j = factory;
    EXPECT_EQ(j.at("type").get<std::string>(), "fcv1table");
    EXPECT_EQ(j.at("seconds_per_frame").get<float>(), 0.24f);
    EXPECT_EQ(j.at("step_seconds").get<float>(), 0.5f);
    EXPECT_EQ(j.at("sheet_width").get<float>(), 5.f);
}

TEST(SimulatorFCV1Table, FactoryFromJson)
{
    nlohmann::json const j = {
        { "type", "fcv1table" }, { "seconds_per_frame", 0.25f }, { "step_seconds", 0.1f }, { "sheet_width", 0.f }
    };
    auto const factory = j.get<dcs::SimulatorFCV1TableFactory>();
    EXPECT_EQ(factory.seconds_per_frame, 0.25f);
    EXPECT_EQ(factory.step_seconds, 0.1f);
    EXPECT_EQ(factory.sheet_width, 0.f);

    // A missing setting has its default, also when the target held another value.
    nlohmann::json const j_default = { { "type", "fcv1table" }, { "seconds_per_frame", 0.001f } };
    dcs::SimulatorFCV1TableFactory target;
    target.step_seconds = 3.f;
    target.sheet_width = 3.f;
    j_default.get_to(target);
    EXPECT_EQ(target.step_seconds, 1.f);
    EXPECT_EQ(target.sheet_width, 4.75f);
}

TEST(SimulatorFCV1Table, InvalidSettings)
{
    float const nan = std::numeric_limits<float>::quiet_NaN();
    float const inf = std::numeric_limits<float>::infinity();
    for (float const step_seconds : { 0.f, -1.f, 0.0005f, 10.5f, nan, inf }) {
        dcs::SimulatorFCV1TableFactory factory;
        factory.step_seconds = step_seconds;
        EXPECT_THROW(factory.CreateSimulator(), std::invalid_argument) << step_seconds;
    }
    for (float const seconds_per_frame : { 0.f, -0.001f, nan, inf }) {
        dcs::SimulatorFCV1TableFactory factory;
        factory.seconds_per_frame = seconds_per_frame;
        EXPECT_THROW(factory.CreateSimulator(), std::invalid_argument) << seconds_per_frame;
    }
    for (float const sheet_width : { -1.f, nan, inf }) {
        dcs::SimulatorFCV1TableFactory factory;
        factory.sheet_width = sheet_width;
        EXPECT_THROW(factory.CreateSimulator(), std::invalid_argument) << sheet_width;
    }
    nlohmann::json const j = { { "type", "fcv1table" }, { "seconds_per_frame", 0.001f }, { "step_seconds", 0.f } };
    EXPECT_THROW(j.get<dcs::SimulatorFCV1TableFactory>(), std::invalid_argument);
}

TEST(SimulatorFCV1Table, StepLength)
{
    AllStones stones;
    stones[0] = Moving(0.f, 0.f, 2.4f, 0.f, 1.5f);
    auto fcv1 = CreateFCV1();

    for (float const step_seconds : { 0.001f, 0.01f, 0.1f, 1.f, 10.f }) {
        auto table = CreateTable(step_seconds);
        table->SetStones(stones);
        table->Step();

        fcv1->SetStones(stones);
        int const frames = static_cast<int>(std::lround(step_seconds * 1000.f));
        for (int i = 0; i < frames; ++i) fcv1->Step();

        auto const& expected = *fcv1->GetStones()[0];
        auto const& actual = *table->GetStones()[0];
        EXPECT_LT((expected.position - actual.position).Length(), 0.002f) << step_seconds;
        EXPECT_NEAR(expected.translational_velocity.Length(), actual.translational_velocity.Length(), 1e-3f) << step_seconds;
        EXPECT_NEAR(expected.angular_velocity, actual.angular_velocity, 1e-3f) << step_seconds;
        EXPECT_NEAR(expected.angle, actual.angle, 1e-2f) << step_seconds;
    }
}

TEST(SimulatorFCV1Table, OtherFrameLengthFollowsFCV1)
{
    // Without its 1 ms frames the table doesn't apply, and the simulator runs plain frames.
    AllStones stones;
    stones[0] = Moving(0.f, 30.f, 2.f, 0.f, 1.5f);
    stones[1] = Resting(0.05f, 33.f);

    dcs::SimulatorFCV1Factory fcv1_factory;
    fcv1_factory.seconds_per_frame = 0.002f;
    auto fcv1 = fcv1_factory.CreateSimulator();
    dcs::SimulatorFCV1TableFactory table_factory;
    table_factory.seconds_per_frame = 0.002f;
    auto table = table_factory.CreateSimulator();
    EXPECT_EQ(table->GetSecondsPerFrame(), 0.002f);

    fcv1->SetStones(stones);
    table->SetStones(stones);
    for (int i = 0; i < 100; ++i) {
        fcv1->Step();
        table->Step();
    }
    EXPECT_LT(Difference(fcv1->GetStones(), table->GetStones()), 1e-4f);

    RunToStop(*fcv1, stones);
    RunToStop(*table, stones);
    EXPECT_LT(Difference(fcv1->GetStones(), table->GetStones()), 0.05f);
}

// --- invalid and extreme states ---

TEST(SimulatorFCV1Table, NonFiniteStonesAreRejected)
{
    AllStones good;
    good[3] = Moving(0.f, 10.f, 2.f, 0.f, 1.f);
    auto simulator = CreateTable();
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

            dcs::SimulatorFCV1TableStorage storage;
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

TEST(SimulatorFCV1Table, HugeSpinDecays)
{
    AllStones stones;
    stones[0] = StoneState(dc::Vector2(0.f, 10.f), 0.f, dc::Vector2(), -1e30f);
    auto simulator = CreateTable(10.f);
    EXPECT_LT(RunToStop(*simulator, stones), 20);
    EXPECT_EQ(simulator->GetStones()[0]->angular_velocity, 0.f);
}

TEST(SimulatorFCV1Table, OverlappingStonesTerminate)
{
    AllStones stones;
    stones[0] = Moving(0.f, 30.f, 1.f, 0.3f, 1.f);
    stones[1] = Moving(0.f, 30.f, 1.f, 0.3f, 1.f);
    stones[2] = Resting(0.1f, 30.1f);
    auto simulator = CreateTable();
    RunToStop(*simulator, stones);
    for (int i = 0; i < 3; ++i) EXPECT_TRUE(std::isfinite(simulator->GetStones()[i]->position.Length()));
}

TEST(SimulatorFCV1Table, SpeedAboveTheTable)
{
    AllStones stones;
    stones[0] = Moving(0.f, 0.f, 6.5f, 0.f, 1.f);
    stones[1] = Resting(0.05f, 8.f);
    auto fcv1 = CreateFCV1();
    auto table = CreateTable();
    fcv1->SetStones(stones);
    for (int i = 0; i < 1000; ++i) fcv1->Step();
    table->SetStones(stones);
    table->Step();
    EXPECT_LT(Difference(fcv1->GetStones(), table->GetStones()), 1e-3f);

    // Slows down into the table's range and goes on with it.
    stones[1].reset();
    auto const differences = CompareWithFCV1({ stones });
    EXPECT_LT(differences[0], 0.15f);  // after about 280 m
}

// --- stopping ---

TEST(SimulatorFCV1Table, NegativeSpinIsNotStopped)
{
    auto simulator = CreateTable();
    for (float const spin : { 1.f, -1.f }) {
        AllStones stones;
        stones[0] = StoneState(dc::Vector2(0.f, 10.f), 0.f, dc::Vector2(), spin);
        simulator->SetStones(stones);
        EXPECT_FALSE(simulator->AreAllStonesStopped());
        RunToStop(*simulator, stones);
        EXPECT_EQ(simulator->GetStones()[0]->angular_velocity, 0.f);
    }
}

TEST(SimulatorFCV1Table, NegativeSpinMirrorsPositiveSpin)
{
    auto simulator = CreateTable();
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

TEST(SimulatorFCV1Table, StoppedStonesAreAtRest)
{
    Random random;
    auto simulator = CreateTable();
    for (int k = 0; k < 30; ++k) {
        AllStones stones;
        stones[0] = Moving(0.f, 20.f, random(0.1f, 2.f), random(-kPi, kPi), random(-5.f, 5.f));
        RunToStop(*simulator, stones);
        EXPECT_EQ(simulator->GetStones()[0]->angular_velocity, 0.f);
        EXPECT_LT(simulator->GetStones()[0]->translational_velocity.Length(), 4e-4f);
    }
}

TEST(SimulatorFCV1Table, SpinRunsOut)
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

TEST(SimulatorFCV1Table, Draws)
{
    Random random;
    std::vector<AllStones> shots;
    for (int k = 0; k < 40; ++k) {
        AllStones stones;
        stones[0] = Moving(0.f, 0.f, random(1.8f, 2.5f), random(-0.06f, 0.06f), random.Sign() * random(0.2f, 2.f));
        shots.push_back(stones);
    }
    for (float const step_seconds : { 0.1f, 1.f }) {
        auto const differences = CompareWithFCV1(shots, step_seconds);
        EXPECT_LT(Median(differences), 0.004f) << step_seconds;
        EXPECT_LT(differences.back(), 0.015f) << step_seconds;
    }
}

TEST(SimulatorFCV1Table, Hits)
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

TEST(SimulatorFCV1Table, TwoMovingStones)
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

TEST(SimulatorFCV1Table, ManyMovingStones)
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

TEST(SimulatorFCV1Table, FrozenStones)
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

TEST(SimulatorFCV1Table, NearMissesAreNotCollisions)
{
    auto simulator = CreateTable();
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
    auto simulator = CreateTable();
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

TEST(SimulatorFCV1Table, RestingContactDoesNotChangeOtherStones)
{
    float const touching = kDiameter - 0.0005f;
    auto const apart = SlideNextToRestingPair(1.f);
    auto const near = SlideNextToRestingPair(touching);
    EXPECT_FLOAT_EQ(near.x, apart.x);
    EXPECT_FLOAT_EQ(near.y, apart.y);
}

TEST(SimulatorFCV1Table, RestingContactIsStillReported)
{
    std::vector<dcs::ISimulator::Collision> collisions;
    SlideNextToRestingPair(kDiameter - 0.0005f, &collisions);
    ASSERT_FALSE(collisions.empty());
    for (auto const& c : collisions) {
        EXPECT_EQ(std::min(c.a.id, c.b.id), 1);
        EXPECT_EQ(std::max(c.a.id, c.b.id), 2);
    }
}

TEST(SimulatorFCV1Table, HitStillMovesTheOtherStone)
{
    auto simulator = CreateTable();
    AllStones stones;
    stones[0] = StoneState(dc::Vector2(0.f, 0.f), 0.f, dc::Vector2(0.f, 2.f), 0.f);
    stones[1] = Resting(0.f, 1.f);
    RunToStop(*simulator, stones);

    auto const& result = simulator->GetStones();
    EXPECT_GT(result[1]->position.y, 1.5f);
    EXPECT_LT(result[0]->position.y, result[1]->position.y);
}

TEST(SimulatorFCV1Table, StepEndsWhenAContactIsOver)
{
    auto simulator = CreateTable(10.f);
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

TEST(SimulatorFCV1Table, EveryContactIsReported)
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
    EXPECT_EQ(pairs(*CreateTable()), expected);
}

// --- the loop that the plugin API runs ---

namespace {

void Simulate(dcs::ISimulator & simulator, DigitalCurling_SimulateModeFlag mode, float sheet_width)
{
    dc::plugins::detail::SimulatorSimulateLoopBody(&simulator, mode, 0, sheet_width);
}

} // unnamed namespace

TEST(SimulatorFCV1Table, CollisionModeStopsAtTheHit)
{
    auto fcv1 = CreateFCV1();
    auto table = CreateTable();
    fcv1->SetStones(HeadOnHit());
    table->SetStones(HeadOnHit());
    Simulate(*fcv1, DIGITALCURLING_SIMULATE_MODE_COLLISION, 4.75f);
    Simulate(*table, DIGITALCURLING_SIMULATE_MODE_COLLISION, 4.75f);

    EXPECT_FALSE(table->GetCollisions().empty());
    EXPECT_FALSE(table->AreAllStonesStopped());
    EXPECT_LT(Difference(fcv1->GetStones(), table->GetStones()), 0.01f);
    EXPECT_NEAR(fcv1->GetStones()[1]->translational_velocity.y, table->GetStones()[1]->translational_velocity.y, 0.02f);
}

TEST(SimulatorFCV1Table, CollisionModeReachesTheEndWithStonesTouchingAtRest)
{
    // As with fcv1, a resting contact makes the loop return before any hit; calling it again goes on.
    AllStones stones = HeadOnHit();
    stones[2] = Resting(2.f, 20.f);
    stones[3] = Resting(2.f, 20.f + kDiameter - 0.0005f);
    auto table = CreateTable();
    table->SetStones(stones);
    int calls = 0;
    bool hit = false;
    while (!table->AreAllStonesStopped() && calls < 100) {
        Simulate(*table, DIGITALCURLING_SIMULATE_MODE_COLLISION, 4.75f);
        ++calls;
        for (auto const& c : table->GetCollisions()) hit |= std::min(c.a.id, c.b.id) == 0;
    }
    EXPECT_LT(calls, 100);
    EXPECT_TRUE(hit);
}

TEST(SimulatorFCV1Table, OutStoneModeStopsAtTheLine)
{
    struct Case { StoneState stone; float line; bool is_x; } const cases[] = {
        { Moving(1.5f, 20.f, 2.f, -0.5f, 1.f), 4.75f / 2.f - dc::Stone::kRadius, true },
        { Moving(-1.5f, 20.f, 2.f, 0.5f, -1.f), -4.75f / 2.f + dc::Stone::kRadius, true },
        { Moving(0.f, 38.f, 2.5f, 0.f, 1.f), dc::coordinate::kBackBoardY - dc::Stone::kRadius, false },
        { Moving(0.f, 2.f, 2.5f, kPi, 1.f), 0.f, false },
    };
    for (auto const& c : cases) {
        // The stone has to leave in the frame in which it crosses the line.
        auto simulator = CreateTable();
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

TEST(SimulatorFCV1Table, FullModeMatchesFCV1NearTheEdges)
{
    Random random;
    auto fcv1 = CreateFCV1();
    auto table = CreateTable();
    std::vector<float> differences;
    for (int k = 0; k < 60; ++k) {
        float const side = random.Sign();
        AllStones stones;
        stones[0] = Moving(side * random(0.f, 1.f), 34.f, random(1.f, 3.f), -side * random(0.f, 0.4f), random.Sign() * 1.5f);
        stones[1] = Resting(side * 1.7f, 38.f);
        stones[2] = Resting(side * random(2.f, 2.2f), random(38.5f, 39.5f));
        stones[3] = Resting(side * 2.1f, 43.f);
        for (auto * const simulator : { fcv1.get(), table.get() }) {
            simulator->SetStones(stones);
            Simulate(*simulator, DIGITALCURLING_SIMULATE_MODE_FULL, 4.75f);
        }
        differences.push_back(Difference(fcv1->GetStones(), table->GetStones()));
    }
    std::sort(differences.begin(), differences.end());
    EXPECT_LT(Median(differences), 0.005f);
    EXPECT_LT(differences[differences.size() * 9 / 10], 0.05f);
}

// --- CalculateShot ---

TEST(SimulatorFCV1Table, CalculateShotMatchesFCV1)
{
    dcs::SimulatorFCV1 const fcv1{ dcs::SimulatorFCV1Factory() };
    dcs::SimulatorFCV1Table const table{ dcs::SimulatorFCV1TableFactory() };
    int shots = 0;
    for (float const speed : { 0.f, 0.02f, 0.05f, 0.5f, 1.f, 2.f, 3.f }) {
        for (float const r : { 33.f, 38.405f, 43.f }) {
            for (float const x : { -1.f, 0.f, 1.f }) {
                for (float const spin : { 1.57f, -1.57f }) {
                    dc::Vector2 const target(x, std::sqrt(r * r - x * x));
                    auto const expected = fcv1.CalculateShot(target, speed, spin);
                    auto const actual = table.CalculateShot(target, speed, spin);
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

TEST(SimulatorFCV1Table, CalculateShotRejectsWhatItCannotDo)
{
    dcs::SimulatorFCV1Table const table{ dcs::SimulatorFCV1TableFactory() };
    float const nan = std::numeric_limits<float>::quiet_NaN();
    dc::Vector2 const tee(0.f, 38.405f);
    EXPECT_THROW(table.CalculateShot(tee, -1.f, 1.f), std::invalid_argument);
    EXPECT_THROW(table.CalculateShot(tee, 4.5f, 1.f), std::invalid_argument);
    EXPECT_THROW(table.CalculateShot(tee, nan, 1.f), std::invalid_argument);
    EXPECT_THROW(table.CalculateShot(dc::Vector2(0.f, 0.05f), 1.f, 1.f), std::invalid_argument);
    EXPECT_THROW(table.CalculateShot(dc::Vector2(nan, 38.f), 1.f, 1.f), std::invalid_argument);
    // Too close for the regression, for which fcv1 returns NaN
    EXPECT_THROW(table.CalculateShot(dc::Vector2(0.f, 20.f), 0.5f, 1.f), std::invalid_argument);
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

TEST(SimulatorFCV1Table, SaveLoadContinuesExactly)
{
    // From every step of a busy shot, continuing, loading into a used simulator,
    // into a new one, and from JSON must all end bit-identically.
    for (float const step_seconds : { 0.001f, 0.02f, 0.1f, 1.f }) {
        auto simulator = CreateTable(step_seconds);
        auto other = CreateTable(step_seconds);
        RunToStop(*other, BusyStones());  // leave something behind in it

        int const total_steps = RunToStop(*simulator, BusyStones());
        auto const expected = simulator->GetStones();

        simulator->SetStones(BusyStones());
        int const interval = std::max(1, total_steps / 40);
        for (int step = 0; !simulator->AreAllStonesStopped(); ++step) {
            if (step % interval == 0) {
                std::shared_ptr<dcs::ISimulatorStorage> const storage = simulator->CreateStorage();
                nlohmann::json const j = *std::dynamic_pointer_cast<dcs::SimulatorFCV1TableStorage>(storage);

                other->Load(*storage);
                auto created = storage->CreateSimulator();
                auto parsed = j.get<dcs::SimulatorFCV1TableStorage>().CreateSimulator();
                for (auto * const s : { other.get(), created.get(), parsed.get() }) {
                    while (!s->AreAllStonesStopped()) s->Step();
                    EXPECT_TRUE(dct::EqualsSimulatorStones(s->GetStones(), expected)) << step_seconds << " " << step;
                }
            }
            simulator->Step();
        }
        EXPECT_TRUE(dct::EqualsSimulatorStones(simulator->GetStones(), expected));
    }
}

TEST(SimulatorFCV1Table, SaveRestoresCollisions)
{
    auto simulator = CreateTable();
    simulator->SetStones(HeadOnHit());
    while (simulator->GetCollisions().empty()) simulator->Step();
    auto const storage = simulator->CreateStorage();
    auto const count = simulator->GetCollisions().size();

    simulator->Step();
    EXPECT_TRUE(simulator->GetCollisions().empty());
    simulator->Load(*storage);
    EXPECT_EQ(simulator->GetCollisions().size(), count);
}
