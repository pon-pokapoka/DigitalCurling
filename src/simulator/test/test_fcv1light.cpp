#include <memory>
#include <string>
#include <nlohmann/json.hpp>
#include "common.hpp"
#include "../src/fcv1light/simulator_fcv1light_factory.hpp"
#include "../src/fcv1light/simulator_fcv1light_storage.hpp"

namespace dct = digitalcurling::test;

TEST(SimulatorFCV1Light, InitialState)
{
    dcs::SimulatorFCV1LightFactory factory;
    auto simulator = factory.CreateSimulator();
    EXPECT_TRUE(simulator->AreAllStonesStopped());
}

TEST(SimulatorFCV1Light, SaveLoad1)
{
    dcs::SimulatorFCV1LightFactory factory;
    auto simulator = factory.CreateSimulator();

    {
        dcs::ISimulator::AllStones init_stones;
        init_stones[0] = dcs::ISimulator::StoneState(dc::Vector2(0.1f, 2.f), 0.5f, dc::Vector2(-0.1f, -2.f), 1.f);
        init_stones[15] = dcs::ISimulator::StoneState(dc::Vector2(0.1f, -4.f), 0.5f, dc::Vector2(-0.1f, -2.f), 1.f);
        simulator->SetStones(init_stones);
    }

    // 1
    auto storage = simulator->CreateStorage();  // create & save
    for (int i = 0; i < 10; ++i) { simulator->Step(); }
    auto const stones1 = simulator->GetStones();

    // 2
    simulator->Load(*storage);
    for (int i = 0; i < 10; ++i) { simulator->Step(); }
    auto const stones2 = simulator->GetStones();

    EXPECT_TRUE(dct::EqualsSimulatorStones(stones1, stones2));
}

TEST(SimulatorFCV1Light, SaveLoad2)
{
    dcs::SimulatorFCV1LightFactory factory;
    auto simulator = factory.CreateSimulator();
    auto storage = simulator->CreateStorage();  // create & save

    {
        dcs::ISimulator::AllStones init_stones;
        init_stones[0] = dcs::ISimulator::StoneState(dc::Vector2(0.1f, 2.f), 0.5f, dc::Vector2(-0.1f, -2.f), 1.f);
        init_stones[15] = dcs::ISimulator::StoneState(dc::Vector2(0.1f, -4.f), 0.5f, dc::Vector2(-0.1f, -2.f), 1.f);
        simulator->SetStones(init_stones);
    }

    // 1
    simulator->Save(*storage);
    for (int i = 0; i < 10; ++i) { simulator->Step(); }
    auto const stones1 = simulator->GetStones();

    // 2
    simulator->Load(*storage);
    for (int i = 0; i < 10; ++i) { simulator->Step(); }
    auto const stones2 = simulator->GetStones();

    EXPECT_TRUE(dct::EqualsSimulatorStones(stones1, stones2));
}

TEST(SimulatorFCV1Light, SaveLoad3)
{
    dcs::SimulatorFCV1LightFactory factory;
    auto simulator = factory.CreateSimulator();

    {
        dcs::ISimulator::AllStones init_stones;
        init_stones[0] = dcs::ISimulator::StoneState(dc::Vector2(0.1f, 2.f), 0.5f, dc::Vector2(-0.1f, -2.f), 1.f);
        init_stones[15] = dcs::ISimulator::StoneState(dc::Vector2(0.1f, -4.f), 0.5f, dc::Vector2(-0.1f, -2.f), 1.f);
        simulator->SetStones(init_stones);
    }

    // 1
    auto storage = simulator->CreateStorage();  // create & save
    for (int i = 0; i < 10; ++i) { simulator->Step(); }
    auto const stones1 = simulator->GetStones();

    // 2
    auto simulator_copy = storage->CreateSimulator();
    for (int i = 0; i < 10; ++i) { simulator_copy->Step(); }
    auto const stones2 = simulator_copy->GetStones();

    EXPECT_TRUE(dct::EqualsSimulatorStones(stones1, stones2));
}

TEST(SimulatorFCV1Light, SaveLoad4)
{
    dcs::SimulatorFCV1LightFactory factory;
    auto simulator = factory.CreateSimulator();

    {
        dcs::ISimulator::AllStones init_stones;
        init_stones[0] = dcs::ISimulator::StoneState(dc::Vector2(0.1f, 2.f), 0.5f, dc::Vector2(-0.1f, -2.f), 1.f);
        init_stones[15] = dcs::ISimulator::StoneState(dc::Vector2(0.1f, -4.f), 0.5f, dc::Vector2(-0.1f, -2.f), 1.f);
        simulator->SetStones(init_stones);
    }

    // 1
    std::shared_ptr<dcs::ISimulatorStorage> storage = simulator->CreateStorage();
    nlohmann::json j_storage = *std::dynamic_pointer_cast<dcs::SimulatorFCV1LightStorage>(storage);
    for (int i = 0; i < 10; ++i) { simulator->Step(); }
    auto const stones1 = simulator->GetStones();

    // 2
    auto storage2 = j_storage.get<dcs::SimulatorFCV1LightStorage>();
    auto simulator_copy = storage2.CreateSimulator();
    for (int i = 0; i < 10; ++i) { simulator_copy->Step(); }
    auto const stones2 = simulator_copy->GetStones();

    EXPECT_TRUE(dct::EqualsSimulatorStones(stones1, stones2));
}


TEST(SimulatorFCV1Light, FactoryToJson)
{
    auto v_fcv1light = std::make_unique<dcs::SimulatorFCV1LightFactory>();
    v_fcv1light->seconds_per_frame = 0.24f;

    nlohmann::json const j_fcv1light = *v_fcv1light.get();
    EXPECT_EQ(j_fcv1light.at("type").get<std::string>(), "fcv1light");
    EXPECT_EQ(j_fcv1light.at("seconds_per_frame").get<float>(), v_fcv1light->seconds_per_frame);
}

TEST(SimulatorFCV1Light, FactoryFromJson)
{
    nlohmann::json const j_fcv1light = {
        { "type", "fcv1light" },
        { "seconds_per_frame", 0.25f }
    };

    dcs::SimulatorFCV1LightFactory v_fcv1light;
    EXPECT_NO_THROW(v_fcv1light = j_fcv1light.get<dcs::SimulatorFCV1LightFactory>());
    EXPECT_EQ(v_fcv1light.seconds_per_frame, 0.25f);
}
