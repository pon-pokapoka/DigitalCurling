// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

/// @file
/// @brief SimulatorFCV1Light を定義

#pragma once

#include <array>
#include <memory>
#include <vector>
#include "digitalcurling/moves/shot.hpp"
#include "digitalcurling/simulators/i_simulator.hpp"

#include "box2d_util.hpp"
#include "simulator_fcv1light_factory.hpp"
#include "simulator_fcv1light_storage.hpp"

namespace digitalcurling::simulators {


/// @brief Friction-CurlVelocity式シミュレータ Version1
class SimulatorFCV1Light : public ISimulator {
public:
    /// @brief ストーンの質量[kg]
    static constexpr float kStoneMass = 19.96f;

    /// @brief コンストラクタ
    /// @param factory このシミュレーターのファクトリー
    explicit SimulatorFCV1Light(SimulatorFCV1LightFactory const& factory);
    /// @brief コンストラクタ
    /// @param storage このシミュレーターのストレージ
    explicit SimulatorFCV1Light(SimulatorFCV1LightStorage const& storage);
    virtual ~SimulatorFCV1Light() = default;

    virtual const char* GetId() const noexcept override { return DIGITALCURLING_PLUGIN_NAME; }

    virtual ISimulator::AllStones const& GetStones() const override;
    virtual void SetStones(ISimulator::AllStones const& stones) override;

    void AdaptiveStep(float);

    virtual void Step() override;
    virtual std::vector<Collision> const& GetCollisions() const override;
    virtual bool AreAllStonesStopped() const override;
    virtual float GetSecondsPerFrame() const override;

    virtual ISimulatorFactory const& GetFactory() const override;

    virtual std::unique_ptr<ISimulatorStorage> CreateStorage() const override;
    virtual void Save(ISimulatorStorage & storage) const override;
    virtual void Load(ISimulatorStorage const& storage) override;

    /// @brief 指定地点を指定速度で通過するショットを逆算(推測)する
    /// @param target_position 目標地点
    /// @param target_speed 目標地点到達時の速度
    /// @param shot_angular_velocity ショットの回転速度
    /// @return 推測されたショット
    /// @note - 関数内でシミュレータFCV1Lightを使用して1ショット分シミュレーションを行っています。ですので、この関数の実行は高速とは言えません。
    /// @note - この関数は解析的にもとめたものでなく、シミュレーション結果から回帰分析で求めた関数です。したがって、特に飛距離にはある程度誤差が存在します。
    virtual moves::Shot CalculateShot(Vector2 const& target_position, float const target_speed, float const shot_angular_velocity) const;

private:
    class ContactListener : public b2ContactListener {
    public:
        ContactListener(SimulatorFCV1Light * instance) : instance_(instance) {}
        virtual void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) override;
    private:
        SimulatorFCV1Light * const instance_;
    };

    mutable SimulatorFCV1LightStorage storage_;
    b2World world_;
    std::array<b2Body*, StoneCoordinate::kStoneMax> stone_bodies_;
    mutable bool stones_dirty_;
    mutable bool all_stones_stopped_;
    mutable bool all_stones_stopped_dirty_;
    ContactListener contact_listener_;

    SimulatorFCV1LightStorage pre_step_storage;

    // ストレージのデータを内部データに適用する
    void UpdateWithStorage();
};

} // namespace digitalcurling::simulators
