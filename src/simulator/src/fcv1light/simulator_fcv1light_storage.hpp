// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

/// @file
/// @brief SimulatorFCV1LightStorage を定義

#pragma once

#include <memory>
#include <vector>
#include <nlohmann/json.hpp>
#include "digitalcurling/simulators/i_simulator_storage.hpp"
#include "simulator_fcv1light_factory.hpp"

namespace digitalcurling::simulators {

/// @brief シミュレータ FCV1Light のストレージ
class SimulatorFCV1LightStorage : public ISimulatorStorage {
public:
    /// @brief デフォルトコンストラクタ
    SimulatorFCV1LightStorage() = default;
    /// @brief コピーコンストラクタ
    SimulatorFCV1LightStorage(SimulatorFCV1LightStorage const&) = default;
    /// @brief コピー代入演算子
    SimulatorFCV1LightStorage & operator = (SimulatorFCV1LightStorage const&) = default;
    /// @brief コンストラクタ
    /// @param factory ストレージを初期化するためのファクトリー
    SimulatorFCV1LightStorage(SimulatorFCV1LightFactory const& factory);
    virtual ~SimulatorFCV1LightStorage() override = default;

    virtual const char* GetId() const noexcept override { return DIGITALCURLING_PLUGIN_NAME; }
    virtual nlohmann::json ToJson() const override;

    virtual std::unique_ptr<ISimulator> CreateSimulator() const override;

    /// @brief このストレージに保存されたシミュレータのファクトリー情報
    SimulatorFCV1LightFactory factory;
    /// @brief 全ストーンの位置と速度
    ISimulator::AllStones stones;
    /// @brief 衝突情報
    std::vector<ISimulator::Collision> collisions;
};


/// @cond Doxygen_Suppress
// json
void to_json(nlohmann::json &, SimulatorFCV1LightStorage const&);
void from_json(nlohmann::json const&, SimulatorFCV1LightStorage &);
/// @endcond

} // namespace digitalcurling::simulators
