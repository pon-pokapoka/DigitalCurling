// Copyright (c) 2022-2026 UEC Takeshi Ito Laboratory
// SPDX-License-Identifier: MIT

/// @file
/// @brief SimulatorFCV1LightFactory を定義

#pragma once

#include <memory>
#include <string_view>
#include <nlohmann/json.hpp>
#include "digitalcurling/simulators/i_simulator.hpp"
#include "digitalcurling/simulators/i_simulator_factory.hpp"
#include "fcv1light.hpp"

namespace digitalcurling::simulators {

/// @brief シミュレータ FCV1Light のファクトリー
class SimulatorFCV1LightFactory : public ISimulatorFactory {
public:
    /// @brief フレームレート(フレーム毎秒)
    ///
    /// CreateSimulator() を呼び出す前にこの値を変更することで、
    /// 生成されるFCV1Lightシミュレータのフレームレート( `Step()` の呼出しによる経過時間)を変更することができます。
    /// ただし，フレームレートをデフォルトの値から変更した際の動作の保証はしません。
    float seconds_per_frame = 0.001f;

    /// @brief デフォルトコンストラクタ
    SimulatorFCV1LightFactory() = default;
    /// @brief コピーコンストラクタ
    SimulatorFCV1LightFactory(SimulatorFCV1LightFactory const&) = default;
    /// @brief コピー代入演算子
    SimulatorFCV1LightFactory & operator = (SimulatorFCV1LightFactory const&) = default;
    virtual ~SimulatorFCV1LightFactory() override = default;

    virtual const char* GetId() const noexcept override { return DIGITALCURLING_PLUGIN_NAME; }
    virtual nlohmann::json ToJson() const override;

    virtual std::unique_ptr<ISimulator> CreateSimulator() const override;
    virtual std::unique_ptr<ISimulatorFactory> Clone() const override;
};


/// @cond Doxygen_Suppress
// json
void to_json(nlohmann::json &, SimulatorFCV1LightFactory const&);
void from_json(nlohmann::json const&, SimulatorFCV1LightFactory &);
/// @endcond

} // namespace digitalcurling::simulators
