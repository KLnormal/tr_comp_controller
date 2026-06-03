//
// Created by 15082 on 2026/5/26.
//

#include "planner/tracking_differ.h"

#include <cmath>
#include <math/math_macro.h>
#include <arm_math.h>

using namespace planner;

/**
 * @param omega 自然频率，值越大响应越快；但离散实现中 omega * dt 过大可能导致振荡
 * @param zeta 阻尼比，值越大超调越小，但响应会变慢，通常取 1
 * @param frc 更新频率，单位 Hz
 */
tracking_differ::tracking_differ(float omega, float zeta, float frc)
    : omega_(omega),
      zeta_(zeta),
      delta_time_(frc > 0.0f ? 1.0f / frc : 0.0f) {
}

/**
 * @brief 清空状态，将目标值和跟踪值都重置为 0
 */
void tracking_differ::clear() {
    target_ = 0.0f;
    track_ = {0.0f, 0.0f};
}

/**
 * @brief 清空状态，并将当前跟踪值直接设置为指定目标值
 * @param target 重置后的目标值
 */
void tracking_differ::clear(float target) {
    target_ = target;
    track_ = {target, 0.0f};
}

/**
 * @brief 设置跟踪微分器参数
 * @param omega 自然频率
 * @param zeta 阻尼比
 * @param frc 更新频率，单位 Hz
 */
void tracking_differ::set_para(float omega, float zeta, float frc) {
    omega_ = omega;
    zeta_ = zeta;
    delta_time_ = frc > 0.0f ? 1.0f / frc : 0.0f;
}

/**
 * @brief 更新跟踪微分器
 * @param target 需要跟踪的目标值，通常为目标位置
 * @return first 为平滑后的目标值，second 为目标值的一阶导数
 */
std::array <float, 2> tracking_differ::update(float target) {
    target_ = target;

    // 二阶跟踪系统：
    // x'' = omega^2 * (target - x) - 2 * zeta * omega * x'
    const float ddot_tracking =
        omega_ * omega_ * (target_ - track_[0])
        - 2.0f * zeta_ * omega_ * track_[1];

    // 半隐式 Euler 积分
    track_[1] += ddot_tracking * delta_time_;
    track_[0]  += track_[1] * delta_time_;

    return track_;
}

/**
 * @brief 更新带角度跳变处理的跟踪微分器
 * @param target 输入目标角度，范围通常为 [-PI, PI]
 * @return first 为平滑后的角度，范围被限制在 [-PI, PI]；second 为角速度
 *
 * @note 该函数用于处理角度从 PI 跳变到 -PI，或从 -PI 跳变到 PI 的情况
 * @note 目前仅支持关于 0 对称的 [-PI, PI] 角度区间
 */
std::array <float, 2> tracking_differ::update_limit(float target) {
    // 计算当前输入角度和上一次展开角度之间的最短角度差
    float delta = std::fmod(target - target_, 2.0f * PI);

    // 如果角度差超过 PI，说明跨过了跳变边界，需要选择另一侧的最短路径
    if (std::fabs(delta) > PI) {
        delta -= (delta > 0.0f ? 1.0f : -1.0f) * 2.0f * PI;
    }

    // 将输入角度展开成连续角度，避免 PI 和 -PI 附近的突变
    target_ += delta;

    return update_mod(target_);
}

/**
 * @brief 更新跟踪微分器，并将输出位置限制到 [-PI, PI]
 * @param target 已经展开后的连续目标角度
 * @return first 为限制后的角度，second 为角速度
 *
 * @note 该函数不处理输入目标的跳变，只负责限制输出角度范围
 */
std::array <float, 2> tracking_differ::update_mod(float target) {
    auto t = update(target);

    // 只限制角度输出，不限制角速度
    t[0] = wrap_pi(t[0]);

    return t;
}

/**
 * @brief 将角度限制到 [-PI, PI]
 * @param x 输入角度
 * @return 归一化后的角度
 */
float tracking_differ::wrap_pi(float x) {
    x = std::fmod(x + PI, 2.0f * PI);

    if (x < 0.0f) {
        x += 2.0f * PI;
    }

    return x - PI;
}