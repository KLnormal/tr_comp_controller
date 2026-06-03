//
// Created by 15082 on 2026/5/26.
//

/*!
 *  @brief 跟踪微分器，用于平滑跟踪目标值，并估计目标值的一阶导数
 *  @author KL_normal
 *  @date 2026-05-27
 */

#ifndef TRACKING_DIFFER_H
#define TRACKING_DIFFER_H

#include <array>
#include <arm_math_types.h>

namespace planner {

    class tracking_differ {
    public:
        tracking_differ() = default;

        /**
         * @param omega 自然频率，值越大响应越快；但离散实现中 omega * dt 过大可能导致振荡
         * @param zeta 阻尼比，值越大超调越小，但响应会变慢，通常取 1
         * @param frc 更新频率，单位 Hz
         */
        tracking_differ(float omega, float zeta, float frc);

        /**
         * @brief 更新跟踪微分器
         * @param target 需要跟踪的目标值，通常为目标位置
         * @return first 为平滑后的目标值，second 为目标值的一阶导数
         */
        std::array <float, 2> update(float target);

        /**
         * @brief 清空状态，将目标值和跟踪值都重置为 0
         */
        void clear();

        /**
         * @brief 清空状态，并将当前跟踪值直接设置为指定目标值
         * @param target 重置后的目标值
         */
        void clear(float target);

        /**
         * @brief 设置跟踪微分器参数
         * @param omega 自然频率
         * @param zeta 阻尼比
         * @param frc 更新频率，单位 Hz
         */
        void set_para(float omega, float zeta, float frc);

        /**
         * @brief 更新带角度跳变处理的跟踪微分器
         * @param target 输入目标角度，范围通常为 [-PI, PI]
         * @return first 为平滑后的角度，范围被限制在 [-PI, PI]；second 为角速度
         *
         * @note 该函数用于处理角度从 PI 跳变到 -PI，或从 -PI 跳变到 PI 的情况
         * @note 目前仅支持关于 0 对称的 [-PI, PI] 角度区间
         */
        std::array <float, 2> update_limit(float target);

    private:
        /**
         * @brief 将角度限制到 [-PI, PI]
         * @param x 输入角度
         * @return 归一化后的角度
         */
        static float wrap_pi(float x);

        /**
         * @brief 更新跟踪微分器，并将输出位置限制到 [-PI, PI]
         * @param target 已经展开后的连续目标角度
         * @return first 为限制后的角度，second 为角速度
         *
         * @note 该函数不处理输入目标的跳变，只负责限制输出角度范围
         */
        std::array <float, 2> update_mod(float target);

    private:
        // 自然频率
        float omega_ = 0.0f;

        // 阻尼比
        float zeta_ = 0.0f;

        // 更新时间间隔，单位 s
        float delta_time_ = 0.0f;

        // track_.first：平滑后的跟踪值
        // track_.second：跟踪值的一阶导数
        std::array <float, 2> track_ = {0.0f, 0.0f};

        // 普通模式下表示当前目标值
        // 角度模式下表示展开后的连续目标角度
        float target_ = 0.0f;
    };

}

#endif // TRACKING_DIFFER_H