//
// Created by 15082 on 2026/5/26.
//

#include "planner/tracking_differ.h"

#include <dsp/fast_math_functions.h>


using namespace Planner;

/**
 * @param omega: natural frequency, the larger the omega, the faster the response, but also more overshoot
 * @param zeta: damping ratio, the larger the zeta, the less overshoot, but also slower response, usually set to 1
 * @param frc: the frequency of update
 */
tracking_differ::tracking_differ(float omega, float zeta, float frc)
    : omega_(omega), zeta_(zeta), delta_time_(1/frc){
}


void tracking_differ::clear() {
    raw_target = 0;
    track_ = {0, 0};
}

void tracking_differ::set_para(float omega, float zeta, float frc) {
    omega_ = omega;
    zeta_ = zeta;
    delta_time_ = 1/frc;
}

/**
 * @param track: the target to track, usually the target position
 */
std::pair<float,float> tracking_differ::update(float track) {
    raw_target = track;
    float32_t ddot_tracking =
        omega_ * omega_ * (raw_target - track_.first)
            - 2 * zeta_ * omega_ * track_.second;
    track_.second += ddot_tracking * delta_time_;
    track_.first += track_.second * delta_time_;
    return track_;
}


/**
 * @brief 处理轨迹规划中跳
 变问题，直接输入跟踪目标
 *
 * @note 目前仅支持+-PI的对称的区域跳变处理
 */
std::pair<float,float> tracking_differ::update_limit(float target) {
    float delta   = fmodf(target - raw_target, 2 * PI);
    if(fabsf(delta) > PI)
        delta -= SGN(delta) * 2 * PI;
    raw_target += delta;
    auto out = this->update_mod(raw_target);
    return out;
}

std::pair<float,float> tracking_differ::update_mod(float target) {
    auto t = this->update(target);
    t.first += PI;
    t.first = fmodf(t.first, 2*PI);
    if(t.first<0)
        t.first+=2*PI;
    t.first -= PI;
    return t;
}