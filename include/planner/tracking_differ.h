//
// Created by 15082 on 2026/5/26.
//

/*!
 *  @brief Tracking Differentiator, used to smooth tracking the target and get the dot_track
 *  @author KL_normal
 *  @date 2026-05-27
 */

#ifndef TRACKING_DIFFER_H
#define TRACKING_DIFFER_H

#include <utility>
#include <arm_math_types.h>
#include "math/math_macro.h"


namespace Planner {
    class tracking_differ {
    public:
        tracking_differ() = default;
        tracking_differ(float omega, float zeta, float frc);
        std::pair<float,float> update(float track);
        std::pair<float,float> get_data();
        void clear();
        void set_para(float omega, float zeta, float frc);
        std::pair<float,float> update_limit(float target);
    private:
        std::pair<float,float> update_mod(float target);
        float omega_ = 0, zeta_ = 0, delta_time_ = 0;
        //track and dot_track
        std::pair<float, float> track_ = {0, 0};
        float raw_target = 0;
    };

}

#endif //TRACKING_DIFFER_H
