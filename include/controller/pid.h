/*!
 *  @brief Simple PID controller
 *  @author tiny_fish
 *  @date 2025-11-27
 */

#pragma once

#include <algorithm>
#include <cmath>

#include "bsp/def.h"

namespace controller {
    class pid {
    public:
        typedef struct { float kp, ki, kd, int_limit, out_limit; } pid_para_t;
        pid() = default;
        ~pid() = default;

        void clear() {
            err = lst_err = p_out = i_out = d_out = out = 0;
        }

        void init(const pid_para_t &para) {
            BSP_ASSERT(std::isfinite(para.kp) && std::isfinite(para.ki) && std::isfinite(para.kd) &&
                std::isfinite(para.int_limit) && std::isfinite(para.out_limit) &&
                para.int_limit >= 0 && para.out_limit >= 0);
            para_ = para;
            clear();
        }

        void init(const float &kp, const float &ki, const float &kd, const float &int_limit, const float &out_limit) {
            init(pid_para_t {
                .kp = kp,
                .ki = ki,
                .kd = kd,
                .int_limit = int_limit,
                .out_limit = out_limit
            });
        }

        explicit pid(const pid_para_t &para) {
            init(para);
        }

        pid(const float &kp, const float &ki, const float &kd, const float &int_limit, const float &out_limit) {
            init(kp, ki, kd, int_limit, out_limit);
        }

        float update(const float &current, const float &target) {
            if (!std::isfinite(current) || !std::isfinite(target)) {
                clear();
                return 0.0f;
            }
            lst_err = err, err = target - current;
            p_out = para_.kp * err;
            i_out += para_.ki * err;
            d_out = para_.kd * (err - lst_err);
            i_out = std::clamp(i_out, -para_.int_limit, para_.int_limit);
            out = p_out + i_out + d_out;
            out = std::clamp(out, -para_.out_limit, para_.out_limit);
            return out;
        }

        float operator()(const float &current, const float &target) {
            return update(current, target);
        }

    private:
        pid_para_t para_ = {};
        float err = 0, lst_err = 0, p_out = 0, i_out = 0, d_out = 0, out = 0;
    };
}
