#pragma once
#include <cmath>
#include <numbers>
#include "biquad_filter.h"

namespace filter {
// 频率单位 Hz，0 < 截止频率 < 采样频率/2；set_params 会清空历史。
class high_pass_filter : public detail::biquad_filter {
public:
    high_pass_filter(float cut_frequency, float sampling_frequency) {
        set_params(cut_frequency, sampling_frequency);
    }

    void set_params(float cut_frequency, float sampling_frequency) {
        const double k = std::tan(std::numbers::pi * cut_frequency / sampling_frequency);
        const double k2 = k * k;
        const double damping = std::numbers::sqrt2 * k;
        set_coefficients(1, -2, 1, 1 + damping + k2, 2 * (k2 - 1), 1 - damping + k2);
    }
};

}
