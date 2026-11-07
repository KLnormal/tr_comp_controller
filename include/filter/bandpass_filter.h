#pragma once
#include <cmath>
#include <numbers>
#include "biquad_filter.h"

namespace filter {
// 下、上边界均为 -3 dB：0 < low_frequency < high_frequency < sampling_frequency/2。
class band_pass_filter : public detail::biquad_filter {
public:
    band_pass_filter(float low_frequency, float high_frequency, float sampling_frequency) {
        set_params(low_frequency, high_frequency, sampling_frequency);
    }

    void set_params(float low_frequency, float high_frequency, float sampling_frequency) {
        const double low = std::tan(std::numbers::pi * low_frequency / sampling_frequency);
        const double high = std::tan(std::numbers::pi * high_frequency / sampling_frequency);
        const double bandwidth = high - low;
        const double center2 = low * high;
        set_coefficients(bandwidth, 0, -bandwidth,
                         1 + bandwidth + center2, 2 * (center2 - 1), 1 - bandwidth + center2);
    }
};

}
