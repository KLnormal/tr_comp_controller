#pragma once
#include <cmath>
#include <numbers>
#include "biquad_filter.h"

namespace filter {
// 两个 -3 dB 边界之间为阻带，频率约束与带通相同。
class band_stop_filter : public detail::biquad_filter {
public:
    band_stop_filter(float low_frequency, float high_frequency, float sampling_frequency) {
        set_params(low_frequency, high_frequency, sampling_frequency);
    }

    void set_params(float low_frequency, float high_frequency, float sampling_frequency) {
        const double low = std::tan(std::numbers::pi * low_frequency / sampling_frequency);
        const double high = std::tan(std::numbers::pi * high_frequency / sampling_frequency);
        const double bandwidth = high - low;
        const double center2 = low * high;
        set_coefficients(1 + center2, 2 * (center2 - 1), 1 + center2,
                         1 + bandwidth + center2, 2 * (center2 - 1), 1 - bandwidth + center2);
    }
};
}
