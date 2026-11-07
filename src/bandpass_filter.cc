#include "filter/bandpass_filter.h"
#include <cmath>
#include <numbers>

using namespace filter;

band_pass_filter::band_pass_filter(float low_frequency, float high_frequency, float sampling_frequency) {
    set_params(low_frequency, high_frequency, sampling_frequency);
}

bool band_pass_filter::set_params(float low_frequency, float high_frequency, float sampling_frequency) {
    if(!std::isfinite(low_frequency) || !std::isfinite(high_frequency) ||
       !std::isfinite(sampling_frequency) || sampling_frequency <= 0 ||
       low_frequency <= 0 || low_frequency >= high_frequency ||
       high_frequency >= 0.5 * sampling_frequency) return false;
    const double low = std::tan(std::numbers::pi * low_frequency / sampling_frequency);
    const double high = std::tan(std::numbers::pi * high_frequency / sampling_frequency);
    const double bandwidth = high - low;
    const double center2 = low * high;
    return set_coefficients(bandwidth, 0, -bandwidth,
                            1 + bandwidth + center2, 2 * (center2 - 1), 1 - bandwidth + center2,
                            {{0, 0}, {2 * std::atan(low), std::numbers::sqrt2 / 2},
                             {2 * std::atan(std::sqrt(center2)), 1},
                             {2 * std::atan(high), std::numbers::sqrt2 / 2}, {std::numbers::pi, 0}});
}
