#include "filter/highpass_filter.h"
#include <cmath>
#include <numbers>

using namespace filter;

high_pass_filter::high_pass_filter(float cut_frequency, float sampling_frequency) {
    set_params(cut_frequency, sampling_frequency);
}

bool high_pass_filter::set_params(float cut_frequency, float sampling_frequency) {
    if(!std::isfinite(cut_frequency) || !std::isfinite(sampling_frequency) ||
       sampling_frequency <= 0 || cut_frequency <= 0 ||
       cut_frequency >= 0.5 * sampling_frequency) return false;
    const double k = std::tan(std::numbers::pi * cut_frequency / sampling_frequency);
    const double k2 = k * k;
    const double damping = std::numbers::sqrt2 * k;
    return set_coefficients(1, -2, 1, 1 + damping + k2, 2 * (k2 - 1), 1 - damping + k2,
                            {{0, 0}, {2 * std::atan(k), std::numbers::sqrt2 / 2}, {std::numbers::pi, 1}});
}
