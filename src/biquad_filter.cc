#include "filter/biquad_filter.h"

using namespace filter::detail;

float biquad_filter::update(float sample) {
    const float output = b0_ * sample + z1_;
    z1_ = b1_ * sample - a1_ * output + z2_;
    z2_ = b2_ * sample - a2_ * output;
    return output;
}

void biquad_filter::clear() { z1_ = z2_ = 0.0f; }

void biquad_filter::set_coefficients(double b0, double b1, double b2, double a0, double a1, double a2) {
    b0_ = static_cast<float>(b0 / a0);
    b1_ = static_cast<float>(b1 / a0);
    b2_ = static_cast<float>(b2 / a0);
    a1_ = static_cast<float>(a1 / a0);
    a2_ = static_cast<float>(a2 / a0);
    clear();
}
