#include "filter/biquad_filter.h"
#include <cmath>
#include <limits>

using namespace filter::detail;

float biquad_filter::update(float sample) {
    if(!std::isfinite(sample)) return std::numeric_limits<float>::quiet_NaN();
    const float output = b0_ * sample + z1_;
    const float z1 = b1_ * sample - a1_ * output + z2_;
    const float z2 = b2_ * sample - a2_ * output;
    if(!std::isfinite(output) || !std::isfinite(z1) || !std::isfinite(z2))
        return std::numeric_limits<float>::quiet_NaN();
    z1_ = z1;
    z2_ = z2;
    return output;
}

void biquad_filter::clear() { z1_ = z2_ = 0.0f; }

bool biquad_filter::set_coefficients(double b0, double b1, double b2, double a0, double a1, double a2) {
    const float b0_new = static_cast<float>(b0 / a0);
    const float b1_new = static_cast<float>(b1 / a0);
    const float b2_new = static_cast<float>(b2 / a0);
    const float a1_new = static_cast<float>(a1 / a0);
    const float a2_new = static_cast<float>(a2 / a0);
    if(!std::isfinite(b0_new) || !std::isfinite(b1_new) || !std::isfinite(b2_new) ||
       !std::isfinite(a1_new) || !std::isfinite(a2_new)) return false;

    // 对最终 float 系数检查二阶 Jury 条件；用 double 求和避免边界舍入。
    if(std::abs(a2_new) >= 1.0f ||
       1.0 + static_cast<double>(a1_new) + a2_new <= 0.0 ||
       1.0 - static_cast<double>(a1_new) + a2_new <= 0.0) return false;

    b0_ = b0_new;
    b1_ = b1_new;
    b2_ = b2_new;
    a1_ = a1_new;
    a2_ = a2_new;
    clear();
    return true;
}
