#pragma once

namespace filter {
namespace detail {
class biquad_filter {
public:
    float update(float sample) {
        const float output = b0_ * sample + z1_;
        z1_ = b1_ * sample - a1_ * output + z2_;
        z2_ = b2_ * sample - a2_ * output;
        return output;
    }

    void clear() { z1_ = z2_ = 0.0f; }

protected:
    void set_coefficients(double b0, double b1, double b2, double a0, double a1, double a2) {
        b0_ = static_cast<float>(b0 / a0);
        b1_ = static_cast<float>(b1 / a0);
        b2_ = static_cast<float>(b2 / a0);
        a1_ = static_cast<float>(a1 / a0);
        a2_ = static_cast<float>(a2 / a0);
        clear();
    }

private:
    float b0_, b1_, b2_, a1_, a2_;
    float z1_ = 0.0f, z2_ = 0.0f;
};
}

}
