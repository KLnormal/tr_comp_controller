#pragma once

namespace filter::detail {
class biquad_filter {
public:
    float update(float sample);
    void clear();

protected:
    void set_coefficients(double b0, double b1, double b2, double a0, double a1, double a2);

private:
    float b0_, b1_, b2_, a1_, a2_;
    float z1_ = 0.0f, z2_ = 0.0f;
};
}
