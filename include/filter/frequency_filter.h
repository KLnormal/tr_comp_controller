#pragma once
#include <cmath>
#include <numbers>

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
