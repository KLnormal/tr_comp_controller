#include "filter/highpass_filter.h"
#include "filter/bandpass_filter.h"
#include "filter/bandstop_filter.h"
#include "filter/window_filter.h"
#include <cassert>
#include <cmath>
#include <complex>
#include <cstdio>
#include <limits>
#include <numbers>

constexpr float invalid_nan = std::numeric_limits<float>::quiet_NaN();
constexpr float inf = std::numeric_limits<float>::infinity();

struct coefficient_probe : filter::detail::biquad_filter {
    using biquad_filter::set_coefficients;
};

template<class Filter>
void check_samples(Filter subject) {
    subject.update(1.0f);
    subject.update(-0.5f);
    auto reference = subject;
    for(float invalid : {invalid_nan, inf, -inf}) assert(std::isnan(subject.update(invalid)));
    for(float sample : {0.0f, 1.0f, -2.0f, 3.0f})
        assert(subject.update(sample) == reference.update(sample));
    subject.clear();
    reference.clear();
    assert(subject.update(7.0f) == reference.update(7.0f));
}

template<class Filter>
double gain(Filter subject, double frequency) {
    std::complex<double> response = 0.0;
    for(int i = 0; i < 10000; ++i) {
        const float output = subject.update(i == 0 ? 1.0f : 0.0f);
        assert(std::isfinite(output));
        response += static_cast<double>(output) *
            std::polar(1.0, -2.0 * std::numbers::pi * frequency * i / 1000.0);
    }
    return std::abs(response);
}

void check_parameters() {
    filter::high_pass_filter hp(50, 1000);
    hp.update(1);
    auto hp_reference = hp;
    for(float invalid : {invalid_nan, inf, -inf, -1.0f, 0.0f, 500.0f, 600.0f})
        assert(!hp.set_params(invalid, 1000));
    for(float invalid : {invalid_nan, inf, -inf, -1.0f, 0.0f})
        assert(!hp.set_params(50, invalid));
    assert(!hp.set_params(0.01f, 1000)); // PR 中量化后不稳定的配置。
    assert(hp.update(0) == hp_reference.update(0));
    assert(hp.set_params(50, 1000));
    hp_reference.clear();
    assert(hp.update(1) == hp_reference.update(1));

    auto check_band = [](auto subject) {
        subject.update(1);
        auto reference = subject;
        for(float invalid : {invalid_nan, inf, -inf, -1.0f, 0.0f, 500.0f}) {
            assert(!subject.set_params(invalid, 200, 1000));
            assert(!subject.set_params(50, invalid, 1000));
        }
        for(float invalid : {invalid_nan, inf, -inf, -1.0f, 0.0f})
            assert(!subject.set_params(50, 200, invalid));
        assert(!subject.set_params(200, 50, 1000));
        assert(!subject.set_params(50, 50, 1000));
        assert(subject.update(0) == reference.update(0));
        assert(subject.set_params(50, 200, 1000));
        reference.clear();
        assert(subject.update(1) == reference.update(1));
    };
    check_band(filter::band_pass_filter(50, 200, 1000));
    check_band(filter::band_stop_filter(50, 200, 1000));
    assert(filter::high_pass_filter(0, 0).update(3) == 3);
    assert(filter::band_pass_filter(0, 0, 0).update(3) == 3);
    assert(filter::band_stop_filter(0, 0, 0).update(3) == 3);

    coefficient_probe probe;
    assert(!probe.set_coefficients(1, 0, 0, 1, -2, 1, {}));
    assert(!probe.set_coefficients(1, 0, 0, 1, 0, 1, {}));
    assert(!probe.set_coefficients(1, 0, 0, 1, 0, -1, {}));
    assert(!probe.set_coefficients(inf, 0, 0, 1, 0, 0, {}));
    assert(probe.set_coefficients(2, 0, 0, 1, 0, 0, {}));
    assert(std::isnan(probe.update(std::numeric_limits<float>::max())));
    assert(probe.update(1) == 2);
}

void check_windows() {
    filter::moving_average_filter<3> mean;
    assert(mean.update(1) == 1);
    assert(mean.update(2) == 1.5f);
    assert(std::isnan(mean.update(invalid_nan)));
    assert(mean.update(3) == 2);
    assert(mean.update(4) == 3);
    assert(mean.update(5) == 4);
    assert(mean.update(6) == 5);
    mean.clear();
    assert(mean.update(9) == 9);
    filter::median_filter<4> median;
    assert(median.update(4) == 4);
    assert(median.update(1) == 2.5f);
    assert(median.update(3) == 3);
    assert(median.update(2) == 2.5f);
    assert(median.update(8) == 2.5f);
    filter::median_filter<2> large;
    const float max = std::numeric_limits<float>::max();
    assert(large.update(max) == max);
    assert(large.update(max) == max);
    filter::trimmed_mean_filter<5> trimmed;
    assert(trimmed.update(100) == 100);
    assert(trimmed.update(1) == 50.5f);
    assert(trimmed.update(2) == 2);
    assert(trimmed.update(3) == 2.5f);
    assert(trimmed.update(-100) == 2);
    check_samples(filter::moving_average_filter<1>());
    check_samples(filter::moving_average_filter<3>());
    check_samples(filter::median_filter<4>());
    check_samples(filter::trimmed_mean_filter<5>());
}

void check_response_rejection() {
    filter::high_pass_filter hp(50, 1000);
    hp.update(1);
    const auto hp_reference = hp;
    assert(!hp.set_params(0.02f, 1000));
    auto hp_copy = hp_reference;
    assert(hp.update(0) == hp_copy.update(0));

    auto check_band = [](auto subject) {
        subject.update(1);
        auto reference = subject;
        assert(!subject.set_params(0.1f, 0.1001f, 1000));
        assert(subject.update(0) == reference.update(0));
    };
    check_band(filter::band_pass_filter(50, 200, 1000));
    check_band(filter::band_stop_filter(50, 200, 1000));
    assert(filter::high_pass_filter(0.02f, 1000).update(3) == 3);
    assert(filter::band_pass_filter(0.1f, 0.1001f, 1000).update(3) == 3);
    assert(filter::band_stop_filter(0.1f, 0.1001f, 1000).update(3) == 3);

    coefficient_probe probe;
    assert(probe.set_coefficients(1, 0.5, 0, 1, 0, 0, {{0, 1.5}}));
    probe.update(1);
    auto reference = probe;
    assert(!probe.set_coefficients(1, 0, 0, 1, 0, 0, {{0, 0}}));
    assert(probe.update(0) == reference.update(0));
}

// 独立使用复数多项式计算量化后响应，不调用生产代码的响应检查。
double quantized_gain(double b0, double b1, double b2, double a0, double a1, double a2, double omega) {
    const std::complex<long double> z = std::polar(1.0L, -static_cast<long double>(omega));
    const long double qb0 = static_cast<float>(b0 / a0);
    const long double qb1 = static_cast<float>(b1 / a0);
    const long double qb2 = static_cast<float>(b2 / a0);
    const long double qa1 = static_cast<float>(a1 / a0);
    const long double qa2 = static_cast<float>(a2 / a0);
    return static_cast<double>(std::abs((qb0 + qb1 * z + qb2 * z * z) /
                                       (1.0L + qa1 * z + qa2 * z * z)));
}

void check_response_sweep() {
    constexpr double pi = std::numbers::pi;
    constexpr double edge = std::numbers::sqrt2 / 2;
    int accepted = 0, rejected = 0;
    const auto close = [](double value, double target) {
        assert(std::isfinite(value) && std::abs(value - target) <= 0.0100001);
    };
    for(float fs : {1.0f, 1000.0f, 1000000.0f}) {
        for(float ratio : {0.000001f, 0.00001f, 0.00002f, 0.0001f, 0.001f,
                           0.01f, 0.05f, 0.2f, 0.49f, 0.4999f}) {
            const float fc = ratio * fs;
            filter::high_pass_filter hp(0.05f * fs, fs);
            if(hp.set_params(fc, fs)) {
                ++accepted;
                const double k = std::tan(pi * fc / fs);
                const double damping = std::numbers::sqrt2 * k;
                const double a0 = 1 + damping + k * k;
                const double a1 = 2 * (k * k - 1);
                const double a2 = 1 - damping + k * k;
                close(quantized_gain(1, -2, 1, a0, a1, a2, 0), 0);
                close(quantized_gain(1, -2, 1, a0, a1, a2, 2 * pi * fc / fs), edge);
                close(quantized_gain(1, -2, 1, a0, a1, a2, pi), 1);
            } else ++rejected;
            for(float width : {0.0000001f, 0.00001f, 0.001f, 0.05f}) {
                const float high = fc + width * fs;
                filter::band_pass_filter bp(0.05f * fs, 0.2f * fs, fs);
                filter::band_stop_filter bs(0.05f * fs, 0.2f * fs, fs);
                const bool bp_ok = bp.set_params(fc, high, fs);
                const bool bs_ok = bs.set_params(fc, high, fs);
                if(!bp_ok && !bs_ok) { rejected += 2; continue; }
                const double low_k = std::tan(pi * fc / fs);
                const double high_k = std::tan(pi * high / fs);
                const double bandwidth = high_k - low_k;
                const double center2 = low_k * high_k;
                const double a0 = 1 + bandwidth + center2;
                const double a1 = 2 * (center2 - 1);
                const double a2 = 1 - bandwidth + center2;
                const double center = 2 * std::atan(std::sqrt(center2));
                if(bp_ok) {
                    ++accepted;
                    close(quantized_gain(bandwidth, 0, -bandwidth, a0, a1, a2, 0), 0);
                    close(quantized_gain(bandwidth, 0, -bandwidth, a0, a1, a2, 2 * pi * fc / fs), edge);
                    close(quantized_gain(bandwidth, 0, -bandwidth, a0, a1, a2, center), 1);
                    close(quantized_gain(bandwidth, 0, -bandwidth, a0, a1, a2, 2 * pi * high / fs), edge);
                    close(quantized_gain(bandwidth, 0, -bandwidth, a0, a1, a2, pi), 0);
                } else ++rejected;
                if(bs_ok) {
                    ++accepted;
                    close(quantized_gain(1 + center2, a1, 1 + center2, a0, a1, a2, 0), 1);
                    close(quantized_gain(1 + center2, a1, 1 + center2, a0, a1, a2, 2 * pi * fc / fs), edge);
                    close(quantized_gain(1 + center2, a1, 1 + center2, a0, a1, a2, center), 0);
                    close(quantized_gain(1 + center2, a1, 1 + center2, a0, a1, a2, 2 * pi * high / fs), edge);
                    close(quantized_gain(1 + center2, a1, 1 + center2, a0, a1, a2, pi), 1);
                } else ++rejected;
            }
        }
    }
    assert(accepted > 0 && rejected > 0);
    std::printf("Quantized response sweep: %d accepted, %d rejected.\n", accepted, rejected);
}


int main() {
    check_response_rejection();
    check_response_sweep();
    check_parameters();
    check_windows();
    check_samples(filter::high_pass_filter(50, 1000));
    check_samples(filter::band_pass_filter(50, 200, 1000));
    check_samples(filter::band_stop_filter(50, 200, 1000));
    constexpr double edge_gain = std::numbers::sqrt2 / 2;
    assert(std::abs(gain(filter::high_pass_filter(50, 1000), 50) - edge_gain) < 1e-4);
    for(double edge : {50.0, 200.0}) {
        assert(std::abs(gain(filter::band_pass_filter(50, 200, 1000), edge) - edge_gain) < 1e-4);
        assert(std::abs(gain(filter::band_stop_filter(50, 200, 1000), edge) - edge_gain) < 1e-4);
    }
    // 极低频、窄带及 Nyquist 附近：接受的配置必须维持有限、有界响应。
    int accepted = 0, rejected = 0;
    auto check_tail = [&](auto subject, bool configured) {
        if(!configured) { ++rejected; return; }
        ++accepted;
        for(int i = 0; i < 100000; ++i) {
            const float output = subject.update(i == 0 ? 1.0f : 0.0f);
            assert(std::isfinite(output) && std::abs(output) < 2);
        }
    };
    for(float frequency : {0.001f, 0.01f, 0.1f, 1.0f, 50.0f, 499.0f, 499.99f}) {
        filter::high_pass_filter hp(50, 1000);
        const bool configured = hp.set_params(frequency, 1000);
        check_tail(hp, configured);
        for(float width : {0.0001f, 0.01f, 1.0f}) {
            filter::band_pass_filter bp(50, 200, 1000);
            filter::band_stop_filter bs(50, 200, 1000);
            const bool bp_ok = bp.set_params(frequency, frequency + width, 1000);
            const bool bs_ok = bs.set_params(frequency, frequency + width, 1000);
            check_tail(bp, bp_ok);
            check_tail(bs, bs_ok);
        }
    }
    std::printf("All filter checks passed; edge sweep: %d accepted, %d rejected.\n", accepted, rejected);
}
