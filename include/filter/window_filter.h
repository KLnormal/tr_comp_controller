#pragma once
#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <limits>
#include <numeric>

namespace filter {
namespace detail {
template<std::size_t N>
struct sample_window {
    static_assert(N > 0);
    std::array<float, N> samples {};
    std::size_t next = 0, count = 0;

    float push(float sample) {
        const float removed = count == N ? samples[next] : 0.0f;
        samples[next] = sample;
        next = (next + 1) % N;
        if(count < N) ++count;
        return removed;
    }

    std::array<float, N> sorted() const {
        auto result = samples;
        std::sort(result.begin(), result.begin() + count);
        return result;
    }

    void clear() { next = count = 0; }
};
}

// N 是样本数，窗口未满时仅使用已收到的样本。
// 三类窗口滤波器均拒绝非有限样本：返回 NaN，窗口及累计和不变。
// 中值及截尾均值每次更新使用 O(N log N) 排序、O(N) 临时存储。
template<std::size_t N>
class moving_average_filter {
public:
    float update(float sample) {
        if(!std::isfinite(sample)) return std::numeric_limits<float>::quiet_NaN();
        sum_ += static_cast<double>(sample) - window_.push(sample);
        return static_cast<float>(sum_ / window_.count);
    }

    void clear() { window_.clear(); sum_ = 0.0; }

private:
    detail::sample_window<N> window_;
    double sum_ = 0.0;
};

template<std::size_t N>
class median_filter {
public:
    float update(float sample) {
        if(!std::isfinite(sample)) return std::numeric_limits<float>::quiet_NaN();
        window_.push(sample);
        const auto values = window_.sorted();
        const std::size_t middle = window_.count / 2;
        if(window_.count % 2) return values[middle];
        return static_cast<float>((static_cast<double>(values[middle - 1]) + values[middle]) * 0.5);
    }

    void clear() { window_.clear(); }

private:
    detail::sample_window<N> window_;
};

// 每侧去掉 Trim 个极值；样本数不超过 2*Trim 时取普通平均。
template<std::size_t N, std::size_t Trim = 1>
class trimmed_mean_filter {
    static_assert(2 * Trim < N);
public:
    float update(float sample) {
        if(!std::isfinite(sample)) return std::numeric_limits<float>::quiet_NaN();
        window_.push(sample);
        const auto values = window_.sorted();
        const std::size_t trim = window_.count > 2 * Trim ? Trim : 0;
        const auto first = values.begin() + trim;
        const auto last = values.begin() + window_.count - trim;
        return static_cast<float>(std::accumulate(first, last, 0.0) / (last - first));
    }

    void clear() { window_.clear(); }

private:
    detail::sample_window<N> window_;
};
}
