#pragma once
#include "biquad_filter.h"

namespace filter {
// 频率单位 Hz，0 < 截止频率 < 采样频率/2；set_params 会清空历史。
class high_pass_filter : public detail::biquad_filter {
public:
    high_pass_filter(float cut_frequency, float sampling_frequency);
    void set_params(float cut_frequency, float sampling_frequency);
};
}
