#pragma once
#include "biquad_filter.h"

namespace filter {
// 频率单位 Hz，下、上边界均为 -3 dB；0 < low < high < sampling/2。
// set_params 会清空历史。
class band_pass_filter : public detail::biquad_filter {
public:
    band_pass_filter(float low_frequency, float high_frequency, float sampling_frequency);
    void set_params(float low_frequency, float high_frequency, float sampling_frequency);
};
}
