#pragma once
#include "biquad_filter.h"

namespace filter {
// 频率单位 Hz，两个 -3 dB 边界之间为阻带；0 < low < high < sampling/2。
// set_params 成功返回 true 并清空历史（会产生瞬态）；失败保留系数及历史。
// 仅接受有限参数且量化后极点严格位于单位圆内；构造失败时为直通。
class band_stop_filter : public detail::biquad_filter {
public:
    band_stop_filter(float low_frequency, float high_frequency, float sampling_frequency);
    bool set_params(float low_frequency, float high_frequency, float sampling_frequency);
};
}
