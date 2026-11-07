#pragma once
#include "biquad_filter.h"

namespace filter {
// 频率单位 Hz，下、上边界均为 -3 dB；0 < low < high < sampling/2。
// set_params 成功返回 true 并清空历史（会产生瞬态）；失败保留系数及历史。
// 仅接受有限参数、量化后稳定且两个边界、预畸变中心及直流/Nyquist幅值绝对误差 <= 0.01 的配置。
// 超出 float 精度支持范围时拒绝；构造失败时为直通。
class band_pass_filter : public detail::biquad_filter {
public:
    band_pass_filter(float low_frequency, float high_frequency, float sampling_frequency);
    bool set_params(float low_frequency, float high_frequency, float sampling_frequency);
};
}
