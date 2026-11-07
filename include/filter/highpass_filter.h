#pragma once
#include "biquad_filter.h"

namespace filter {
// 频率单位 Hz，0 < 截止频率 < 采样频率/2。
// set_params 成功返回 true 并清空历史（会产生瞬态）；失败保留系数及历史。
// 仅接受有限参数、量化后稳定且截止频率及直流/Nyquist幅值绝对误差 <= 0.01 的配置。
// 超出 float 精度支持范围时拒绝；构造失败时为直通。
class high_pass_filter : public detail::biquad_filter {
public:
    high_pass_filter(float cut_frequency, float sampling_frequency);
    bool set_params(float cut_frequency, float sampling_frequency);
};
}
