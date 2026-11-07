#pragma once
#include <initializer_list>
#include <utility>

namespace filter::detail {
// 非有限样本或计算溢出返回 NaN，保留历史；下一次有效输入可继续更新。
class biquad_filter {
public:
    float update(float sample);
    void clear();

protected:
    // response 为 {数字角频率 rad/sample, 目标幅值}；量化后绝对幅值误差不超过 0.01。
    bool set_coefficients(double b0, double b1, double b2, double a0, double a1, double a2,
                          std::initializer_list<std::pair<double, double>> response);

private:
    float b0_ = 1.0f, b1_ = 0.0f, b2_ = 0.0f, a1_ = 0.0f, a2_ = 0.0f;
    float z1_ = 0.0f, z2_ = 0.0f;
};
}
