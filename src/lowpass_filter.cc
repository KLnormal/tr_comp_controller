//
// Created by 15082 on 2026/6/1.
//

#include "filter/lowpass_filter.h"
#include "dsp/fast_math_functions.h"

using namespace filter;

/**
 * @brief 低通滤波器
 * @param cut_frequency 截止频率（-3dB增益点）
 * @param sample_frec 采样频率
 */
low_pass_filter::low_pass_filter(float cut_frequency, float sampling_frequency)
	:cut_frequency_(cut_frequency), sampling_frequency_(sampling_frequency) {
	filter_coe_ = 1/(1+sampling_frequency/(2*PI*cut_frequency));
}

void low_pass_filter::clear() {
	last_out_ = 0;
	current_out_ = 0;
}

void low_pass_filter::set_params(float cut_frequency, float sampling_frequency) {
	cut_frequency_ = cut_frequency;
	sampling_frequency_ = sampling_frequency;
	filter_coe_ = 1/(1+sampling_frequency/(2*PI*cut_frequency));
}

float low_pass_filter::update(float sample) {
	last_out_ = current_out_;
	current_out_ = (1-filter_coe_)*last_out_ + filter_coe_*sample;
	return current_out_;
}
