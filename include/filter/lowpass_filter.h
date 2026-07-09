//
// Created by 15082 on 2026/6/1.
//

#pragma once

namespace filter {
	class low_pass_filter {
	public:
		low_pass_filter() = default;
		low_pass_filter(float cut_frequency, float sampling_frequency);
		void clear();
		void set_params(float cut_frequency, float sampling_frequency);
		float update(float sample);
	private:
		float cut_frequency_ = 0, sampling_frequency_ = 0;
		float last_out_ = 0, current_out_ = 0;
		float filter_coe_ = 0;
	};
};
