//
// Created by 15082 on 2026/6/1.
//

#ifndef TROBOT_FILTER_H
#define TROBOT_FILTER_H

namespace  Filter
{
	class LowPassFilter
	{
	public:
		LowPassFilter() = default;
		LowPassFilter(float cut_frequency, float sampling_frequency);
		void clear();
		void set_params(float cut_frequency, float sampling_frequency);
		float update(float sample);
	private:
		float cut_frequency_ = 0, sampling_frequency_ = 0;
		float last_out_ = 0, current_out_ = 0;
		float filter_coe_ = 0;
	};
};

#endif //TROBOT_FILTER_H
