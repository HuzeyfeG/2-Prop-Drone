#ifndef BLDC_H
#define BLDC_H

#include "pico/stdlib.h"


class BLDC {
	public:
		uint8_t p_gp;
		uint16_t min_duty, max_duty;
		uint slice_num;
		
		BLDC(uint8_t gp, uint16_t min_duty, uint16_t max_duty);
		void set_power(uint16_t power);
};


#endif