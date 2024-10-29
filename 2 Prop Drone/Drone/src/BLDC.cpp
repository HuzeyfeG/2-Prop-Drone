#include <stdio.h>
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "BLDC.h"


BLDC::BLDC(uint8_t gp, uint16_t min_duty, uint16_t max_duty) {
	this->p_gp = gp;
	this->min_duty = min_duty;
	this->max_duty = max_duty;

	gpio_init(gp);
	gpio_set_function(gp, GPIO_FUNC_PWM);
	pwm_set_gpio_level(gp, 0);
	
	this->slice_num = pwm_gpio_to_slice_num(gp);

	pwm_set_clkdiv(this->slice_num, 125.0);

	pwm_set_wrap(this->slice_num, 4000);
}


void BLDC::set_power(uint16_t duty){
	if(duty >= 2000){
		duty = 2000;
	} else if(duty <= 1000){
		duty = 1000;
	}
	
	pwm_set_chan_level(this->slice_num, PWM_CHAN_A, duty);
	pwm_set_enabled(this->slice_num, true);
}







