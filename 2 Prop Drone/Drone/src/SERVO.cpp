#include <stdio.h>
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "SERVO.h"


Servo::Servo(uint8_t gp, uint16_t min_duty, uint16_t max_duty){
	this->p_gp = gp;
	this->min_duty = min_duty;
	this->max_duty = max_duty;

	gpio_init(gp);
	gpio_set_function(gp, GPIO_FUNC_PWM);
	pwm_set_gpio_level(gp, 0);
	
	//	Get the slice number of the pin.
	uint slice_num = pwm_gpio_to_slice_num(gp);

	//	Set the frequency to 50Hz. Formula: (Clock / Divider) / Wrap -> (125MHz / 125) / 20000 = 50Hz.
	pwm_set_clkdiv(slice_num, 125.0);

	// Set the wrap to 20000.
	pwm_set_wrap(slice_num, 20000);

	//	Enable pwm.
	pwm_set_enabled(slice_num, true);
}


void Servo::set_duty(int duty){
	if(duty >= 2000){
		duty = 2000;
	} else if(duty <= 1000){
		duty = 1000;
	}
	pwm_set_gpio_level(p_gp, duty);
}