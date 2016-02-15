#include <stdio.h>
#include <stdint.h>
#include "pid.h"


#ifdef SIMULATE
int16_t g_output;
#endif

void pid_reset(pid_t *p){
	p->last_err = 0;
	p->integral = 0;
}

void pid_init(pid_t *p, uint8_t kp, uint8_t ki, uint8_t kd, uint8_t dt, uint16_t max){
	p->kp = kp;
	p->ki = ki; 
	p->kd = kd;
	p->max = max;
	p->max_integral = max * 3;
	p->dt = dt;
	p->integral = 0;
	pid_reset(p);
}

int16_t pid_update(pid_t *p, int16_t setpoint, int16_t input){
	int16_t error;
	int32_t output;
	int16_t derivative=0;
	
	//get output values
	error = setpoint - input;
	p->integral += ((int32_t)error * p->dt)/100;
	if(p->integral > p->max_integral){
		p->integral = p->max_integral;
	}
	//derivative = ((error - p->last_err)/p->dt)*(int32_t)100;
	//derivative = 0; //todo: fix D-contributor

	printf("input: %d, error: %d, p->integral: %d ", input, error, p->integral);

	output = (((int32_t)p->kp*error)/100) + (((int32_t)p->ki*p->integral)/100) + (((int32_t)p->kd*derivative)/100);
	printf("output: %ld ", output);

	
	p->last_err = error;
	
	if(output > p->max)
		output = p->max;

	printf("real output: %ld\n\r", output);

	#ifdef SIMULATE
	printf("SP:%d - INPUT:%d - OUTPUT:%d\r\n", setpoint, input, output);
	#endif

	//printf("current error: %d, current output: %ld, current integral: %d, current derivative: %d\n\r", error, output, p->integral, derivative);

	return output;
} 	

#ifdef SIMULATE

int16_t pid_get_simulate_pv(void){
	static int16_t PV;
	PV = PV + (g_output * 20)/100 - (PV * 10)/100;
	return PV;
}

int main(void){
	pid_t tp;

	pid_init(&tp, 100, 150, 1, 10, 255);

	uint8_t n;
	for(n=0;n<NUMOF_LOOPS;n++){
		g_output = pid_update(&tp, SP, pid_get_simulate_pv());

	}

	return 1;
}

#endif
