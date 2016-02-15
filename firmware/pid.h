#ifndef PID_H_
#define PID_H_

//#define SIMULATE

#ifdef SIMULATE
#define SP 200
#define NUMOF_LOOPS 20

#endif

typedef struct{
	uint8_t kp;
	uint8_t ki;
	uint8_t kd;
	int16_t last_err;
	int16_t integral;
	int16_t max_integral;
	int16_t max;
	uint8_t dt;
}pid_t;


int16_t pid_update(pid_t *p, int16_t setpoint, int16_t input);
void pid_init(pid_t *p, uint8_t kp, uint8_t ki, uint8_t kd, uint8_t dt, uint16_t max);
int16_t pid_update(pid_t *p, int16_t setpoint, int16_t input);
void pid_print_values(pid_t *p);

#endif