#ifndef LBL_BSD_TIMER_H
#define LBL_BSD_TIMER_H

#include <stdint.h>

/* 50 Hz frame rate for servo PWM */
/* Servo PWM counter (Timer 1) tick: 2 MHz */
#define PWM_SRV_MIN 2000 /* 5% duty cycle (1 ms) */
#define PWM_SRV_MID 3000 /* 7.5% duty cycle (1.5 ms) */
#define PWM_SRV_MAX 4000 /* 10% duty cycle (2 ms) */
#define PWM_STEP ((PWM_SRV_HIGH - PWM_SRV_LOW) / 100)

/* 50 Hz frame rate for ESC PWM */
/* ESC PWM counter (Timer 0) tick: 15625 Hz */
#define PWM_ESC_OFF 16 /* 5% duty cycle (1 ms) */
#define PWM_ESC_MAX 32 /* 10% duty cycle (2 ms) */
#define PWM_ESC_STEP 1 /* 0.3125% */

extern volatile uint16_t pwm_rud;
extern volatile uint16_t pwm_elv;
extern volatile uint8_t  pwm_esc;

extern void timer_init(void);

#endif /* LBL_BSD_TIMER_H */
