#ifndef __AVR_ATtiny44A__ 
	.error "Unknown AVR model"
#endif
	.include "sfr_t44a.inc"
#define _SFR_MEM(io) (io + _SFR_OFFSET)
#define _RL(reg) (reg + 0)
#define _RH(reg) (reg + 1)

.ifndef F_CPU
	.error "symbol F_CPU not defined"
.endif
	.set TIMER1_PS, 3 ; prescaler (exponent to base of 2)
	.set PWM_PERIOD, ((F_CPU >> TIMER1_PS) / 50) ; 20 ms frame
	.set PWM_LOW,  (PWM_PERIOD / 16)  ; ~5.5% duty cycle
	.set PWM_HIGH, (PWM_PERIOD / 12)  ; ~8.3% duty cycle
	.set PWM_TOL,  (PWM_PERIOD / 100) ; tolerance
	.set PWM_MIN,  (PWM_LOW - PWM_TOL)
	.set PWM_MAX,  (PWM_HIGH + PWM_TOL)
	.set PWM_CNT, 20

; static checks
.ifne (PWM_LOW >> 8) | (PWM_HIGH >> 8) | (PWM_MIN >> 8) | (PWM_MAX >> 8)
	.error "PWM boundaries out of 8-bit range"
.endif
.ifle (PWM_LOW - PWM_MIN)
	.error "ensure that PWM_MIN < PWM_LOW"
.endif
.ifle (PWM_MAX - PWM_HIGH)
	.error "ensure that PWM_HIGH < PWM_MAX"
.endif
.ifle (PWM_HIGH - PWM_LOW)
	.error "ensure that PWM_LOW < PWM_HIGH"
.endif

.section .text
	.global main

; register assignments
	.set _r_tmp,  0
	.set _r_zero, 1 ; assumed initialized by crt0
	.set _r_count, 2
	.set _r_tccr1b, 21
	.set _r_t0, 22 ; r23:r22 word
	.set _r_t1, 24 ; r25:r24 word

main:
	cbi DDRA, PA7 ; Set Input Capture Port as input
	sbi DDRA, PA0 ; Set mux select pin as output
	cbi PORTA, PA0 ; Default to initial OFF state
	clr _r_count ; Reset frame counter
	; Initialize 16-bit timer
	out TCCR1A, _r_zero
	ldi _r_tccr1b, (1 << ICNC1) | (1 << ICES1) | (1 << CS11)
	out TCCR1B, _r_tccr1b
	sbi TIMSK1, ICIE1
	; Set idle mode
	ldi r16, (1 << SE)
	out MCUCR, r16
	sei
1:
	sleep
	rjmp 1b

.global __vector_5
__vector_5: ; TIM1_CAPT interrupt
	sbrs _r_tccr1b, ICES1
	rjmp 1f
	; Rising edge detected
	; Select falling edge detection for end of current pulse
	cbr _r_tccr1b, (1 << ICES1)
	out TCCR1B, _r_tccr1b
	; Sample timer
	in _RL(_r_t0), ICR1L
	in _RH(_r_t0), ICR1H
	reti
1:	; Falling edge detected
	; Select rising edge detection for subsequent pulse
	sbr _r_tccr1b, (1 << ICES1)
	out TCCR1B, _r_tccr1b
	; Sample timer
	in _RL(_r_t1), ICR1L
	in _RH(_r_t1), ICR1H
	; Calculate pulse width
	sub _RL(_r_t1), _RL(_r_t0)
	sbc _RH(_r_t1), _RH(_r_t0)

	; Check if pulse width is between PWM_MIN and PWM_LOW
	movw _r_t0, _r_t1 ; Avoid clobbering _r_t1
	subi _RL(_r_t0), PWM_MIN
	sbc  _RH(_r_t0), _r_zero
	cpi  _RL(_r_t0), (PWM_LOW - PWM_MIN)
	cpc  _RH(_r_t0), _r_zero
	brlo 2f ; (unsigned)(n - PWM_MIN) < (PWM_LOW - PWM_MIN)
	; Check if pulse width is between PWM_HIGH and PWM_MAX
	subi _RL(_r_t1), PWM_HIGH
	sbc  _RH(_r_t1), _r_zero
	cpi  _RL(_r_t1), (PWM_MAX - PWM_HIGH)
	cpc  _RH(_r_t1), _r_zero
	brlo 3f ; (unsigned)(n - PWM_HIGH) < (PWM_MAX - PWM_HIGH)
	; Out-of-range pulse width; transient error assumed
	clr _r_count ; Reset frame counter
	reti

2:	; Short pulse width detected (ON)
	sbis PORTA, PA0
	rjmp 4f
	clr _r_count
	reti ; no change in state
3:	; Long pulse width detected (OFF)
	sbic PORTA, PA0
	rjmp 4f
	clr _r_count
	reti ; no change in state
4:	; Opposite state encountered
	inc _r_count ; Increment frame counter
	ldi _r_t0, PWM_CNT
	; Check counter of consecutive identical frames
	cp _r_count, _r_t0
	brsh 5f
	reti
5:	; Reached threshold count for state transition
	sbi PINA, PA0 ; Toggle mux select pin
	clr _r_count ; Reset frame counter
	reti
