	.set _SFR_OFFSET, 0x20

; Port A Input Pins (10.3.4)
	.set PINA, 0x19
	.set PA0, 0
	.set PA1, 1
	.set PA2, 2
	.set PA3, 3
	.set PA4, 4
	.set PA5, 5
	.set PA6, 6
	.set PA7, 7
; Port A Data Direction Register (10.3.3)
	.set DDRA, 0x1A
; Port A Data Register (10.3.2)
	.set PORTA, 0x1B
; Port B Input Pins (10.3.7)
	.set PINB, 0x16
	.set PB0, 0
	.set PB1, 1
	.set PB2, 2
	.set PB3, 3
; Port B Data Direction Register (10.3.6)
	.set DDRB, 0x17
; Port B Data Register (10.3.5)
	.set PORTB, 0x18

; Timer/Counter1 Control Register A (12.11.1)
	.set TCCR1A, 0x2F
	.set WGM10, 0
	.set WGM11, 1
	.set COM1B0, 4
	.set COM1B1, 5
	.set COM1A0, 6
	.set COM1A1, 7
; Timer/Counter1 Control Register B (12.11.2)
	.set TCCR1B, 0x2E
	.set CS10, 0
	.set CS11, 1
	.set CS12, 2
	.set WGM12, 3
	.set WGM13, 4
	.set ICES1, 6
	.set ICNC1, 7
; Timer/Counter1 Control Register C (12.11.3)
	.set TCCR1C, 0x22
	.set FOC1B, 6
	.set FOC1A, 7
; Timer/Counter1 (12.11.4)
	.set TCNT1L, 0x2C
	.set TCNT1H, 0x2D
; Input Capture Register 1 (12.11.7)
	.set ICR1L, 0x24
	.set ICR1H, 0x25
; Output Compare Register 1 A (12.11.5)
	.set OCR1AL, 0x2A
	.set OCR1AH, 0x2B
; Output Compare Register 1 B (12.11.6)
	.set OCR1BL, 0x28
	.set OCR1BH, 0x29
; Timer/Counter Interrupt Flag Register 1 (12.11.9)
	.set TIFR1, 0x0B
	.set TOV1, 0
	.set OCF1A, 1
	.set OCF1B, 2
	.set ICF1, 5
; Timer/Counter Interrupt Mask Register 1 (12.11.8)
	.set TIMSK1, 0x0C
	.set TOIE1, 0
	.set OCIE1A, 1
	.set OCIE1B, 2
	.set ICIE1, 5

; Timer/Counter0 Control Register A (11.9.1)
	.set TCCR0A, 0x30
	.set WGM00, 0
	.set WGM01, 1
	.set COM0B0, 4
	.set COM0B1, 5
	.set COM0A0, 6
	.set COM0A1, 7
; Timer/Counter0 (11.9.3)
	.set TCNT0, 0x32
; Timer/Counter0 Control Register B (11.9.2)
	.set TCCR0B, 0x33
	.set CS00, 0
	.set CS01, 1
	.set CS02, 2
	.set WGM02, 3
	.set FOC0B, 6
	.set FOC0A, 7
; Output Compare Register 0 A (11.9.4)
	.set OCR0A, 0x36
; Output Compare Register 0 B (11.9.5)
	.set OCR0B, 0x3C
; Timer/Counter0 Interrupt Flag Register (11.9.7)
	.set TIFR0, 0x38
	.set TOV0, 0
	.set OCF0A, 1
	.set OCF0B, 2
; Timer/Counter0 Interrupt Mask Register (11.9.6)
	.set TIMSK0, 0x39
	.set TOIE0, 0
	.set OCIE0A, 1
	.set OCIE0B, 2

; MCU Control Register (7.5.1)
	.set MCUCR, 0x34
	.set ISC00, 0
	.set ISC01, 1
	.set BODSE, 2
	.set SM0, 3
	.set SM1, 4
	.set SE, 5
	.set PUD, 6
	.set BODS, 7

; Stack Pointer Register (4.8.1)
	.set SPL, 0x3E
	.set SPH, 0x3E
; AVR Status Register (4.8.2)
	.set SREG, 0x3F

; vim:ft=asm
