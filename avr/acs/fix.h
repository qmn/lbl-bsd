#ifndef LBL_BSD_FIX_H
#define LBL_BSD_FIX_H
#include <stdint.h>
/* Completely Redundant Arithmetic Special Hacks */

/* Assumes GCC compound statement extension */
#define muls16_inline(srca,srcb) ({ \
	register int16_t __tmp_c; \
	register int16_t __tmp_a = (srca); \
	register int16_t __tmp_b = (srcb); \
	__asm__ __volatile__ ( \
		"mul %A1, %A2" "\n\t" /* (AL * BL) */ \
		"movw %A0, r0" "\n\t" \
		"mul %B1, %A2" "\n\t" /* (AH * BL) */ \
		"add %B0, r0"  "\n\t" \
		"mul %A1, %B2" "\n\t" /* (AL * BH) */ \
		"add %B0, r0"  "\n\t" \
		"clr r1"       "\n\t" \
		: "=&r" (__tmp_c) \
		: "r" (__tmp_a), "r" (__tmp_b) \
	); \
	__tmp_c; })

/* Assumes GCC compound statement extension */
#define muls16q_inline(srca,srcb) ({ \
	register int16_t __tmp_c; \
	register int16_t __tmp_a = (srca); \
	register int16_t __tmp_b = (srcb); \
	__asm__ __volatile__ ( \
		"clr r18"        "\n\t" \
		"muls %B1, %B2"  "\n\t" /* (AH * BH) */ \
		"movw %A0, r0"   "\n\t" \
		"mul %A1, %A2"   "\n\t" /* (AL * BL) */ \
		"mov r19, r1"    "\n\t" \
		"mulsu %B1, %A2" "\n\t" /* (AH * BL) */ \
		"sbc %B0, r18"   "\n\t" \
		"add r19, r0"    "\n\t" \
		"adc %A0, r1"    "\n\t" \
		"adc %B0, r18"   "\n\t" \
		"mulsu %B2, %A1" "\n\t" /* (BH * AL) */ \
		"sbc %B0, r18"   "\n\t" \
		"add r19, r0"    "\n\t" \
		"adc %A0, r1"    "\n\t" \
		"adc %B0, r18"   "\n\t" \
		"lsl r19"        "\n\t" \
		"rol %A0"        "\n\t" \
		"rol %B0"        "\n\t" \
		"lsl r19"        "\n\t" \
		"rol %A0"        "\n\t" \
		"rol %B0"        "\n\t" \
		"lsl r19"        "\n\t" \
		"rol %A0"        "\n\t" \
		"rol %B0"        "\n\t" \
		"clr r1"         "\n\t" /* restore zero reg */ \
		: "=&r" (__tmp_c) \
		: "r" (__tmp_a), "r" (__tmp_b) \
		: \
			"r18", /* temporary zero register */ \
			"r19"  /* [31:24] bits of result */ \
	); \
	__tmp_c; })

typedef int16_t accum_t;
#define ACCUM_0 0x0000
#define ACCUM_1 0x2000
#define ACCUM_3 0x6000
#define M_PI 0x6488

extern accum_t muls16q(accum_t a, accum_t b);
extern void cross3q(accum_t *c, const accum_t *a, const accum_t *b);
extern accum_t asinq(accum_t x);
extern accum_t atan2q(accum_t y, accum_t x);

#endif /* LBL_BSD_FIX_H */
