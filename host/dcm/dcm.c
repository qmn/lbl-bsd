#define _POSIX_C_SOURCE 200809L
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <errno.h>
#ifdef USE_FIXED
#include <stdbool.h>
#endif /* USE_FIXED */

static int16_t htoi(const char *buf) {
	int16_t n;
	unsigned int i;
	for (n = 0, i = 4; i > 0; i--) {
		uint8_t d;
		unsigned char c;
		c = (unsigned char)(*buf++);
		if ((d = c - '0') <= 0x9
		    || (d = c - 'A' + 0xA) <= 0xF
		    || (d = c - 'a' + 0xA) <= 0xF) {
			n <<= 4;
			n |= d;
		} else {
			fputs("unexpected character\n", stderr);
			exit(1);
			return 0;
		}
	}
	return n;
}

#ifdef USE_FIXED
/* Fixed point */
typedef int16_t accum_t;
#define MUL(a,b) ((accum_t)(((int32_t)(a) * (int32_t)(b)) >> 13))
#define ACCUM_ZERO  0
#define ACCUM_UNITY 8192
#define GYRO_SENS 2     /* Q3.13: (0.00875 / 37) */
#define DEG_RAD 5290    /* Q3.13: ((M_PI / 180) * 37) */
#define ACCEL_X_BIAS 11 /* Q16.0 */
#define ACCEL_X_SENS 8  /* Q3.13: (1 / 1022.40) */
#define ACCEL_Y_BIAS 1  /* Q16.0 */
#define ACCEL_Y_SENS 8  /* Q3.13: (1 / 1029.44) */
#define ACCEL_Z_BIAS 4  /* Q16.0 */
#define ACCEL_Z_SENS 8  /* Q3.13: (1 / 1002.12) */
#define IMU_DT 82       /* Q3.13: 0.01 */
#define COR_KP 9011     /* Q3.13: 1.1 */
#else
/* Floating point */
typedef float accum_t;
#define MUL(a,b) ((a) * (b))
#define ACCUM_ZERO  (0.0f)
#define ACCUM_UNITY (1.0f)
#define GYRO_SENS (0.00875f)
#define M_PI (3.14159265358979f)
#define DEG_RAD (M_PI / 180.0f)
#define ACCEL_X_BIAS (11.12f)
#define ACCEL_X_SENS (1.0f / 1022.4f)
#define ACCEL_Y_BIAS (0.72f)
#define ACCEL_Y_SENS (1.0f / 1029.44f)
#define ACCEL_Z_BIAS (4.12f)
#define ACCEL_Z_SENS (1.0f / 1002.12f)
#define IMU_DT (0.01f)
#define COR_KP (1.1f)
#define COR_KI (0.4f)
#endif

/* Direction Cosine Matrix */
static accum_t dcm[3][3] = {
	{ ACCUM_UNITY, ACCUM_ZERO, ACCUM_ZERO },
	{ ACCUM_ZERO, ACCUM_UNITY, ACCUM_ZERO },
	{ ACCUM_ZERO, ACCUM_ZERO, ACCUM_UNITY },
};

#ifdef USE_FIXED
#define ASIN_BIAS 1710 /* Q3.13: 0.208700 */
static accum_t _fxp_asin(accum_t x) {
#include "asin_lut.c"
	bool neg;
	neg = 0;
	if (x < 0) {
		neg = 1;
		x = -x;
	}
	x = (x & 0x1FFF) >> 4;
	x = lut[x];
	return (neg ? -x : x);
}
#endif

#define FMT_TRIPLET "%7.4f\t%7.4f\t%7.4f\n"
static inline void _print_vector(accum_t x, accum_t y, accum_t z) {
#ifdef USE_FIXED
	printf(FMT_TRIPLET,
		(float)(x) / 8192.0f,
		(float)(y) / 8192.0f,
		(float)(z) / 8192.0f);
#else
	printf(FMT_TRIPLET, x, y, z);
#endif
}

static inline void _dcm_renorm(void) {
	accum_t rx[3];
	accum_t ry[3];
	accum_t rz[3];
	accum_t er, rxd, ryd, rzd;
	int i;

	/* Calculate error */
	er = ACCUM_ZERO;
	for (i = 0; i < 3; i++) {
		er += MUL(dcm[0][i], dcm[1][i]);
	}
#ifdef USE_FIXED
	er >>= 1;
#else
	er *= 0.5f;
#endif
	/* Determine orthogonal vectors */
	for (i = 0; i < 3; i++) {
		rx[i] = dcm[0][i] - MUL(er, dcm[1][i]); 
		ry[i] = dcm[1][i] - MUL(er, dcm[0][i]);
	}
	rz[0] = MUL(rx[1], ry[2]) - MUL(rx[2], ry[1]);
	rz[1] = MUL(rx[2], ry[0]) - MUL(rx[0], ry[2]);
	rz[2] = MUL(rx[0], ry[1]) - MUL(rx[1], ry[0]);
	/* Calculate dot products */
	rxd = ACCUM_ZERO;
	ryd = ACCUM_ZERO;
	rzd = ACCUM_ZERO;
	for (i = 0; i < 3; i++) {
		rxd += MUL(rx[i], rx[i]);
		ryd += MUL(ry[i], ry[i]);
		rzd += MUL(rz[i], rz[i]);
	}
	/* Calculate adjustment weights */
#ifdef USE_FIXED
	rxd = (24576 - rxd) >> 1;
	ryd = (24576 - ryd) >> 1;
	rzd = (24576 - rzd) >> 1;
#else
	rxd = (3.0f - rxd) * 0.5f;
	ryd = (3.0f - ryd) * 0.5f;
	rzd = (3.0f - rzd) * 0.5f;
#endif
	/* Normalize */
	for (i = 0; i < 3; i++) {
		dcm[0][i] = MUL(rxd, rx[i]);
		dcm[1][i] = MUL(ryd, ry[i]);
		dcm[2][i] = MUL(rzd, rz[i]);
	}
}

void dcm_update(const char *str) {
	static unsigned int n = 10;
	int i;
	accum_t gyro[3];
	accum_t accel[3];
	accum_t mag[3];
	accum_t dgx, dgy, dgz;
	accum_t rpc[3];
	static accum_t tci[3] = {
		ACCUM_ZERO, ACCUM_ZERO, ACCUM_ZERO };
	accum_t pitch, roll;

	i = 0;
	do {
		gyro[i++] = (accum_t)(htoi(str));
		str += 4;
	} while (i < 3);
	i = 0;
	do {
		accel[i++] = (accum_t)(htoi(str) >> 4);
		str += 4;
	} while (i < 3);
	i = 0;
	do {
		mag[i++] = (accum_t)(htoi(str));
		str += 4;
	} while (i < 3);

	gyro[0] = MUL((gyro[0] * GYRO_SENS), DEG_RAD);
	gyro[1] = MUL((gyro[1] * GYRO_SENS), DEG_RAD);
	gyro[2] = MUL((gyro[2] * GYRO_SENS), DEG_RAD);

	accel[0] = (accel[0] - ACCEL_X_BIAS) * ACCEL_X_SENS;
	accel[1] = (accel[1] - ACCEL_Y_BIAS) * ACCEL_Y_SENS;
	accel[2] = (accel[2] - ACCEL_Z_BIAS) * ACCEL_Z_SENS;

	/* Cross-product */
	rpc[0] = MUL(dcm[2][1], accel[2]) - MUL(dcm[2][2], accel[1]);
	rpc[1] = MUL(dcm[2][2], accel[0]) - MUL(dcm[2][0], accel[2]);
	rpc[2] = MUL(dcm[2][0], accel[1]) - MUL(dcm[2][1], accel[0]);
	/* PI controller */
#ifdef USE_FIXED
	tci[0] += (rpc[0] >> 8); 
	tci[1] += (rpc[1] >> 8);
	tci[2] += (rpc[2] >> 8);
#else
	tci[0] += (COR_KI * IMU_DT) * rpc[0]; 
	tci[1] += (COR_KI * IMU_DT) * rpc[1]; 
	tci[2] += (COR_KI * IMU_DT) * rpc[2]; 
#endif
	/* Gyro drift cancellation */
	gyro[0] -= MUL(COR_KP, rpc[0]) + tci[0];
	gyro[1] -= MUL(COR_KP, rpc[1]) + tci[1];
	gyro[2] -= MUL(COR_KP, rpc[2]) + tci[2];

	/* Right-multiply DCM by infinitesimal rotation matrix */
	dgx = MUL(gyro[0], IMU_DT);
	dgy = MUL(gyro[1], IMU_DT);
	dgz = MUL(gyro[2], IMU_DT);
	for (i = 0; i < 3; i++) {
		accum_t r0, r1, r2;
		r0 = dcm[i][0];
		r1 = dcm[i][1];
		r2 = dcm[i][2];
		dcm[i][0] = r0 + MUL(dgz, r1) - MUL(dgy, r2);
		dcm[i][1] = -(MUL(dgz, r0)) + r1 + MUL(dgx, r2); 
		dcm[i][2] = MUL(dgy, r0) - MUL(dgx, r1) + r2;
	}
	_dcm_renorm();

#ifdef USE_FIXED
	pitch = _fxp_asin(dcm[2][0]);
	roll = _fxp_asin(dcm[2][1]);
#else
	pitch = asinf(dcm[2][0]);
	roll = asinf(dcm[2][1]);
#endif
	if (--n <= 0) {
		_print_vector(pitch, roll, 0);
		n = 10;
	}
}
