#include <avr/interrupt.h>

#include "proc.h"
#include "i2c.h"
#include "sensor.h"
#include "fix.h"
#include "usart.h"
#include "timer.h"

volatile uint8_t status = FLAG_I2C;

static struct i2c_op imu_init_op[] = {
	I2C_OP_TX(IMU_GYRO_SLA, CTRL_REG1_G, 1),
	I2C_OP_TX(IMU_ACCL_SLA, CTRL_REG1_A, 1),
	I2C_OP_TX(IMU_MAG_SLA, MR_REG_M, 1),
	I2C_OP_HALT()
};
static uint8_t imu_init_tx[] = {
	0x0F, /* enable gyroscope */
	0x27, /* enable accelerometer */
	0x00, /* enable magnetometer */
};

struct i2c_op sen_ins_read_op[] = {
	I2C_OP_RX(IMU_GYRO_SLA, (OUT_X_L_G | IMU_AUTOINCR), 6),
	I2C_OP_RX(IMU_ACCL_SLA, (OUT_X_L_A | IMU_AUTOINCR), 6),
	I2C_OP_RX(IMU_MAG_SLA, (OUT_X_H_M | IMU_AUTOINCR), 6),
	I2C_OP_HALT()
};
struct i2c_op sen_gps_read_op[] = {
	I2C_OP_RX(IMU_GYRO_SLA, (OUT_X_L_G | IMU_AUTOINCR), 6),
	I2C_OP_RX(IMU_ACCL_SLA, (OUT_X_L_A | IMU_AUTOINCR), 6),
	I2C_OP_RX(IMU_MAG_SLA, (OUT_X_H_M | IMU_AUTOINCR), 6),
	I2C_OP_RX(GPS_SLA, GPS_LAT_MIN_H, 16),
	I2C_OP_HALT()
};

/* Zero-copy buffers for sensor data */
union sen_buf sen_zbuf[2];

static inline void sen_init(void) {
	i2c_send(imu_init_op, imu_init_tx);
	i2c_routine = sen_ins_read_op;
	i2c_data_rx = sen_zbuf[0].raw;
}

static inline void sen_norm(union sen_buf *buf) {
	uint8_t i;
	i = 0;
	/* Convert magnetometer values to little-endian */
	do {
		uint8_t t1, t2;
		t1 = buf->raw[i];
		t2 = buf->raw[i + 1];
		buf->raw[i++] = t2;
		buf->raw[i++] = t1;
	} while (i < 12);
}

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

static accum_t dcm[3][3] = {
	{ ACCUM_1, 0, 0 },
	{ 0, ACCUM_1, 0 },
	{ 0, 0, ACCUM_1 },
};

static inline void _dcm_renorm(void) {
	accum_t rx[3];
	accum_t ry[3];
	accum_t rz[3];
	accum_t err, rxd, ryd, rzd;
	uint8_t i;

	/* Calculate error */
	err = 0;
	for (i = 0; i < 3; i++) {
		err += muls16q(dcm[0][i], dcm[1][i]);
	}
	err >>= 1;
	/* Generate orthogonal vectors */
	for (i = 0; i < 3; i++) {
		accum_t x, y;
		x = dcm[0][i];
		y = dcm[1][i];
		rx[i] = x - muls16q(err, y);
		ry[i] = y - muls16q(err, x);
	}
	cross3q(rz, rx, ry);
	/* Calculate dot products */
	rxd = 0;
	ryd = 0;
	rzd = 0;
	for (i = 0; i < 3; i++) {
		rxd += muls16q(rx[i], rx[i]);
		ryd += muls16q(ry[i], ry[i]);
		rzd += muls16q(rz[i], rz[i]);
	}
	/* Calculate adjustment weights */
	rxd = ((ACCUM_3) - rxd) >> 1;
	ryd = ((ACCUM_3) - ryd) >> 1;
	rzd = ((ACCUM_3) - rzd) >> 1;
	/* Normalize */
	for (i = 0; i < 3; i++) {
		dcm[0][i] = muls16q(rxd, rx[i]);
		dcm[1][i] = muls16q(ryd, ry[i]);
		dcm[2][i] = muls16q(rzd, rz[i]);
	}
}

void _dcm_update(struct sen_data *sen) {
	static accum_t cor_i[3] = { ACCUM_0, ACCUM_0, ACCUM_0 };
	accum_t cor_p[3];
	accum_t dgx, dgy, dgz;
	uint8_t i;

	/* Gyroscope pre-processing */
	sen->gyro[0] = muls16q(muls16_inline(sen->gyro[0], GYRO_SENS), DEG_RAD);
	sen->gyro[1] = muls16q(muls16_inline(sen->gyro[1], GYRO_SENS), DEG_RAD);
	sen->gyro[2] = muls16q(muls16_inline(sen->gyro[2], GYRO_SENS), DEG_RAD);
	/* Accelerometer pre-processing */
	sen->accl[0] = muls16_inline(
		((sen->accl[0] >> 4) - ACCEL_X_BIAS), ACCEL_X_SENS);
	sen->accl[1] = muls16_inline(
		((sen->accl[1] >> 4) - ACCEL_Y_BIAS), ACCEL_Z_SENS);
	sen->accl[2] = muls16_inline(
		((sen->accl[2] >> 4) - ACCEL_Z_BIAS), ACCEL_Z_SENS);

	/* Roll-pitch correction */
	cross3q(cor_p, (const accum_t *)(sen->accl), dcm[2]);
	/* (COR_KI * IMU_DT) ~ 32 = 2^5 */
	cor_i[0] += (cor_p[0] >> 8);
	sen->gyro[0] -= muls16q(COR_KP, cor_p[0]) + cor_i[0];
	cor_i[1] += (cor_p[1] >> 8);
	sen->gyro[1] -= muls16q(COR_KP, cor_p[1]) + cor_i[1];
	cor_i[2] += (cor_p[2] >> 8);
	sen->gyro[2] -= muls16q(COR_KP, cor_p[2]) + cor_i[2];

	dgx = muls16q(sen->gyro[0], IMU_DT);
	dgy = muls16q(sen->gyro[1], IMU_DT);
	dgz = muls16q(sen->gyro[2], IMU_DT);
	i = 0;
	do {
		accum_t rx, ry, rz;
		rx = dcm[i][0];
		ry = dcm[i][1];
		rz = dcm[i][2];
		dcm[i][0] = rx + muls16q(dgz, ry) - muls16q(dgy, rz);
		dcm[i][1] = ry + muls16q(dgx, rz) + muls16q(dgz, rx);
		dcm[i][2] = rz + muls16q(dgy, rx) - muls16q(dgx, ry);
	} while (++i < 3);

	_dcm_renorm();
}

static void _u16toa(uint8_t *buf, uint16_t n) {
	static uint8_t hex[] = {
		'0', '1', '2', '3', '4', '5', '6', '7',
		'8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
	};
	*buf++ = hex[(n >> 12)];
	*buf++ = hex[(n >> 8) & 0xF];
	*buf++ = hex[(n >> 4) & 0xF];
	*buf   = hex[(n & 0xF)];
}

int main(void) {
	timer_init();
	i2c_init();
	sen_init();
	usart_init();
	sei();

	for (;;) {
		uint8_t _status;
		_status = status;
		if (_status & FLAG_TICK) {
			struct sen_data *sdr;
			_status &= ~(FLAG_TICK);
			sdr = (_status & SEL_SEN_ZBUF) ?
				&(sen_zbuf[0].field) : &(sen_zbuf[1].field);
			_dcm_update(sdr);

			_status |= FLAG_DCM;
			status = _status;
		}
	}

	return 0;
}
