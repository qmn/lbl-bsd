#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "global.h"
#include "i2c.h"
#include "imu.h"

static struct i2c_op imu_init_op[] = {
	I2C_OP_TX(IMU_GYRO_SLA, CTRL_REG1_G, 1),
	I2C_OP_TX(IMU_ACCEL_SLA, CTRL_REG1_A, 1),
	I2C_OP_TX(IMU_MAG_SLA, MR_REG_M, 1),
	I2C_OP_HALT()
};
static uint8_t imu_init_tx[] = {
	0x0F, /* enable gyroscope */
	0x27, /* enable accelerometer */
	0x00, /* enable magnetometer */
};

static struct i2c_op imu_read_op[] = {
	I2C_OP_RX(IMU_GYRO_SLA, (OUT_X_L_G | IMU_AUTOINCR), 6),
	I2C_OP_RX(IMU_ACCEL_SLA, (OUT_X_L_A | IMU_AUTOINCR), 6),
	I2C_OP_RX(IMU_MAG_SLA, (OUT_X_H_M | IMU_AUTOINCR), 6),
	I2C_OP_HALT()
};

union imu_raw * volatile imu_cur;

void imu_init(void) {
	i2c_tx(imu_init_op, imu_init_tx);
	i2c_routine = imu_read_op;
}

void imu_normalize(union imu_raw *raw) {
	uint8_t i;
	i = 0;
	/* Convert magnetometer values to little-endian */
	do {
		uint8_t t1, t2;
		t1 = raw->buf[i];
		t2 = raw->buf[i + 1];
		raw->buf[i++] = t2;
		raw->buf[i++] = t1;
	} while (i < 12);
}

