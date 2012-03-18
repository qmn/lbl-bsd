#ifndef LBL_BSD_IMU_H
#define LBL_BSD_IMU_H

/* L3G4200D 3-axis gyroscope */
#define IMU_GYRO_SLA  0x69
/* LSM303DLM 3-axis accelerometer and 3-axis magnetometer */
#define IMU_ACCEL_SLA 0x18
#define IMU_MAG_SLA   0x1E

/* gyroscope registers */
#define WHO_AM_I_G 0x0F
#define CTRL_REG1_G 0x20
#define CTRL_REG5_G 0x24
#define OUT_X_L_G 0x28
#define OUT_X_H_G 0x29
#define OUT_Y_L_G 0x2A
#define OUT_Y_H_G 0x2B
#define OUT_Z_L_G 0x2C
#define OUT_Z_H_G 0x2D

/* accelerometer registers */
#define STATUS_REG_A 0x27
#define CTRL_REG1_A 0x20
#define OUT_X_L_A 0x28
#define OUT_X_H_A 0x29
#define OUT_Y_L_A 0x2A
#define OUT_Y_H_A 0x2B
#define OUT_Z_L_A 0x2C
#define OUT_Z_H_A 0x2D

/* magnetometer registers */
#define WHO_AM_I_M 0x0F
#define MR_REG_M 0x02
#define OUT_X_L_M 0x04
#define OUT_X_H_M 0x03
#define OUT_Y_L_M 0x08
#define OUT_Y_H_M 0x07
#define OUT_Z_L_M 0x06
#define OUT_Z_H_M 0x05

/* auto-increment register address flag */
#define IMU_AUTOINCR 0x80

union imu_raw {
	struct imu_out {
		uint16_t gx, gy, gz;
		uint16_t ax, ay, az;
		uint16_t mx, my, mz;
	} out;
	uint8_t buf[18];
};

extern union imu_raw * volatile imu_cur;

extern void imu_init(void);
extern void imu_normalize(union imu_raw *);

#endif /* LBL_BSD_IMU_H */
