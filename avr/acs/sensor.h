#ifndef LBL_BSD_SENSOR_H
#define LBL_BSD_SENSOR_H

/* L3G4200D 3-axis gyroscope */
#define IMU_GYRO_SLA 0x69
/* LSM303DLM 3-axis accelerometer and 3-axis magnetometer */
#define IMU_ACCL_SLA 0x18
#define IMU_MAG_SLA  0x1E
/* GPS NMEA parser */
#define GPS_SLA      0x2C

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

/* GPS NMEA registers */
#define GPS_LAT_MIN_H 0x00
#define GPS_LAT_MIN_L 0x01
#define GPS_LON_MIN_H 0x02
#define GPS_LON_MIN_L 0x03
#define GPS_SPEED_H   0x04
#define GPS_SPEED_L   0x05
#define GPS_COURSE_H  0x06
#define GPS_COURSE_L  0x07
#define GPS_DATE_H    0x08
#define GPS_DATE_L    0x09
#define GPS_TIME_H    0x0A
#define GPS_TIME_L    0x0B
#define GPS_LAT_DEG   0x0C
#define GPS_LON_DEG   0x0D
#define GPS_STATUS    0x0F

#define GPS_S_NO_RMC   0x00
#define GPS_S_A_FIX    0x01
#define GPS_S_V_NOFIX  0x02
#define GPS_S_OVERFLOW 0x80

/* auto-increment register address flag */
#define IMU_AUTOINCR 0x80

union sen_buf {
	/* sensor data record */
	struct sen_data {
		int16_t gyro[3];
		int16_t accl[3];
		int16_t mag[3];
		uint16_t gps_lat_min;
		uint16_t gps_lon_min;
		uint16_t gps_speed;
		uint16_t gps_cog;
		uint8_t gps_mon;
		uint8_t gps_day;
		uint16_t gps_time;
		uint8_t gps_lat_deg;
		uint8_t gps_lon_deg;
		uint8_t _gps_rsv;
		uint8_t gps_status;
	} field;
	uint8_t raw[34];
};

/* Zero-copy buffer */
extern union sen_buf sen_zbuf[2];

extern struct i2c_op sen_ins_read_op[];
extern struct i2c_op sen_gps_read_op[];

#endif /* LBL_BSD_SENSOR_H */
