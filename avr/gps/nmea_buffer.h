/*
 * nmea_buffer.h
 *
 * Contains indices into the NMEA parser's buffer
 *
 * LAT is given in milli-minutes, and no degree component.
 * LON is given in milli-minutes, and no degree component.
 * TIME is given as seconds past the hour
 *
 *
 *
 */

// NOT SHIFTED.
#define NP_SLA_ADDR 44

// With changes for little-endianness
#define NP_LAT_L     0
#define NP_LAT_H     1
#define NP_LON_L     2
#define NP_LON_H     3
#define NP_SPEED_L   4
#define NP_SPEED_H   5
#define NP_COURSE_L  6
#define NP_COURSE_H  7
#define NP_DATE_L    8
#define NP_DATE_H    9
#define NP_TIME_L   10
#define NP_TIME_H   11
#define NP_LAT_DEG  12
#define NP_LON_DEG  13
#define NP_STATUS   15

// Status bits
#define NP_S_NO_RMC   (1 << 0)
#define NP_S_A_FIX    (1 << 1)
#define NP_S_V_NOFIX  (1 << 2)
#define NP_S_OVERFLOW (1 << 7)
