#ifndef __CALIBRATION_H
#define __CALIBRATION_H


typedef struct
{
	uint8_t rc_cal_flag;
	uint8_t accel_cal_flag;
	uint8_t accel_cal_finished[6];
	uint8_t accel_cal_total_finished;
	uint8_t mag_cal_flag;
	uint8_t mag_cal_finished[3];
	uint8_t mag_cal_finished_last[3];
	uint8_t mag_cal_total_finished;
	uint8_t hor_cal_flag;
  uint8_t gyro_cal_flag;
	
	uint16_t accel_cal_makesure_cnt;
	uint16_t gyro_cal_makesure_cnt;
	uint16_t accel_step_makesure_cnt[6];
	uint16_t hor_cal_makesure_cnt;
	uint16_t quit_all_cal_cnt;
	uint16_t mag_cal_makesure_cnt;
	uint16_t period_unit;
	uint8_t free;
}flight_cal_flag;


void rc_range_calibration(rc *data);
void check_calibration(void);	


void accel_calibration(float gyro_total);
void mag_calibration(void);
void hor_calibration(void);
void gyro_calibration(uint8_t *finish);

uint8_t cal_show_statemachine(void);

extern flight_cal_flag  flight_calibrate;


#endif


