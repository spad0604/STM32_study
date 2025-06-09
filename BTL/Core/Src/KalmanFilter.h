/*
 * KalmanFilter.h
 *
 *  Created on: Jun 3, 2025
 *      Author: LENOVO
 */

#ifndef SRC_KALMANFILTER_H_
#define SRC_KALMANFILTER_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	double angle;    // Góc đã được lọc
	double bias;     // Độ lệch góc
	double rate;     // Tốc độ góc đã được lọc
	
	// Ma trận hiệp phương sai
	double P[2][2];
	
	// Các tham số Kalman
	double Q_angle;  // Nhiễu quá trình góc
	double Q_bias;   // Nhiễu quá trình độ lệch
	double R_measure;// Nhiễu đo lường
} KalmanFilter_t;

// Khởi tạo Kalman Filter
void KalmanFilter_Init(KalmanFilter_t* kalman, double Q_angle, double Q_bias, double R_measure);

// Cập nhật góc với dữ liệu mới
double KalmanFilter_Update(KalmanFilter_t* kalman, double newAngle, double newRate, double dt);

// Lấy góc đã được lọc
double KalmanFilter_GetAngle(KalmanFilter_t* kalman);

// Lấy tốc độ góc đã được lọc
double KalmanFilter_GetRate(KalmanFilter_t* kalman);

#ifdef __cplusplus
}
#endif

#endif /* SRC_KALMANFILTER_H_ */
