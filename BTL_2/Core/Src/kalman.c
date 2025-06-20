/*
 * kalman.c
 *
 *  Created on: Jun 20, 2025
 *      Author: LENOVO
 */

#include "kalman.h"
#include "math.h"
#include "main.h"

void KalmanFilter_Init(KalmanFilter_t* kalman, double Q_angle, double Q_bias, double R_measure) {
	kalman->Q_angle = Q_angle;
	kalman->Q_bias = Q_bias;
	kalman->R_measure = R_measure;

	// Khởi tạo các giá trị mặc định
	kalman->angle = 0.0;
	kalman->bias = 0.0;
	kalman->rate = 0.0;

	// Khởi tạo ma trận hiệp phương sai
	kalman->P[0][0] = 0.0;
	kalman->P[0][1] = 0.0;
	kalman->P[1][0] = 0.0;
	kalman->P[1][1] = 0.0;
}

double KalmanFilter_Update(KalmanFilter_t* kalman, double newAngle, double newRate, double dt) {
	// Bước 1: Dự đoán
	// Cập nhật góc dựa trên tốc độ góc
	kalman->angle += dt * (newRate - kalman->bias);

	// Cập nhật ma trận hiệp phương sai
	kalman->P[0][0] += dt * (dt * kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
	kalman->P[0][1] -= dt * kalman->P[1][1];
	kalman->P[1][0] -= dt * kalman->P[1][1];
	kalman->P[1][1] += kalman->Q_bias * dt;

	// Bước 2: Cập nhật
	// Tính độ lệch giữa góc đo được và góc dự đoán
	double S = kalman->P[0][0] + kalman->R_measure;

	// Tính Kalman gain
	double K[2];
	K[0] = kalman->P[0][0] / S;
	K[1] = kalman->P[1][0] / S;

	// Cập nhật góc và độ lệch
	double y = newAngle - kalman->angle;
	kalman->angle += K[0] * y;
	kalman->bias += K[1] * y;

	// Cập nhật ma trận hiệp phương sai
	double P00_temp = kalman->P[0][0];
	double P01_temp = kalman->P[0][1];

	kalman->P[0][0] -= K[0] * P00_temp;
	kalman->P[0][1] -= K[0] * P01_temp;
	kalman->P[1][0] -= K[1] * P00_temp;
	kalman->P[1][1] -= K[1] * P01_temp;

	// Lưu tốc độ góc đã được lọc
	kalman->rate = newRate - kalman->bias;

	return kalman->angle;
}

double KalmanFilter_GetAngle(KalmanFilter_t* kalman) {
	return kalman->angle;
}

double KalmanFilter_GetRate(KalmanFilter_t* kalman) {
	return kalman->rate;
}

