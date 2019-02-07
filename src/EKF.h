/*
 * EKF.h
 *
 *  Created on: 11 Sep 2018
 *      Author: sylva
 */

#ifndef EKF_H_
#define EKF_H_

#include "stm32f4xx.h"
#include "control.h"
#include "math.h"

#define GRAVITY					9.8066
#define PI_VAL					3.1416
#define IMU_RADIUS				0.03
#define ACC_GAIN				8192
#define ACC_NOISE				0.008*GRAVITY
#define GYRO_GAIN				16.4
#define GYRO_NOISE				0.1*PI_VAL/180
#define ENCODER_GAIN			500*4/(2*PI_VAL)
#define ENCODER_START_VALUE		0x7fffffff
#define ENCODER_NOISE			(0.5/ENCODER_GAIN)*(0.5/ENCODER_GAIN)
#define SAMPLE_TIME				0.001
#define MAX_JERK				0.1
#define N						51


//EKF functions
void X_Predict(void);
void P_Predict(void);
void Z_Pack( float vals[4]);
void H_Pack(void);
void Y_Calc(void);
void H_JacobianCalc(void);
void K_Calc(void);
void X_Update(void);
void P_Update(void);
void EKF_RetrieveStates(float states[]);
void EKF_RetrieveCovarianceMat(float covar_mat[]);

//Matrix Functions
void matrix_multi_3(float in_A[], int nrows_A, int ncols_A, float in_B[], int nrows_B, int ncols_B, float out[]);
void matrix_multi_4(float in_A[], int nrows_A, int ncols_A, float in_B[], int nrows_B, int ncols_B, float out[]);
void matrix_add(float in_A[], float in_B[], int nrows, int ncols, float out[]);
void matrix_subtract(float in_A[], float in_B[], int nrows, int ncols, float out[]);
void matrix_multi_4x3_3x4(float in_A[], int nrows_A, int ncols_A, float in_B[], int nrows_B, int ncols_B, float out[]);
void gluInvertMatrix_4x4(const float m[16], float invOut[16]);
#endif /* EKF_H_ */
