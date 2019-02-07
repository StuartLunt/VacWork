/*
 * EKF.c
 *
 *  Created on: 11 Sep 2018
 *      Author: sylva
 */

#include "EKF.h"


float max_jerk = 0.1;
float n = 51; //gearbox ratio
float pi = 3.1416;

float acc_gain = 8192; // LSB/g
float acc_noise = 0.008*GRAVITY;
float gyro_gain = 16.4; // LSB/(deg/s)
float gyro_noise = 0.1*3.1416/180;
float IMU_radius = 0.03; //This affects H_jacobian
float gravity = 9.81; // m/s^2 TODO reset 9.806
float encoder_gain = 500/(2*3.1416);
uint32_t encoder_start_value = 0x7fffffff; //remember to change in encoder


float X[3] = 	{0,
				 0,
				 0};



float P[9] = 	{1, 0, 0,
				 0, 1, 0,
				 0, 0, 1};


//float32_t F[3][3] =  		{{1, sample_time,     	 	 0},
//					 	 {0,      		1, sample_time},
//						 {0,      		0,      	 1}}; TODO sample time 500 instead of 1000

float F[9] =  		{1, 	SAMPLE_TIME,    		0,
					 0, 			  1,  SAMPLE_TIME,
					 0,  			0,    			1};

float Ft[9] = 		{    1, 	0, 0,
				SAMPLE_TIME, 	1, 0,
					     0, SAMPLE_TIME, 1};

//float32_t Q[3][3] = 		{0, 0, 		 0,
//					 	 0, 0, 		 0,
//						 0, 0, max_jerk};

float Q[9] = 		{MAX_JERK*SAMPLE_TIME*SAMPLE_TIME, 0,   0,
					 0, MAX_JERK*SAMPLE_TIME,   0,
					 0, 0, MAX_JERK};

float Z[4] =   {0,
				0,
				0,
				0};

float H[4] =   {0,
				0,
				0,
				0};

float H_Jacobian[12] = {0, 			0, 					 0,
						0, 			0, -IMU_RADIUS/GRAVITY,
						0, 180/PI_VAL, 					 0,
						1, 			0, 					 0};

float Ht_Jacobian[12] = {0,          		  0, 		  0, 	1,
						 0,          		  0, 180/PI_VAL, 	0,
						 0, -IMU_RADIUS/GRAVITY, 		  0, 	0};

float Y[4] =   {0,
				0,
				0,
				0};

float K[12] = {0, 0, 0, 0,
			   0, 0, 0, 0,
			   0, 0, 0, 0};

//float R[16] = {   acc_noise,   0,           0,           0,
//        0,           acc_noise,   0,           0,
//        0,           0,           gyro_noise,  0,
//        0,           0,           0,           0.5/encoder_gain)^2};

float R[16] = {   ACC_NOISE,   0,           0,           0,
						0,           ACC_NOISE,   0,           0,
						0,           0,           GYRO_NOISE,  0,
						0,           0,           0,           ENCODER_NOISE};

float I[9] = {1, 0, 0,
			  0, 1, 0,
			  0, 0, 1};


void X_Predict(void){
	//X + 1/sample_rate * [X(2);X(3);0];
	matrix_multi_3(F,3,3,X,3,1,X);
}

void P_Predict(void){
	//predicted_P = F * P * transpose(F) + Q;
	float temp[9];

	// F * P
	matrix_multi_3(F,3,3,P,3,3,temp);
	// (F*P) * Ft
	matrix_multi_3(temp,3,3,Ft,3,3,temp);
	// (F*P) * Ft + Q
	matrix_add(temp,Q,3,3,P);
}

void Z_Pack( float vals[4]){
//    Z = [   double(acc_norm);...
//            double(acc_tang);...
//            double(gyro_vel);...
//            double(count)];

	Z[0] = vals[0];
	Z[1] = vals[1];
	Z[2] = vals[2];
	Z[3] = vals[3];

}

void H_Pack(void){
	H[0] = -(X[1] * X[1]) * IMU_RADIUS / GRAVITY + (float)cos(X[0]);
	H[1] = -IMU_RADIUS * X[2] / GRAVITY + (float)sin(X[0]);
	H[2] = 180/PI_VAL * X[1];
	H[3] = X[0];
}

void Y_Calc(void){
	matrix_subtract(Z, H, 4, 1, Y);
}


void H_JacobianCalc(void){
//    H = [       gravity*cos(X(1)),   2*X(2)*IMU_radius,    0;...
//                -gravity*sin(X(1)),         0, IMU_radius..
//                             0,       1,    0;...
// 				               1,         0,    0];
    H_Jacobian[0] = -sin(X[0]);
    H_Jacobian[1] = -2*IMU_RADIUS*X[1]/GRAVITY;
	H_Jacobian[3]=  cos(X[0]);


    Ht_Jacobian[0] = H_Jacobian[0];
    Ht_Jacobian[4] = H_Jacobian[1];
	Ht_Jacobian[1]=  H_Jacobian[3];


}

void K_Calc(void){
//	K = (P * transpose(H_jacobian(X)))...
//	    /(H_jacobian(X) * P * transpose(H_jacobian(X)) + R);

	float temp1[12];
	float temp2[16];
	float temp3[16];


	matrix_multi_4(H_Jacobian, 4, 3, P, 3, 3, temp1);
	matrix_multi_4x3_3x4(temp1, 4, 3, Ht_Jacobian, 3, 4, temp2);
	matrix_add(temp2,R,4,4,temp2);

	gluInvertMatrix_4x4(temp2,temp3);
	matrix_multi_4(P,3,3,Ht_Jacobian,3,4,temp1);
	matrix_multi_4(temp1, 3,4,temp3,4,4,K);



}

void X_Update(void){
//	X = X + K * y;
	float temp[3];
	matrix_multi_4(K,3,4,Y,4,1,temp);
	matrix_add(X,temp,3,1,X);
}

void P_Update(void){
//	P = (eye(3) - K * H_jacobian(X)) * P;
	float temp[9];
	matrix_multi_4(K,3,4,H_Jacobian,4,3,temp);
	matrix_subtract(I,temp,3,3,temp);
	matrix_multi_3(temp,3,3,P,3,3,P);
}

void EKF_RetrieveStates(float states[]){
	states[0] = X[0];
	states[1] = X[1];
	states[2] = X[2];
}

void EKF_RetrieveCovarianceMat(float covar_mat[]){
	covar_mat[0] = P[0];
	covar_mat[1] = P[1];
	covar_mat[2] = P[2];
	covar_mat[3] = P[3];
	covar_mat[4] = P[4];
	covar_mat[5] = P[5];
	covar_mat[6] = P[6];
	covar_mat[7] = P[7];
	covar_mat[8] = P[8];

}
void matrix_multi_4(float in_A[], int nrows_A, int ncols_A, float in_B[], int nrows_B, int ncols_B, float out[]) {
	int row = 0;
	int col = 0;
	float temp[nrows_A*ncols_B];

	for ( row = 0; row < nrows_A; row++){
		for (col = 0; col < ncols_B; col++){
			temp[row*ncols_B + col] = 	in_A[row 	*	ncols_A] 		  * 	  in_B[ 0 * ncols_B + col] +
										in_A[row	*	ncols_A + 1] 	  * 	  in_B[ 1 * ncols_B + col] +
										in_A[row	*	ncols_A + 2] 	  * 	  in_B[ 2 * ncols_B + col] +
										in_A[row	*	ncols_A + 3] 	  *  	  in_B[ 3 * ncols_B + col];

		}
	}

	for (row = 0; row < nrows_A; row++){
		for (col = 0; col < ncols_B; col++){
			out[row * ncols_B + col] = temp[row * ncols_B + col];
		}
	}
}

void matrix_multi_4x3_3x4(float in_A[], int nrows_A, int ncols_A, float in_B[], int nrows_B, int ncols_B, float out[]) {
	int row = 0;
	int col = 0;
	float temp[nrows_A*ncols_B];

	for ( row = 0; row < nrows_A; row++){
		for (col = 0; col < ncols_B; col++){
			temp[row*ncols_B + col] = 	in_A[row 	*	ncols_A] 		  * 	  in_B[ 0 * ncols_B + col] +
										            in_A[row	*	ncols_A + 1] 	  * 	  in_B[ 1 * ncols_B + col] +
										            in_A[row	*	ncols_A + 2] 	  * 	  in_B[ 2 * ncols_B + col];
		}
	}

	for (row = 0; row < nrows_A; row++){
		for (col = 0; col < ncols_B; col++){
			out[row * ncols_B + col] = temp[row * ncols_B + col];
		}
	}
}

void matrix_add(float in_A[], float in_B[], int nrows, int ncols, float out[]) {
	int row = 0;
	int col = 0;
	float temp[nrows*ncols];

	for ( row = 0; row < nrows; row++){
		for (col = 0; col < ncols; col++){
			temp[row*ncols+ col] = 	in_A[row *ncols + col] + in_B[ row*ncols + col];

		}
	}

	for (row = 0; row < nrows; row++){
		for (col = 0; col < ncols; col++){
			out[row * ncols + col] = temp[row * ncols + col];
		}
	}
}

void matrix_subtract(float in_A[], float in_B[], int nrows, int ncols, float out[]) {
	int row = 0;
	int col = 0;
	float temp[nrows*ncols];

	for ( row = 0; row < nrows; row++){
		for (col = 0; col < ncols; col++){
			temp[row*ncols+ col] = 	in_A[row *ncols + col] - in_B[ row*ncols + col];

		}
	}

	for (row = 0; row < nrows; row++){
		for (col = 0; col < ncols; col++){
			out[row * ncols + col] = temp[row * ncols + col];
		}
	}
}

void matrix_multi_3(float in_A[], int nrows_A, int ncols_A, float in_B[], int nrows_B, int ncols_B, float out[]) {
	int row = 0;
	int col = 0;
	float temp[nrows_A*ncols_B];

	for ( row = 0; row < nrows_A; row++){
		for (col = 0; col < ncols_B; col++){
			temp[row*ncols_B + col] = 	in_A[row 	*	ncols_A] 		  * 	  in_B[ 0 * ncols_B + col] +
										            in_A[row	*	ncols_A + 1] 	  * 	  in_B[ 1 * ncols_B + col] +
										            in_A[row	*	ncols_A + 2] 	  * 	  in_B[ 2 * ncols_B + col];

		}
	}

	for (row = 0; row < nrows_A; row++){
		for (col = 0; col < ncols_B; col++){
			out[row * ncols_B + col] = temp[row * ncols_B + col];
		}
	}
}

void gluInvertMatrix_4x4(const float m[16], float invOut[16])
{
    float inv[16], det;
    int i;

    inv[0] = m[5]  * m[10] * m[15] -
             m[5]  * m[11] * m[14] -
             m[9]  * m[6]  * m[15] +
             m[9]  * m[7]  * m[14] +
             m[13] * m[6]  * m[11] -
             m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] +
              m[4]  * m[11] * m[14] +
              m[8]  * m[6]  * m[15] -
              m[8]  * m[7]  * m[14] -
              m[12] * m[6]  * m[11] +
              m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9] * m[15] -
             m[4]  * m[11] * m[13] -
             m[8]  * m[5] * m[15] +
             m[8]  * m[7] * m[13] +
             m[12] * m[5] * m[11] -
             m[12] * m[7] * m[9];

    inv[12] = -m[4]  * m[9] * m[14] +
               m[4]  * m[10] * m[13] +
               m[8]  * m[5] * m[14] -
               m[8]  * m[6] * m[13] -
               m[12] * m[5] * m[10] +
               m[12] * m[6] * m[9];

    inv[1] = -m[1]  * m[10] * m[15] +
              m[1]  * m[11] * m[14] +
              m[9]  * m[2] * m[15] -
              m[9]  * m[3] * m[14] -
              m[13] * m[2] * m[11] +
              m[13] * m[3] * m[10];

    inv[5] = m[0]  * m[10] * m[15] -
             m[0]  * m[11] * m[14] -
             m[8]  * m[2] * m[15] +
             m[8]  * m[3] * m[14] +
             m[12] * m[2] * m[11] -
             m[12] * m[3] * m[10];

    inv[9] = -m[0]  * m[9] * m[15] +
              m[0]  * m[11] * m[13] +
              m[8]  * m[1] * m[15] -
              m[8]  * m[3] * m[13] -
              m[12] * m[1] * m[11] +
              m[12] * m[3] * m[9];

    inv[13] = m[0]  * m[9] * m[14] -
              m[0]  * m[10] * m[13] -
              m[8]  * m[1] * m[14] +
              m[8]  * m[2] * m[13] +
              m[12] * m[1] * m[10] -
              m[12] * m[2] * m[9];

    inv[2] = m[1]  * m[6] * m[15] -
             m[1]  * m[7] * m[14] -
             m[5]  * m[2] * m[15] +
             m[5]  * m[3] * m[14] +
             m[13] * m[2] * m[7] -
             m[13] * m[3] * m[6];

    inv[6] = -m[0]  * m[6] * m[15] +
              m[0]  * m[7] * m[14] +
              m[4]  * m[2] * m[15] -
              m[4]  * m[3] * m[14] -
              m[12] * m[2] * m[7] +
              m[12] * m[3] * m[6];

    inv[10] = m[0]  * m[5] * m[15] -
              m[0]  * m[7] * m[13] -
              m[4]  * m[1] * m[15] +
              m[4]  * m[3] * m[13] +
              m[12] * m[1] * m[7] -
              m[12] * m[3] * m[5];

    inv[14] = -m[0]  * m[5] * m[14] +
               m[0]  * m[6] * m[13] +
               m[4]  * m[1] * m[14] -
               m[4]  * m[2] * m[13] -
               m[12] * m[1] * m[6] +
               m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] +
              m[1] * m[7] * m[10] +
              m[5] * m[2] * m[11] -
              m[5] * m[3] * m[10] -
              m[9] * m[2] * m[7] +
              m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] -
             m[0] * m[7] * m[10] -
             m[4] * m[2] * m[11] +
             m[4] * m[3] * m[10] +
             m[8] * m[2] * m[7] -
             m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] +
               m[0] * m[7] * m[9] +
               m[4] * m[1] * m[11] -
               m[4] * m[3] * m[9] -
               m[8] * m[1] * m[7] +
               m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] -
              m[0] * m[6] * m[9] -
              m[4] * m[1] * m[10] +
              m[4] * m[2] * m[9] +
              m[8] * m[1] * m[6] -
              m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];



    det = 1.0 / det;

    for (i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;

}

