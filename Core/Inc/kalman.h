/*
 * kalman.h
 *
 *  Created on: Apr 8, 2026
 *      Author: rajap
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

typedef struct {
	float x[2];        // state vector [d, v]
	float P[2][2];     // covariance
	float F[2][2];     // state transition
	float Q[2][2];     // process noise
	float H[2];        // measurement matrix (1x2)
	float R;           // measurement noise
	float K[2];        // Kalman gain
} KF_TypeDef;

void KF_Predict(KF_TypeDef*);
void KF_Update(KF_TypeDef*, float);

#endif /* INC_KALMAN_H_ */
