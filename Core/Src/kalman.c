/*
 * kalman.c
 *
 *  Created on: Apr 8, 2026
 *      Author: rajap
 */

#include <kalman.h>

void KF_Predict(KF_TypeDef *kf) {
	// x = F * x
	float x0 = kf->F[0][0] * kf->x[0] + kf->F[0][1] * kf->x[1];
	float x1 = kf->F[1][0] * kf->x[0] + kf->F[1][1] * kf->x[1];
	kf->x[0] = x0;
	kf->x[1] = x1;

	// P = F*P*F^T + Q
	float P00 = kf->F[0][0] * kf->P[0][0] + kf->F[0][1] * kf->P[1][0];
	float P01 = kf->F[0][0] * kf->P[0][1] + kf->F[0][1] * kf->P[1][1];
	float P10 = kf->F[1][0] * kf->P[0][0] + kf->F[1][1] * kf->P[1][0];
	float P11 = kf->F[1][0] * kf->P[0][1] + kf->F[1][1] * kf->P[1][1];

	float newP00 = P00 * kf->F[0][0] + P01 * kf->F[0][1] + kf->Q[0][0];
	float newP01 = P00 * kf->F[1][0] + P01 * kf->F[1][1] + kf->Q[0][1];
	float newP10 = P10 * kf->F[0][0] + P11 * kf->F[0][1] + kf->Q[1][0];
	float newP11 = P10 * kf->F[1][0] + P11 * kf->F[1][1] + kf->Q[1][1];

	kf->P[0][0] = newP00;
	kf->P[0][1] = newP01;
	kf->P[1][0] = newP10;
	kf->P[1][1] = newP11;
}

void KF_Update(KF_TypeDef *kf, float z) {
	// Innovation
	float y = z - (kf->H[0] * kf->x[0] + kf->H[1] * kf->x[1]);

	// Innovation covariance S = H P H^T + R
	float S = kf->P[0][0] + kf->R;

	// Kalman gain K = P H^T S^-1
	kf->K[0] = kf->P[0][0] / S;
	kf->K[1] = kf->P[1][0] / S;

	// Update state
	kf->x[0] += kf->K[0] * y;
	kf->x[1] += kf->K[1] * y;

	// Update covariance
	float P00 = (1 - kf->K[0]) * kf->P[0][0];
	float P01 = (1 - kf->K[0]) * kf->P[0][1];
	float P10 = kf->P[1][0] - kf->K[1] * kf->P[0][0];
	float P11 = kf->P[1][1] - kf->K[1] * kf->P[0][1];

	kf->P[0][0] = P00;
	kf->P[0][1] = P01;
	kf->P[1][0] = P10;
	kf->P[1][1] = P11;
}
