/*
 * kalman.c
 *
 *  Created on: Apr 8, 2026
 *      Author: rajap
 */

#include <kalman.h>

void KF_Init(KF_TypeDef *kf, float Q, float R, float dt) {
	// Initial state
	kf->x[0] = 0.0f; // distance
	kf->x[1] = 0.0f; // velocity

	// F matrix
	kf->F[0][0] = 1.0f;
	kf->F[0][1] = dt;
	kf->F[1][0] = 0.0f;
	kf->F[1][1] = 1.0f;

	// Measurement matrix
	kf->H[0] = 1.0f;  // only distance measured
	kf->H[1] = 0.0f;

	// Covariance P
	kf->P[0][0] = 1.0f;
	kf->P[0][1] = 0.0f;
	kf->P[1][0] = 0.0f;
	kf->P[1][1] = 1.0f;

	// Process noise (continuous white noise mapped to discrete)
	float sigma_squared = Q * Q;  // random variance in velocity
	float dt2 = dt * dt;
	float dt3 = dt2 * dt;
	float dt4 = dt3 * dt;

	kf->Q[0][0] = 0.25f * dt4 * sigma_squared;
	kf->Q[0][1] = 0.5f * dt3 * sigma_squared;
	kf->Q[1][0] = 0.5f * dt3 * sigma_squared;
	kf->Q[1][1] = dt2 * sigma_squared;

	// Measurement noise
	kf->R = R; // tuned based on sensor noise
}

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
	float K0 = kf->P[0][0] / S;
	float K1 = kf->P[1][0] / S;

	// Update state
	kf->x[0] += K0 * y;
	kf->x[1] += K1 * y;

	// Update covariance
	float P00 = (1 - K0) * kf->P[0][0];
	float P01 = (1 - K0) * kf->P[0][1];
	float P10 = kf->P[1][0] - K1 * kf->P[0][0];
	float P11 = kf->P[1][1] - K1 * kf->P[0][1];

	kf->P[0][0] = P00;
	kf->P[0][1] = P01;
	kf->P[1][0] = P10;
	kf->P[1][1] = P11;
}
