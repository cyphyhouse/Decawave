/*************************************************
 *	
 *  Class for computing 3D position from Time-Difference of Arrival information.
 *  Position is computed using an Extended Kalman Filter.
 *
 *  
 *
 *  Created on: 06/19/2017
 *  Author: Joao Paulo Jansch Porto <janschp2(at)illinois.edu>
 *
 *  Changelog:
 *      v0.2 - initial release (06/19/2017)
 *
 *************************************************/


#include "tdoa.h"

TDOA::TDOA(void)
{
	tdoaCount = 0;
	S.setZero(STATE_DIM);
	S(STATE_X) = 2;
	S(STATE_Y) = 2.6;
	P.setZero(STATE_DIM, STATE_DIM);
	
	P(STATE_X, STATE_X) = powf(100,2);
	P(STATE_Y, STATE_Y) = powf(100,2);
	P(STATE_Z, STATE_Z) = powf(1,2);
	
	P(STATE_VX, STATE_VX) = powf(0.01,2);
	P(STATE_VY, STATE_VY) = powf(0.01,2);
	P(STATE_VZ, STATE_VZ) = powf(0.01,2);
	
	A = Eigen::MatrixXf::Identity(STATE_DIM,STATE_DIM);
	A(STATE_X,STATE_VX) = 0.016;
	A(STATE_Y,STATE_VY) = 0.016;
	A(STATE_Z,STATE_VZ) = 0.016;;
	
	initAnchorPos();
	stdDev = 0.15f;
}

void TDOA::stateEstimatorUpdate(uint8_t Ar, uint8_t An, float distanceDiff)
{
  /**
   * Measurement equation:
   * dR = dT + d1 - d0
   */

	float measurement = distanceDiff;

	// predict based on current state
	float x = S(STATE_X);
	float y = S(STATE_Y);
	float z = S(STATE_Z);

	float x1 = anchorPosition[An].x, y1 = anchorPosition[An].y, z1 = anchorPosition[An].z;
	float x0 = anchorPosition[Ar].x, y0 = anchorPosition[Ar].y, z0 = anchorPosition[Ar].z;

	float d1 = sqrtf(powf(x - x1, 2) + powf(y - y1, 2) + powf(z - z1, 2));
	float d0 = sqrtf(powf(x - x0, 2) + powf(y - y0, 2) + powf(z - z0, 2));

	float predicted = d1 - d0;
	float error = measurement - predicted;

	Eigen::RowVectorXf h = Eigen::RowVectorXf::Constant(STATE_DIM, 0);

	h(STATE_X) = ((x - x1) / d1 - (x - x0) / d0);
	h(STATE_Y) = ((y - y1) / d1 - (y - y0) / d0);
	h(STATE_Z) = ((z - z1) / d1 - (z - z0) / d0);

	stateEstimatorScalarUpdate(h, error, stdDev);

  
}

void TDOA::stateEstimatorScalarUpdate(Eigen::RowVectorXf H, float error, float stdMeasNoise)
{
	// The Kalman gain as a column vector
	static Eigen::VectorXf K(STATE_DIM);

	// Temporary matrices for the covariance updates
	static Eigen::MatrixXf tmpNN1m(STATE_DIM, STATE_DIM);
	static Eigen::MatrixXf tmpNN2m(STATE_DIM, STATE_DIM);
	static Eigen::MatrixXf tmpNN3m(STATE_DIM, STATE_DIM);
	static Eigen::VectorXf PHTm(STATE_DIM);
	static Eigen::MatrixXf I = Eigen::MatrixXf::Identity(STATE_DIM, STATE_DIM);

	// ====== INNOVATION COVARIANCE ======
	PHTm = P*H.transpose(); // PH'
	float R = stdMeasNoise*stdMeasNoise;
	float HPHR = H*PHTm + R; // HPH' + R

	// ====== MEASUREMENT UPDATE ======
	// Calculate the Kalman gain and perform the state update
//	for (int i=0; i<STATE_DIM; i++) {
//		K(i) = PHTm(i)/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
//		S(i) = S(i) + K(i) * error; // state update
//	}
	K = PHTm/HPHR;
	S = S + K*error;
	

	// ====== COVARIANCE UPDATE ======
	P = (I - K*H)*P;
/*	tmpNN1m = (K*H - I);  // (KH - I)
	tmpNN2m = tmpNN1m.transpose();  // (KH - I)'
	tmpNN3m = tmpNN1m*P;  // (KH - I)*P
	P = tmpNN3m*tmpNN2m; // (KH - I)*P*(KH - I)'

	// add the measurement variance and ensure boundedness and symmetry
	for (int i=0; i<STATE_DIM; i++) {
		for (int j=i; j<STATE_DIM; j++) {
			float v = K(i) * R * K(j);
			float p = 0.5f*P(i,j) + 0.5f*P(j,i) + v; // add measurement noise
			if (std::isnan(p) || p > MAX_COVARIANCE) {
				P(i,j) = P(j,i) = MAX_COVARIANCE;
			} else if ( i==j && p < MIN_COVARIANCE ) {
				P(i,j) = P(j,i) = MIN_COVARIANCE;
			} else {
				P(i,j) = P(j,i) = p;
			}
		}
	}*/

}

void TDOA::stateEstimatorPredict()
{
	// Covariance update
	P = A*P*A.transpose();
}

void TDOA::initAnchorPos(void)
{
	anchorPosition[0].x = 4.628;
	anchorPosition[0].y = 0.600;
	anchorPosition[0].z = 1.312;
	
	anchorPosition[1].x = 4.628;
	anchorPosition[1].y = 3.810;
	anchorPosition[1].z = 1.297;
	
	anchorPosition[2].x = 0.043;
	anchorPosition[2].y = 4.210;
	anchorPosition[2].z = 1.302;
	
	anchorPosition[3].x = 0.123;
	anchorPosition[3].y = 1.673;
	anchorPosition[3].z = 1.903;
}

void TDOA::getPos(void)
{
	std::cout << S.transpose() << std::endl;
}
