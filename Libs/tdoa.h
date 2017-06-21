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

#ifndef _TDOA_h
#define _TDOA_h

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>

#define STATE_X   0
#define STATE_Y   1
#define STATE_Z   2
#define STATE_VX  3
#define STATE_VY  4
#define STATE_VZ  5
#define STATE_DIM 6

#define MAX_COVARIANCE 100
#define MIN_COVARIANCE 1e-6f

typedef struct vec3d
{
	double	x;
	double	y;
	double	z;
}vec3d;

/* typedef enum
{
	STATE_X, STATE_Y, STATE_Z, STATE_PX, STATE_PY, STATE_PZ, STATE_D0, STATE_D1, STATE_D2, STATE_DIM
} stateIdx_t; */

class TDOA
{
public:
	
	// Contructor
	TDOA();
	
	void stateEstimatorUpdate(uint8_t Ar, uint8_t An, float distanceDiff);
	void stateEstimatorPredict();
	
	// Get functions
	void getPos();
	
private:
	
	//variables
	uint32_t tdoaCount;
	float stdDev;
	vec3d anchorPosition[4];
	
	Eigen::VectorXf S;
	Eigen::MatrixXf P;
	Eigen::MatrixXf A;
	
	//Functions
	void stateEstimatorScalarUpdate(Eigen::RowVectorXf H, float error, float stdMeasNoise);
	
	void initAnchorPos(void);

};

#endif
