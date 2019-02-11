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
 *      v0.3 - Added extra library functions (06/26/2017)
 *      v0.2 - initial release (06/19/2017)
 *
 *************************************************/

#ifndef _EKF_CAR_h
#define _EKF_CAR_h

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>

#define STATE_X       0
#define STATE_Y       1
#define STATE_Z       2
#define STATE_PHI     3
#define STATE_THETA   4
#define STATE_V       5
#define STATE_PHIDOT  6
#define STATE_ACC     7
#define STATE_DIM     8

#define MAX_NR_ANCHORS 8

#define MAX_COVARIANCE 100
#define MIN_COVARIANCE 1e-6f

typedef struct vec3d_s
{
    double   x;
    double   y;
    double   z;
}vec3d_t;

class EKF
{
public:
    
    // Contructor
    EKF();
    EKF(const Eigen::MatrixXf transition_mat, const Eigen::MatrixXd prediction_mat, const Eigen::MatrixXd covariance_mat, const vec3d_t init_pos);
    
    // Set Functions
    void setTransitionMat(const Eigen::MatrixXd transition_mat);
    void setPredictionMat(const Eigen::MatrixXd prediction_mat);
    void setCovarianceMat(const Eigen::MatrixXd covariance_mat);
    
    void setAncPosition(const int anc_num, const vec3d_t anc_pos);
    void setAncPosition(const int anc_num, const double x, const double y, const double z);
    
    void setInitPos(vec3d_t init_pos);
    
    void setStdDev(double sdev);
    
    // Update functions
    void scalarTDOADistUpdate(uint8_t Ar, uint8_t An, float distanceDiff);
    void IMUUpdate(double gyro, double acc);
    void ViconUpdate(double x, double y, double z);
    void stateEstimatorPredict(const double dt, const double u1, const double u2);
    
    
    // Get functions
    vec3d_t getLocation();
	double getVelocity();
    double getAngle();
    vec3d_t getAncPosition(const int anc_num);
    
private:
    
    //variables
    uint32_t tdoaCount;
    uint32_t nr_states;
    double stdDevTDOA, stdDevGyro, stdDevAcc;
    
    vec3d_t anchorPosition[MAX_NR_ANCHORS];
    
    // Matrices used by the kalman filter
    Eigen::VectorXd S;
    Eigen::MatrixXd P;
    Eigen::MatrixXd A;
    Eigen::MatrixXd Q;
    
    //Functions
    void stateEstimatorScalarUpdate(Eigen::RowVectorXf H, double error, double stdMeasNoise);
    
    void stateEstimatorAddProcessNoise();
    void stateEstimatorFinalize();
    
    void PredictionBound();
    double AngleBound(const double angle);

};

#endif

