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


#include "tdoa.h"

TDOA::TDOA(void)
{
    tdoaCount = 0;
    nr_states = STATE_DIM;
    
    S.setZero(STATE_DIM);

    P.setZero(STATE_DIM, STATE_DIM);
    P(STATE_X, STATE_X) = powf(100,2);
    P(STATE_Y, STATE_Y) = powf(100,2);
    P(STATE_Z, STATE_Z) = powf(1,2);
    P(STATE_VX, STATE_VX) = powf(0.01,2);
    P(STATE_VY, STATE_VY) = powf(0.01,2);
    P(STATE_VZ, STATE_VZ) = powf(0.01,2);
    
    A.setIdentity(STATE_DIM,STATE_DIM);
    A(STATE_X,STATE_VX) = 0.016;
    A(STATE_Y,STATE_VY) = 0.016;
    A(STATE_Z,STATE_VZ) = 0.016;
    
    Q.setZero(STATE_DIM, STATE_DIM);
    
    initAnchorPos();
    stdDev = 0.15f;
}

TDOA::TDOA(Eigen::MatrixXf transition_mat, Eigen::MatrixXf prediction_mat, Eigen::MatrixXf covariance_mat)
{
    tdoaCount = 0;
    nr_states = STATE_DIM;
    
    S.setZero(STATE_DIM);
    
    P = prediction_mat;
    A = transition_mat;
    Q = covariance_mat;
    
    initAnchorPos();
    stdDev = 0.15f;
}

void TDOA::setTransitionMat(Eigen::MatrixXf transition_mat)
{
    if( (transition_mat.rows() != nr_states) || (transition_mat.cols() != nr_states) )
    {
        // If provided transition_mat is of wrong size, ignore input
        return;
    }

    A = transition_mat;

}

void TDOA::setPredictionMat(Eigen::MatrixXf prediction_mat)
{
    if( (prediction_mat.rows() != nr_states) || (prediction_mat.cols() != nr_states) )
    {
        // If provided transition_mat is of wrong size, ignore input
        return;
    }

    P = prediction_mat;
}

void TDOA::setEstimationMat(Eigen::VectorXf estimation_mat)
{
    if(estimation_mat.size() != nr_states)
    {
        // If provided transition_mat is of wrong size, ignore input
        return;
    }

    S = estimation_mat;
}

void TDOA::setAncPosition(int anc_num, vec3d_t anc_pos)
{
    if( (anc_num < 0) || (anc_num > MAX_NR_ANCHORS) )
    {
        //invalid anchor number
        return;
    }

    anchorPosition[anc_num] = anc_pos;
}

void TDOA::setAncPosition(int anc_num, float x, float y, float z)
{
    vec3d_t temp;
    temp.x = x;
    temp.y = y;
    temp.z = z;
    setAncPosition(anc_num, temp);
}

void TDOA::setStdDev(float sdev)
{
    stdDev = sdev;
}

vec3d_t TDOA::getAncPosition(int anc_num)
{
    return anchorPosition[anc_num];
}

void TDOA::scalarTDOADistUpdate(uint8_t Ar, uint8_t An, float distanceDiff)
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
    static Eigen::VectorXf PHTm(STATE_DIM);
    static Eigen::MatrixXf I = Eigen::MatrixXf::Identity(STATE_DIM, STATE_DIM);

    // ====== INNOVATION COVARIANCE ======
    PHTm = P*H.transpose(); // PH'
    float R = stdMeasNoise*stdMeasNoise;
    float HPHR = H*PHTm + R; // HPH' + R

    // ====== MEASUREMENT UPDATE ======
    // Calculate the Kalman gain and perform the state update
    K = PHTm/HPHR;
    S = S + K*error;
    

    // ====== COVARIANCE UPDATE ======
    P = (I - K*H)*P + K*R*K.transpose();
    PredictionBound();
}

void TDOA::stateEstimatorPredict()
{
    // Covariance update
    P = A*P*A.transpose();
    
    // If we had info from IMU, we would add it here
}

void TDOA::stateEstimatorFinalize()
{
    // So far nothing happens here
    // Placeholder for future function
    
    PredictionBound();
}

void TDOA::stateEstimatorAddProcessNoise()
{
    // Covariance update
    P += Q;
    
    PredictionBound();
}

void TDOA::PredictionBound()
{
    //Ensure boundedness and symmetry of Prediction Matrix
    for (int i=0; i<STATE_DIM; i++) 
    {
        for (int j=i; j<STATE_DIM; j++) 
        {
            float p = 0.5f*P(i,j) + 0.5f*P(j,i);
            if (isnan(p) || (p > MAX_COVARIANCE) ) 
            {
                P(i,j) = P(j,i) = MAX_COVARIANCE;
            } 
            else if ( (i==j) && (p < MIN_COVARIANCE) ) 
            {
                P(i,j) = P(j,i) = MIN_COVARIANCE;
            } 
            else 
            {
                P(i,j) = P(j,i) = p;
            }
        }
    }
}

vec3d_t TDOA::getLocation(void)
{
    vec3d_t pos;
    pos.x = S(STATE_X);
    pos.y = S(STATE_Y);
    pos.z = S(STATE_Z);
    
    //For debugging purposes:
    std::cout << S.transpose() << std::endl;
    
    return pos;
}

void TDOA::initAnchorPos(void)
{
    setAncPosition(0, 0.155, 0.190, 2.390);
    setAncPosition(1, 0.159, 0.792, 0.175);
    setAncPosition(2, 4.495, 0.6096, 2.381);
    setAncPosition(3, 4.498, 0.6746, 0.180);
    setAncPosition(4, 0.155, 4.240, 2.379);
    setAncPosition(5, 0.159, 4.360, 0.175);
    setAncPosition(6, 4.498, 4.342, 2.374);
    setAncPosition(7, 4.500, 4.332, 0.180);
}
