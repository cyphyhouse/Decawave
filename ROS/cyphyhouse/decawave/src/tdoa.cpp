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
    P(STATE_Z, STATE_Z) = powf(100,2);
    P(STATE_VX, STATE_VX) = powf(0.01,2);
    P(STATE_VY, STATE_VY) = powf(0.01,2);
    P(STATE_VZ, STATE_VZ) = powf(0.01,2);
    
    A.setIdentity(STATE_DIM,STATE_DIM);
    A(STATE_X,STATE_VX) = 0.016;
    A(STATE_Y,STATE_VY) = 0.016;
    A(STATE_Z,STATE_VZ) = 0.016;
    
    Q.setZero(STATE_DIM, STATE_DIM);
    
    stdDev = 0.15f;
}

TDOA::TDOA(Eigen::MatrixXf transition_mat, Eigen::MatrixXf prediction_mat, Eigen::MatrixXf covariance_mat, vec3d_t init_pos)
{
    tdoaCount = 0;
    nr_states = transition_mat.rows();
    
    setPredictionMat(prediction_mat);
    setTransitionMat(transition_mat);
    setCovarianceMat(covariance_mat);
    
    S.setZero(nr_states);
    S(0) = init_pos.x;
    S(1) = init_pos.y;
    S(2) = init_pos.z;
    
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

void TDOA::setCovarianceMat(Eigen::MatrixXf covariance_mat)
{
    if( (covariance_mat.rows() != nr_states) || (covariance_mat.cols() != nr_states) )
    {
        // If provided transition_mat is of wrong size, ignore input
        return;
    }

    Q = covariance_mat;
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
    static Eigen::VectorXf K(nr_states);

    // Temporary matrices for the covariance updates
    static Eigen::VectorXf PHTm(nr_states);
    static Eigen::MatrixXf I = Eigen::MatrixXf::Identity(nr_states, nr_states);

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

void TDOA::stateEstimatorPredict(double dt)
{
    A(STATE_X,STATE_VX) = A(STATE_X,STATE_X) * dt;
    A(STATE_Y,STATE_VY) = A(STATE_Y,STATE_Y) * dt;
    A(STATE_Z,STATE_VZ) = A(STATE_Z,STATE_Z) * dt;
    // Covariance update
    P = A*P*A.transpose();
    
    // Prediction Step
    // If we had info from IMU, we would add it here
    
    S[STATE_X] += S[STATE_VX] * dt;
    S[STATE_Y] += S[STATE_VY] * dt;
    S[STATE_Z] += S[STATE_VZ] * dt;
}

void TDOA::stateEstimatorPredictAcc(double dt, vec3d_t acc)
{
    A(STATE_X,STATE_VX) = A(STATE_X,STATE_X) * dt;
    A(STATE_Y,STATE_VY) = A(STATE_Y,STATE_Y) * dt;
    A(STATE_Z,STATE_VZ) = A(STATE_Z,STATE_Z) * dt;
    // Covariance update
    P = A*P*A.transpose();
    
    // Prediction Step
    // Assume acc is already rotated from body to global frame
    dt2 = dt*dt;
    S[STATE_X] += S[STATE_VX] * dt + acc.x*dt2/2.0;
    S[STATE_Y] += S[STATE_VY] * dt + acc.y*dt2/2.0;
    S[STATE_Z] += S[STATE_VZ] * dt + (acc.z - GRAVITY_MAGNITUDE)*dt2/2.0;
    S[STATE_VX] += acc.x*dt;
    S[STATE_VY] += acc.y*dt;
    S[STATE_VZ] += (acc.z - GRAVITY_MAGNITUDE)*dt;
}

void TDOA::stateEstimatorFinalize()
{
    // So far nothing happens here
    // Placeholder for future function
    
    PredictionBound();
}

void TDOA::stateEstimatorAddProcessNoise(double dt, double acc_noise)
{
    // Covariance update
    P += Q ;
    
    P(STATE_X, STATE_X) += pow(acc_noise*dt*dt,2);
    P(STATE_Y, STATE_Y) += pow(acc_noise*dt*dt,2);
    P(STATE_Z, STATE_Z) += pow(acc_noise*dt*dt,2);
    P(STATE_VX, STATE_VX) += pow(acc_noise*dt,2);
    P(STATE_VY, STATE_VY) += pow(acc_noise*dt,2);
    P(STATE_VZ, STATE_VZ) += pow(acc_noise*dt,2);
    
    PredictionBound();
}

void TDOA::PredictionBound()
{
    //Ensure boundedness and symmetry of Prediction Matrix
    for (int i=0; i<nr_states; i++) 
    {
        for (int j=i; j<nr_states; j++) 
        {
            float p = 0.5f*P(i,j) + 0.5f*P(j,i);
            if (std::isnan(p) || (p > MAX_COVARIANCE) ) 
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
//    std::cout << S.transpose() << std::endl;
    
    return pos;
}

vec3d_t TDOA::getVelocity(void)
{
    vec3d_t vel;
    vel.x = S(STATE_VX);
    vel.y = S(STATE_VY);
    vel.z = S(STATE_VZ);

    return vel;
}

