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


#include "ekf_car.h"

EKF::EKF(void)
{
    tdoaCount = 0;
    nr_states = STATE_DIM;
    
    S.setZero(STATE_DIM);

    P.setZero(STATE_DIM, STATE_DIM);
    P(STATE_X, STATE_X) = powf(100,2);
    P(STATE_Y, STATE_Y) = powf(100,2);
    P(STATE_Z, STATE_Z) = powf(100,2);
    P(STATE_PHI, STATE_PHI) = powf(1,2);
    P(STATE_THETA, STATE_THETA) = powf(0.01,2);
    P(STATE_V, STATE_V) = powf(0.01,2);
    P(STATE_PHIDOT, STATE_PHIDOT) = powf(0.01,2);
    P(STATE_ACC, STATE_ACC) = powf(0.01,2);
    
    A.setIdentity(STATE_DIM,STATE_DIM);
    
    Q.setZero(STATE_DIM, STATE_DIM);
    
    stdDevTDOA = 0.15f;
}

EKF::EKF(const Eigen::MatrixXf transition_mat, const Eigen::MatrixXf prediction_mat, const Eigen::MatrixXf covariance_mat, const vec3d_t init_pos)
{
    tdoaCount = 0;
    nr_states = transition_mat.rows();
    
    setPredictionMat(prediction_mat);
    setTransitionMat(transition_mat);
    setCovarianceMat(covariance_mat);
    
    S.setZero(nr_states);
    setInitPos(init_pos);
    
    stdDev = 0.15f;
}

void EKF::setInitPos(const vec3d_t init_pos)
{
    S(0) = init_pos.x;
    S(1) = init_pos.y;
    S(2) = init_pos.z;
}

void EKF::setTransitionMat(const Eigen::MatrixXf transition_mat)
{
    if( (transition_mat.rows() != nr_states) || (transition_mat.cols() != nr_states) )
    {
        // If provided transition_mat is of wrong size, ignore input
        return;
    }

    A = transition_mat;

}

void EKF::setPredictionMat(const Eigen::MatrixXf prediction_mat)
{
    if( (prediction_mat.rows() != nr_states) || (prediction_mat.cols() != nr_states) )
    {
        // If provided transition_mat is of wrong size, ignore input
        return;
    }

    P = prediction_mat;
}

void EKF::setCovarianceMat(const Eigen::MatrixXf covariance_mat)
{
    if( (covariance_mat.rows() != nr_states) || (covariance_mat.cols() != nr_states) )
    {
        // If provided transition_mat is of wrong size, ignore input
        return;
    }

    Q = covariance_mat;
}

void EKF::setAncPosition(const int anc_num, const vec3d_t anc_pos)
{
    if( (anc_num < 0) || (anc_num > MAX_NR_ANCHORS) )
    {
        //invalid anchor number
        return;
    }

    anchorPosition[anc_num] = anc_pos;
}

void EKF::setAncPosition(const int anc_num, const float x, const float y, const float z)
{
    vec3d_t temp;
    temp.x = x;
    temp.y = y;
    temp.z = z;
    setAncPosition(anc_num, temp);
}

void EKF::setStdDev(const float sdev)
{
    stdDev = sdev;
}

vec3d_t EKF::getAncPosition(const int anc_num)
{
    return anchorPosition[anc_num];
}

void EKF::scalarTDOADistUpdate(uint8_t Ar, uint8_t An, float distanceDiff)
{

    double measurement = distanceDiff;

    // predict based on current state
    double x = S(STATE_X);
    double y = S(STATE_Y);
    double z = S(STATE_Z);

    double x1 = anchorPosition[An].x, y1 = anchorPosition[An].y, z1 = anchorPosition[An].z;
    double x0 = anchorPosition[Ar].x, y0 = anchorPosition[Ar].y, z0 = anchorPosition[Ar].z;

    double d1 = sqrtf(pow(x - x1, 2) + pow(y - y1, 2) + pow(z - z1, 2));
    double d0 = sqrtf(pow(x - x0, 2) + pow(y - y0, 2) + pow(z - z0, 2));

    double predicted = d1 - d0;
    double error = measurement - predicted;

    Eigen::RowVectorXd h = Eigen::RowVectorXd::Constant(STATE_DIM, 0);

    h(STATE_X) = ((x - x1) / d1 - (x - x0) / d0);
    h(STATE_Y) = ((y - y1) / d1 - (y - y0) / d0);
    h(STATE_Z) = ((z - z1) / d1 - (z - z0) / d0);

    stateEstimatorScalarUpdate(h, error, stdDevTDOA);

}

void EKF::stateEstimatorScalarUpdate(Eigen::RowVectorXd H, double error, double stdMeasNoise)
{
    // The Kalman gain as a column vector
    static Eigen::VectorXd K(nr_states);

    // Temporary matrices for the covariance updates
    static Eigen::VectorXd PHTm(nr_states);
    static Eigen::MatrixXd I = Eigen::MatrixXd::Identity(nr_states, nr_states);

    // ====== INNOVATION COVARIANCE ======
    PHTm = P*H.transpose(); // PH'
    double R = stdMeasNoise*stdMeasNoise;
    double HPHR = H*PHTm + R; // HPH' + R

    // ====== MEASUREMENT UPDATE ======
    // Calculate the Kalman gain and perform the state update
    K = PHTm/HPHR;
    S = S + K*error;
    

    // ====== COVARIANCE UPDATE ======
    P = (I - K*H)*P + K*R*K.transpose();
    //PredictionBound();
}

void EKF::IMUUpdate(double gyro, double acc)
{
    static Eigen::MatrixXd I = Eigen::MatrixXd::Identity(nr_states, nr_states);
    
    Eigen::MatrixXd h = Eigen::RowVectorXd::Constant(2, STATE_DIM, 0);
    h(1, STATE_ACC) = 1;
    h(2, STATE_PHIDOT) = 1;
    
    Eigen::Vector2d error;
    error(0) = acc - S(STATE_ACC);
    error(1) = gyro - S(STATE_PHIDOT);
    
    Eigen::Matrix2d R = pow(0.1, 2)*Eigen::Matrix2d::Identity();
    
    stateEstimatorUpdate(h, error, R);
}

void EKF::ViconUpdate(double x, double y, double z)
{
    static Eigen::MatrixXd I = Eigen::MatrixXd::Identity(nr_states, nr_states);
    
    Eigen::MatrixXd h = Eigen::RowVectorXd::Constant(3, STATE_DIM, 0);
    h(1, STATE_X) = 1;
    h(2, STATE_Y) = 1;
    h(3, STATE_Z) = 1;
    
    Eigen::Vector3d error;
    error(0) = x - S(STATE_X);
    error(1) = y - S(STATE_Y);
    error(2) = z - S(STATE_Z);
    
    Eigen::Matrix3d R = pow(0.01, 2)*Eigen::Matrix3d::Identity();
    
    stateEstimatorUpdate(h, error, R);
}

void EKF::stateEstimatorUpdate(Eigen::MatrixXd H, Eigen::VectorXd error, Eigen::MatrixXd R)
{
    // The Kalman gain as a column vector
    static Eigen::VectorXd K(nr_states);

    // Temporary matrices for the covariance updates
    static Eigen::MatrixXd I = Eigen::MatrixXd::Identity(nr_states, nr_states);

    // ====== INNOVATION COVARIANCE ======
    Eigen::MatrixXd HPHR = H*P*H.transpose() + R; // HPH' + R
    K = P*H.transpose() * HPHR.inverse();

    // ====== MEASUREMENT UPDATE ======
    S = S + K*error;
    
    // ====== COVARIANCE UPDATE ======
    P = (I - K*H)*P + K*R*K.transpose();
    //PredictionBound();
}

void EKF::stateEstimatorPredict(const double dt, const double u1, const double u2)
{
    double phi_k = S(STATE_PHI);
    double theta_k = S(STATE_THETA);
    double v_k = S(STATE_V);
    double a_k = S(STATE_ACC);
    
    A(STATE_X,STATE_PHI) = -v_k*sin(phi_k)*dt;
    A(STATE_X, STATE_V) = cos(phi_k)*dt;
    A(STATE_Y,STATE_PHI) = v_k*cos(phi_k)*dt;
    A(STATE_Y, STATE_V) = sin(phi_k)*dt;
    A(STATE_PHI,STATE_THETA) = ((pow(tan(theta_k),2) + 1)*v_k*dt)/d;
    A(STATE_PHI, STATE_V) = (tan(theta_k)*dt)/d;
    A(STATE_V,STATE_ACC) = dt;
    
    // Covariance update
    P = A*P*A.transpose();
    
    // Prediction
    double v_in = u1; // might need some conversion
    double theta_in = u2;
    S(STATE_X) += v_in*cos(phi_k)*dt;
    S(STATE_Y) += v_in*sin(phi_k)*dt;
    S(STATE_PHI) += (v_in*tan(theta_in)*dt)/d;
    S(STATE_V) += v_in;
}

void EKF::stateEstimatorFinalize()
{
    // So far nothing happens here
    // Placeholder for future function
    
    //PredictionBound();
}

void EKF::stateEstimatorAddProcessNoise()
{
    // Covariance update
    P += Q;
    
    //PredictionBound();
}

void EKF::PredictionBound()
{
    //Ensure boundedness and symmetry of Prediction Matrix
    for (int i=0; i<nr_states; i++) 
    {
        for (int j=i; j<nr_states; j++) 
        {
            double p = 0.5f*P(i,j) + 0.5f*P(j,i);
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

double EKF::AngleBound(const double angle)
{
    if(angle < -PI)
    {
        return angle + 2*PI;
    }
    else if(angle > PI)
    {
        return angle - 2*PI;
    }
    
    return angle;
}

vec3d_t EKF::getLocation()
{
    vec3d_t pos;
    pos.x = S(STATE_X);
    pos.y = S(STATE_Y);
    pos.z = S(STATE_Z);
    
    //For debugging purposes:
//    std::cout << S.transpose() << std::endl;
    
    return pos;
}

double EKF::getVelocity()
{
    return S(STATE_V);
}

double EKF::getAngle()
{
    return S(STATE_PHI);
}
