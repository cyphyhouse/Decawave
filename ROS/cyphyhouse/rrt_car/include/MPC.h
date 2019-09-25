//
// Created by Amelia Gosse on 5/29/19.
//

#ifndef MPC_MPC_H
#define MPC_MPC_H
#include "geometry_msgs/PointStamped.h"
#include <deque>
#include "Eigen/Dense"


class MPC {
public:
    MPC();

    virtual ~MPC();
    std::deque<double> x_vals;
    std::deque<double> y_vals;
    // Solve the model given an initial state
    std::deque<double> Solve(Eigen::VectorXd state, std::deque<geometry_msgs::Point> waypoints);

};

#endif //MPC_MPC_H
