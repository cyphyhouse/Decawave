//
// Created by Amelia Gosse on 5/29/19.
//

#ifndef MPC_MPC_H
#define MPC_MPC_H
#include "geometry_msgs/PointStamped.h"
#include <vector>
#include "Eigen/Dense"


class MPC {
public:
    MPC();

    virtual ~MPC();
    std::vector<double> x_vals;
    std::vector<double> y_vals;
    // Solve the model given an initial state
    std::vector<double> Solve(Eigen::VectorXd state, std::vector<geometry_msgs::Point> waypoints);

};

#endif //MPC_MPC_H
