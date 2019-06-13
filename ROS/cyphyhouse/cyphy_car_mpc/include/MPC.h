//
// Created by Amelia Gosse on 5/29/19.
//

#ifndef MPC_MPC_H
#define MPC_MPC_H
#include "geometry_msgs/PointStamped.h"
#include <vector>
#include "Eigen/Dense"

using namespace std;

class MPC {
public:
    MPC();

    virtual ~MPC();
    vector<double> x_vals;
    vector<double> y_vals;
    // Solve the model given an initial state
    vector<double> Solve(Eigen::VectorXd state, geometry_msgs::Point waypoint);

};

#endif //MPC_MPC_H
