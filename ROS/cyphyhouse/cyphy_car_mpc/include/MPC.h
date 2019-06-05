//
// Created by Amelia Gosse on 5/29/19.
//

#ifndef MPC_MPC_H
#define MPC_MPC_H

#include <vector>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include "Eigen/Dense"

using namespace std;

class MPC {
public:
    MPC();

    virtual ~MPC();

    // Solve the model given an initial state
    vector<double> Solve(Eigen::VectorXd state);

};

#endif //MPC_MPC_H
