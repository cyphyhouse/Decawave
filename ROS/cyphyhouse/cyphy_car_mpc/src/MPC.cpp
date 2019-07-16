#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen/Dense"
#include <cmath>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"

using CppAD::AD;

// Set the timestep length and duration
size_t N = 10;
double dt = 0.1;

//Geometric parameters of car
const double lr = 0.3;

//Arena boundaries
const double x_bound = 3;
const double y_bound = 3;

//Define target states
double x_ref, y_ref;
//double y_ref = current_waypoint.y;

//Initialize
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t delta_start = v_start + N - 1;

class FG_eval {
public:
    FG_eval() = default;
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& vars) {
        //Initialize cost at 0
        fg[0] = 0;

        //State cost weights
        const int x_weight = 300;
        const int y_weight = 300;
        
        //Boundary cost weights
        const double xbound_weight = 0.1;
        const double ybound_weight = 0.1;

        //Input/input derivative cost weights
        const double delta_weight = 150;
        const double delta_rate_weight = 200;
        const double v_weight = 25;
        const double v_rate_weight = 150;

        //Set up the cost function
        for (unsigned int t = 0; t < N; ++t){
            //Penalize x-distance from waypoint and boundary
            fg[0] += x_weight * CppAD::pow(vars[x_start + t] - x_ref,2);// + xbound_weight / pow(abs(vars[x_start + t]) - x_bound,2);
            //Penalize y-distance from waypoint and boundary
            fg[0] += y_weight * CppAD::pow(vars[y_start + t] - y_ref, 2);// + ybound_weight / pow(abs(vars[y_start + t]) - y_bound,2);
        }

        //Minimize inputs
        for (unsigned int t = 0; t < N - 1; ++t) {
            fg[0] += delta_weight * CppAD::pow(vars[delta_start + t], 2);
            fg[0] += v_weight * CppAD::pow(vars[v_start + t], 2);
        }

        //Minimize input derivatives
        for (unsigned int t = 0; t < N - 2; ++t) {
            fg[0] += delta_rate_weight * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
            fg[0] += v_rate_weight * CppAD::pow(vars[v_start + t + 1] - vars[v_start + t], 2);
        }

        //Set the constraints at time t=0
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];

        for (unsigned int t = 1; t < N; ++t) {
            //State at time t+1
            AD<double> x1 = vars[x_start + t];
            AD<double> y1 = vars[y_start + t];
            AD<double> psi1 = vars[psi_start + t];

            //State at time t
            AD<double> x0 = vars[x_start + t - 1];
            AD<double> y0 = vars[y_start + t - 1];
            AD<double> psi0 = vars[psi_start + t - 1];

            //Actuations at time, t
            AD<double> delta0 = vars[delta_start + t - 1];
            AD<double> v0 = vars[v_start + t - 1];

            //Set up the SS model constraints for time steps [1,N]
            fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[1 + psi_start + t] = psi1 - (psi0 + v0 * CppAD::tan(delta0) * dt / lr);
        }
    }
};

//
// MPC class definition implementation.
//
MPC::MPC() = default;
MPC::~MPC() = default;

vector<double> MPC::Solve(Eigen::VectorXd state, geometry_msgs::Point waypoint) {
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;
    x_ref = waypoint.x;
    y_ref = waypoint.y;
    double x = state[0];
    double y = state[1];
    double psi = state[2];

    // Set number of model variables
    size_t n_vars = N * 3 + (N - 1) * 2; // 3N state elements, 2(N-1) actuators
    // Set the number of constraints
    size_t n_constraints = N * 3; // (x, y, psi)

    // Initialize model variables to zero
    Dvector vars(n_vars);
    for (unsigned int i = 0; i < n_vars; ++i) {
        vars[i] = 0;
    }

    //Set the initial state
    vars[x_start] = x;
    vars[y_start] = y;
    vars[psi_start] = psi;

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    //Define positive and negative infinities
    for (unsigned int i = 0; i < delta_start; ++i) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }
/*
    //X and Y bounds
    for (unsigned int i = x_start; i < y_start; ++i) {
        vars_lowerbound[i] = -3.0;
        vars_upperbound[i] = 3.0;
    }

    for (unsigned int i = y_start; i < psi_start; ++i) {
        vars_lowerbound[i] = -3.0;
        vars_upperbound[i] = 3.0;
    }
*/
    // Steering angle upper and lower limits [rad]
    for (unsigned int i = delta_start; i < n_vars; ++i) {
        vars_lowerbound[i] = -0.35;
        vars_upperbound[i] = 0.35;
    }

    // Velocity upper and lower limits [m/s]
    for (unsigned int i = v_start; i < delta_start; ++i) {
        vars_lowerbound[i] = -3.0;
        vars_upperbound[i] = 3.0;
    }

    // Lower and upper bounds for hard constraints (0 except for initial states)
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (unsigned int i = 0; i < n_constraints; ++i) {
        constraints_lowerbound[i] = 0.0;
        constraints_upperbound[i] = 0.0;
    }

    //Initial states constrained to last measured value
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;

    // Object that computes objective and constraints
    FG_eval fg_eval;

    //
    // NOTE: You don't have to worry about these options
    //
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    //options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.099\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
            constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    //auto cost = solution.obj_value;
    //std::cout << "Cost " << cost << std::endl;

    std::vector<double> result;

    result.push_back(solution.x[delta_start]);
    result.push_back(solution.x[v_start]);

    //Clear the mpc x & y value vectors
    this->x_vals.clear();
    this->y_vals.clear();

    //push back the predicted x,y values into the attributes
    for (unsigned int i = 1; i < N; ++i){
        this->x_vals.push_back(solution.x[x_start+i]);
        this->y_vals.push_back(solution.x[y_start+i]);
    }
    return result;
}
