#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TO_DID: Set the timestep length and duration
const size_t N = 10;
const double dt = 0.10;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// reference velocity
const double ref_v = 100;

//weighting factors to tune the MPC
const double cte_wt      = 160.0;  // cross-track error
const double epsi_wt     = 40.0;  // psi error
const double v_wt        = 0.1 ;  // velocity
const double delta_wt    = 800.0;  // steering delta
const double a_wt        = 0.1;  // acceleration
const double deltadot_wt = 1000.0;  // steering delta change
const double adot_wt     = 0.01;  // acceleration change

//weighting factors for ref_v=80;
// const double cte_wt       = 20.0;  // cross-track error
// const double epsi_wt      = 60.0;  // psi error
// const double v_wt         = 1.0 ;  // velocity
// const double delta_wt     = 160.0;  // steering delta
// const double a_wt         = 10.0;  // acceleration
// const double deltadot_wt  = 8.0;  // steering delta change
// const double adot_wt      = 0.1;  // acceleration change

// specify starting points for state & actuator variables
const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TO_DID: implement MPC
    // `fg` a vector of the cost constraints, 
    // `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // The cost is stored in the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // Reference State Cost
    // TO_DID: Define the cost related the reference state and
    // any anything you think may be beneficial.
    for (int t=0; t<N; t++) {
      fg[0] += cte_wt * CppAD::pow(vars[cte_start+t], 2);
      fg[0] += epsi_wt * CppAD::pow(vars[epsi_start+t], 2);
      fg[0] += v_wt * CppAD::pow(vars[v_start+t] - ref_v, 2);
    }

    // Actuator Cost
    for (int t=0; t < N-1; t++) {
      fg[0] += delta_wt * CppAD::pow(vars[delta_start+t], 2);
      fg[0] += a_wt * CppAD::pow(vars[a_start+t], 2);
    }

    // Value gap between sequential Actuator Cost
    for (int t=0; t < N-2; t++) {
      fg[0] += deltadot_wt * CppAD::pow(vars[delta_start+t+1] - vars[delta_start+t], 2);
      fg[0] += adot_wt * CppAD::pow(vars[a_start+t+1] - vars[a_start+t], 2);
    }

    //
    // Setup model constraints
    //
    // Initial constraints
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t=0; t < N-1; t++) {
      // vars at time t
      AD<double> x0 = vars[x_start + t];
      AD<double> y0 = vars[y_start + t];
      AD<double> psi0 = vars[psi_start + t];
      AD<double> v0 = vars[v_start + t];
      AD<double> cte0 = vars[cte_start + t];
      AD<double> epsi0 = vars[epsi_start + t];

      // vars at time t+1
      AD<double> x1 = vars[x_start + t+1];
      AD<double> y1 = vars[y_start + t+1];
      AD<double> psi1 = vars[psi_start + t+1];
      AD<double> v1 = vars[v_start + t+1];
      AD<double> cte1 = vars[cte_start + t+1];
      AD<double> epsi1 = vars[epsi_start + t+1];

      // actuation at time t
      AD<double> delta0 = vars[delta_start + t];
      AD<double> a0 = vars[a_start + t];

      AD<double> f0 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*x0*x0 + coeffs[3]*x0*x0*x0;
      AD<double> psides0 = CppAD::atan(3*coeffs[3]*x0*x0 + 2*coeffs[2]*x0 + coeffs[1]);

      // The idea here is to constraint the below values to be 0.
      // NOTE: The use of `AD<double>` and use of `CppAD`!
      // This is also CppAD can compute derivatives and pass
      // these to the solver.
      fg[2 + x_start+t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start+t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start+t] = psi1 - (psi0 + (v0/Lf) * delta0 * dt);
      fg[2 + v_start+t] = v1 - (v0 + a0 * dt);
      fg[2 + cte_start+t] = cte1 - ((f0-y0) + v0 * CppAD::sin(epsi0) * dt);
      fg[2 + epsi_start+t] = epsi1 - ((psi0-psides0) + (v0/Lf) * delta0 * dt);
    }

  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {

  bool ok = true;
  //size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // TO_DID: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 6 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 10*6 + 9*2
  size_t n_vars = N * 6 + (N-1) * 2;
  // TO_DID: Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  
  /*
  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;
  */

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // TO_DID: Set lower and upper limits for variables.

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper & lower limits of delta are set to -25 and 25 deg (in radians)
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

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
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TO_DID: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> result;

  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);

  for (int i=0; i < N; i++) {
    result.push_back(solution.x[x_start + i]);
    result.push_back(solution.x[y_start + i]);
  }

  return result;
}
