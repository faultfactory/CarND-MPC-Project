#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using CppAD::AD;

// TODO: Set the timestep length and duration
static size_t N = 10;
double dt = 0.1;

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

// Both the reference cross track and orientation errors are 0.
// The reference velocity is set to 40 mph.
double ref_v = 70;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }



  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
   // The cost is stored is the first element of `fg`.
   
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;
    
    // The part of the cost based on the reference state.
    
    for (int t = 0; t < N; t++) {
      fg[0] += 2*CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 2* CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
      std::cout<<"Velocity"<< AD<double>(vars[v_start+t])<<std::endl;
    }
    
    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += 1*CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 1*CppAD::pow(vars[a_start + t], 2);
    }
    
    // Penalize paths with small curvature to trend the system toward the racing line
    // and reduce aggressive values for delta, especially in the beginning of the path
    // Radius of curvature: 
    // https://www.intmath.com/applications-differentiation/8-radius-curvature.php
    // Rc = ([1+(dy/dx)^2]^(3/2))/abs(d2y/dx^2)

    // Using CppAd instructions from: 
    // https://coin-or.github.io/CppAD/doc/get_started.cpp.htm

    // I have to generate a polynomial of the model-predicted candidate path from which to
    // compute curvature.
    Eigen::VectorXd x_mpc;
    Eigen::VectorXd y_mpc;

    for (int t=0;t<N;t++){
      x_mpc<<(vars[x_start+t]);
      y_mpc(vars[y_start+t]);
    };
    
    // assert(x_mpc.size() == y_mpc.size());
    // assert(3 >= 1 && 3 <= x_mpc.size() - 1);
    // Eigen::MatrixXd A(x_mpc.size(), 3 + 1);

    // for (int i = 0; i < x_mpc.size(); i++) {
    //   A(i, 0) = 1.0;
    // }

    // for (int j = 0; j < x_mpc.size(); j++) {
    //   for (int i = 0; i < 3; i++) {
    //     A(j, i + 1) = A(j, i) * x_mpc(j);
    //   }
    // }

    // auto Q = A.householderQr();
    // auto result = Q.solve(y_mpc);

    // //CppAD::Poly wants to see coeffs as a simple vector
    // ADvector f;
    
    // for(int i=0;i<coeffs.size();i++){
    //   f.push_back(AD<double>(coeffs[i]));
    // }
    


    // //std::cout<<"f: "<<f<<std::endl;
    // for(int t = 0; t<N; t++){
    //   // compute value of derivative at this location
    //   AD<double> dydx = CppAD::Poly(size_t(1),f,AD<double>(1.0*t));
    //   // compute the value of the 2nd derivative at this point 

    //   AD<double> d2ydx2 = CppAD::Poly(size_t(2),f,AD<double>(1.0*t));
    //   if(abs(d2ydx2)<0.001){d2ydx2=AD<double>(0.001);}
    //   AD<double> rc = CppAD::pow((1+dydx*dydx),(3.0/2.0))/abs(d2ydx2);
    //   //std::cout<<"dydx: "<<dydx<<" d2ydx2: "<<d2ydx2<<" RC: "<<rc<<std::endl;
    //   if(abs(rc)<0.001){rc=AD<double>(0.001);}
    //   std::cout<<"Curvature cost "<< 100*CppAD::pow(1.0/rc, 2.0)<<std::endl;
    //   //fg[0] += 10000*CppAD::pow(1.0/rc, 2.0);
    // };

    


    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += 1*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    //
    // Setup Constraints
    //
    // NOTE: In this section you'll setup the model constraints.

    // Initial constraints
    //
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
    for (int t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] *pow(x0,2) + coeffs[3]*pow(x0,3);
      AD<double> psides0 = CppAD::atan(coeffs[1]+2*coeffs[2]*x0+3*coeffs[3]*x0*x0);

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

CppAD::ipopt::solve_result<CPPAD_TESTVECTOR(double)> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;
 
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // number of independent variables
  // N timesteps == N - 1 actuations
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
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
  // NOTE: You don't have to worry about  these options
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
  for(int i = delta_start; i< a_start;i++){
  }


  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
    return solution; 
};

MPC_Output::MPC_Output(){
  this->n = N;
}

void MPC_Output::fill(CppAD::ipopt::solve_result<CPPAD_TESTVECTOR(double)> sol){
  bool verbose = false;

  if(verbose){std::cout<<"X:"<<std::endl;}
  for(int i = 0;    i<n;    i++){this->X.push_back(sol.x[i]);
  if(verbose){std::cout<<sol.x[i]<<std::endl;}
  };

  if(verbose){std::cout<<"Y:"<<std::endl;}
  for(int i = n;    i<2*n;  i++){this->Y.push_back(sol.x[i]);
  if(verbose){std::cout<<sol.x[i]<<std::endl;}
  };

  if(verbose){std::cout<<"PSI:"<<std::endl;}
  for(int i = 2*n;  i<3*n;  i++){this->PSI.push_back(sol.x[i]);
  if(verbose){std::cout<<sol.x[i]<<std::endl;}
  };

  if(verbose){std::cout<<"V:"<<std::endl;}
  for(int i = 3*n;  i<4*n;  i++){this->V.push_back(sol.x[i]);
  if(verbose){std::cout<<sol.x[i]<<std::endl;}
  };

  if(verbose){std::cout<<"CTE:"<<std::endl;}
  for(int i = 4*n;  i<5*n;  i++){this->CTE.push_back(sol.x[i]);
  if(verbose){std::cout<<sol.x[i]<<std::endl;}
  };
  
  if(verbose){std::cout<<"EPSI:"<<std::endl;}
  for(int i = 5*n;  i<(6*n);i++){this->EPSI.push_back(sol.x[i]);
  if(verbose){std::cout<<sol.x[i]<<std::endl;}
  };
  
  if(verbose){std::cout<<"Delta:"<<std::endl;}
  for(int i = (6*n);i<(7*n-1);i++){this->DELTA.push_back(sol.x[i]);
  if(verbose){std::cout<<sol.x[i]<<std::endl;}
  };

  if(verbose){std::cout<<"A:"<<std::endl;}
  for(int i = (7*n-1);i<(8*n-2);i++){this->A.push_back(sol.x[i]);
  if(verbose){std::cout<<sol.x[i]<<std::endl;}
  };

  if(verbose){std::cout<<std::endl<<std::endl;}
}
