#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  CppAD::ipopt::solve_result<CPPAD_TESTVECTOR(double)> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};


// define a class that takes the solution and disseminates the information for each field
class MPC_Output{
  public:
  MPC_Output();
  ~MPC_Output(){};
  void fill(CppAD::ipopt::solve_result<CPPAD_TESTVECTOR(double)>);
  size_t n; 
  vector<double> X; 
  vector<double> Y; 
  vector<double> PSI; 
  vector<double> V; 
  vector<double> CTE; 
  vector<double> EPSI;
  vector<double> DELTA;
  vector<double> A;
};

#endif /* MPC_H */

