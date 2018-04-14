#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<vector<double>> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, vector< vector<double> > closest_prediction, string state_mpc, double delta_lim);
};

#endif /* MPC_H */
