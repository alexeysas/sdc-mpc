#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

// Define global constants
#define latency  0.1
#define Lf 2.67
#define MhpToMs 0.44704 

// Defime model tuning parameters
#define N_Points 10
#define dt 0.1

#define SPEED_GOAL 70

//Define Cost penalties
#define CTE_PENALTY  1
#define EPSI_PENALTY  10
#define SPEED_PENALTY  2

#define STEER_PENALTY  0
#define A_PENALTY  0

#define STEER_CHANGE_PENALTY  10000
#define A_CHANGE_PENALTY  200

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
