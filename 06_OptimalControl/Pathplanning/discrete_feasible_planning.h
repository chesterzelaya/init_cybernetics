// by Chester Zelaya

#include "Eigen/Core"
#include <Eigen/Dense>

/* The world may be transformed through the application of actions
 * that are chosen by a planner. Each action u when applied to current
 * state x produces a new state x' */

class stateSpace {
public:
  struct State {
    Eigen::VectorXd x;
    bool is_init;
    bool is_goal;
    bool is_seen;
  };

  Eigen::MatrixXd states_; // Collection of States
  Eigen::MatrixXd getStates() const;
  void addState(State X); // Add state, matrix of measurable
                          // variables, can determine if a state is initial
                          // state of goal state

  Eigen::VectorXd computeNextState(Eigen::MatrixXd curr_state,
                                   Eigen::MatrixXd actions) const;
};

class actionSpace {
public:
  Eigen::MatrixXd actions_;
  Eigen::MatrixXd getActions() const; // Call to get actions in actionspace
  void addActions(Eigen::VectorXd actions,
                  Eigen::VectorXd state); // Add actions based on state (not all
                                          // necessarily disjoint)
};

bool forwardSearch(const stateSpace::State &start,
                   const stateSpace::State &goal, const stateSpace &S,
                   const actionSpace &A);

bool backwardSearch(const stateSpace::State &start,
                    const stateSpace::State &goal, const stateSpace &S,
                    const actionSpace &A);
