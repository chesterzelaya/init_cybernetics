// by Chester Zelaya

#include "discrete_feasible_planning.h"

#include <queue>
#include <vector>

static bool stateVisited(const std::vector<Eigen::VectorXd> &visited,
                         const Eigen::VectorXd &candidate) {
  for (const auto &seen : visited) {
    if (seen.isApprox(candidate)) {
      return true;
    }
  }
  return false;
}

bool forwardSearch(const stateSpace::State &start,
                   const stateSpace::State &goal, const stateSpace &S,
                   const actionSpace &A) {
  std::queue<Eigen::VectorXd> queue;
  std::vector<Eigen::VectorXd> visited;

  queue.push(start.x);
  visited.push_back(start.x);

  const Eigen::MatrixXd actions = A.getActions();

  while (!queue.empty()) {
    const Eigen::VectorXd current = queue.front();
    queue.pop();

    if (current.isApprox(goal.x)) {
      return true;
    }

    for (int col = 0; col < actions.cols(); ++col) {
      const Eigen::VectorXd action = actions.col(col);
      const Eigen::VectorXd next = S.computeNextState(current, action);

      if (!stateVisited(visited, next)) {
        visited.push_back(next);
        queue.push(next);
      } else {
        // Duplicate state -- solution not entirely specified rn
      }
    }
  }

  return false;
}
