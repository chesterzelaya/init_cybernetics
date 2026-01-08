// by Chester Zelaya

#include "discrete_feasible_planning.h"
#include "Eigen/Core"

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

/* General template of search algorithms, there are three different type
 * of states at play here. We have: Univisited states, Dead states, and Alive
 * states.
 * Particular flavors of forward search:
 * Breadth First:  First in First out for uniformally expanding forward
 * Depth First: Last in First out - for exploring long paths early
 * Dijkstra: Implements a cost function l(x,u) and sorts the queue according to
 * function.
 * A*: Similar to Dijkstra, but different in the way that summed with the cost
 * function l(x,u) measuring the effort of going from one state to another, we
 * have an additional underestimate G* cost function to the goal from a given
 * state. If G*(x) = 0, it defaults to Dijkstra's algorithm
 * Best First: Chooses locally optimal cost-to-go
 * Iterative Deepening: Use depth-first search to find all states of distance i
 * or less from x1. If goal not found, discard previous work, and find all
 * states of ditance i+1
 */
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

/* Other general search schemes
 * Backward search: start at the goal state, and compute the preceeding state
 * and action such that x' = f(x,u)
 * Bidirectional search: Forward search and backward search running
 * simulatnously until convergence
 */
bool backwardSearch(const stateSpace::State &start,
                    const stateSpace::State &goal, const stateSpace &S,
                    const actionSpace &A) {

  std::queue<Eigen::VectorXd> queue;
  std::vector<Eigen::VectorXd> visited;

  queue.push(goal.x);
  visited.push_back(goal.x);

  const Eigen::MatrixXd actions = A.getActions();

  while (!queue.empty()) {
    const Eigen::VectorXd current = queue.front();
    queue.pop();

    if (current.isApprox(start.x)) {
      return true;
    }

    for (int col = 0; col < actions.cols(); ++col) {
      const Eigen::VectorXd action = actions.col(col);
      const Eigen::VectorXd inverse_action =
          -action; // Note inverse action here is simply the negative. A
                   // f_inv(x, u) must be computed for more complex search
                   // environments
      const Eigen::VectorXd prev = S.computeNextState(current, inverse_action);

      if (!stateVisited(visited, prev)) {
        visited.push_back(prev);
        queue.push(prev);
      } else {
        // Duplicate state -- solution not entirely specified rn
      }
    }
  }

  return false;
}

bool bidirectionalSearch(const stateSpace::State &start,
                         const stateSpace::State &goal, const stateSpace &S,
                         const actionSpace &A) {
  std::queue<Eigen::VectorXd> queue_forward;
  std::vector<Eigen::VectorXd> visited_forward;

  queue_forward.push(start.x);
  visited_forward.push_back(start.x);

  std::queue<Eigen::VectorXd> queue_backward;
  std::vector<Eigen::VectorXd> visited_backward;

  queue_backward.push(goal.x);
  visited_backward.push_back(goal.x);

  const Eigen::MatrixXd actions = A.getActions();

  while (!queue_forward.empty() && !queue_backward.empty()) {
    if (!queue_forward.empty()) {
      const Eigen::VectorXd current_forward = queue_forward.front();
      queue_forward.pop();

      if (stateVisited(visited_backward, current_forward)) {
        return true;
      }
      for (int col = 0; col < actions.cols(); ++col) {
        const Eigen::VectorXd action = actions.col(col);
        const Eigen::VectorXd next_forward =
            S.computeNextState(current_forward, action);
        if (stateVisited(visited_backward, next_forward)) {
          return true;
        }
        if (!stateVisited(visited_forward, next_forward)) {
          visited_forward.push_back(next_forward);
          queue_forward.push(next_forward);
        } else {
          // Duplicate state -- solution not entirely specified rn
        }
      }
    }

    if (!queue_backward.empty()) {
      const Eigen::VectorXd current_backward = queue_backward.front();
      queue_backward.pop();

      if (stateVisited(visited_forward, current_backward)) {
        return true;
      }
      for (int col = 0; col < actions.cols(); ++col) {
        const Eigen::VectorXd inverse_action = actions.col(col) * -1;
        const Eigen::VectorXd next_backward =
            S.computeNextState(current_backward, inverse_action);
        if (stateVisited(visited_forward, next_backward)) {
          return true;
        }
        if (!stateVisited(visited_backward, next_backward)) {
          visited_backward.push_back(next_backward);
          queue_backward.push(next_backward);
        } else {
          // Duplicate state -- solution not entirely specified rn
        }
      }
    }
  }
  return false;
}

/* A unified view of search methods --
 * All of the planning methods in the secton follow the same basic template
 * 1. Initialization: Let the search graph G(V,E) be initialized with E empty
 * and V containing some starting states. For forward search V = {x1} and for
 * backward search V{xg}. If bidirectional V{x1,xg}
 * 2. Select Vetex: Choose a vertex for expansion -- usually accompliished by
 * maintaining a priority queue.
 * 3. Apply an action: An xnew state could arise from f(x,u)
 * 4. Insert a Directed Edge into the Graph: If certain algorithm specific tests
 * are passed, then generate an edge fro x to xnew.
 * 5. Check for solution: Check wether G encodes a path from x1 to xg
 * 6. Return to step 2: Iterate unless a solutoin has been found
 */
