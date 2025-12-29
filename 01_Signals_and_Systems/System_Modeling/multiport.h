// by Chester Zelaya

#include <string>
#include <vector>

/* Multiports in system modeling work under the assumption
 * the interconnected subsystems have power
 * that flow between themselves. The terminology effort
 * and flow are used to describe the power variables
 */

/* @brief Power table shown for different domains
 *  in terms of their effort and flow
 */

enum class domain {
  mechanicalTranslation,
  mechanicalRotation,
  hydraulic,
  electric
};

enum class ioDirection {
  input,
  output
};

struct powerPair {
  std::string name;
  double effort;
  double flow;
  domain domain;
  // Flow direction is the opposite of effort_direction.
  ioDirection effort_direction;
};

powerPair addPowerPair(domain d);

/* @brief Power calculated by multiplying effort with flow */
double getPower(powerPair pp);

/* @brief Momentum and dispalcement are important variables
 * in being able to describe a system. They are called energy variables
 * and defined by the integral of effor or flow
 *
 * The integral definitions can also be stated in differential form:
 * dp/dt = e(t) and dq/dt = f(t), where p is momentum and q is displacement.
 */

struct energyPair {
  double momentum;
  double displacement;
  ioDirection direction;
};

/* @brief Active bond for control input (single signal, not a power pair) */
struct activeBond {
  double control;
  ioDirection direction;
};

/* @brief Subsystem groups multiple ports, energies, and control bonds */
struct subsystem {
  std::string name;
  std::vector<powerPair> ports;
  std::vector<energyPair> energies;
  std::vector<activeBond> controls;
};

/* @brief Energy variables found by the integral of power with respect to time
 */
constexpr energyPair findEnergy(const powerPair &power,
                                const energyPair &initial, double t0,
                                double tf);
/* @brief Total energy found by multiplying momentum and displacement together
 */
double getEnergy(energyPair ep);
