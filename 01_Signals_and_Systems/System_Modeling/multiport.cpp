// by Chester Zelaya

#include "multiport.h"

powerPair addPowerPair(domain d) {
  switch (d) {
  case domain::mechanicalTranslation:
    return {/* name */ "",
            /* force */ 0.0, /* velocity */ 0.0,
            domain::mechanicalTranslation, ioDirection::input};
  case domain::mechanicalRotation:
    return {/* name */ "",
            /* torque */ 0.0, /* angular velocity */ 0.0,
            domain::mechanicalRotation, ioDirection::input};
  case domain::hydraulic:
    return {/* name */ "",
            /* pressure */ 0.0, /* flow rate */ 0.0,
            domain::hydraulic, ioDirection::input};
  case domain::electric:
    return {/* name */ "",
            /* voltage */ 0.0, /* current */ 0.0,
            domain::electric, ioDirection::input};
  }
  return {"", 0.0, 0.0, domain::electric, ioDirection::input};
}

double getPower(powerPair pp) { return pp.effort * pp.flow; }

constexpr energyPair findEnergy(const powerPair &power,
                                const energyPair &initial, double t0,
                                double tf) {
  const double dt = tf - t0;
  return {initial.momentum + power.effort * dt,
          initial.displacement + power.flow * dt, initial.direction};
}

double getEnergy(energyPair ep) { return ep.displacement * ep.momentum; }
