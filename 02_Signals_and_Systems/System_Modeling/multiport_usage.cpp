#include "multiport.h"

struct DCMotorPorts {
  powerPair armature;
  powerPair field;
  powerPair mechanical;
};

int main() {
  // Inputs (effort, flow) for each port.
  const powerPair armature_in{/* ea */ 24.0, /* ia */ 2.0,
                              domain::electric, ioDirection::input};
  const powerPair field_in{/* ef */ 5.0, /* if */ 0.5, domain::electric,
                           ioDirection::input};

  // Outputs (effort, flow) for the mechanical port.
  const powerPair mechanical_out{/* tau */ 0.24, /* omega */ 180.0,
                                 domain::mechanicalRotation,
                                 ioDirection::output};

  const DCMotorPorts ports{armature_in, field_in, mechanical_out};
  (void)ports;
  return 0;
}
