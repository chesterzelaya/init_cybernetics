#include "multiport.h"

int main() {
  // DC motor subsystem with two electrical inputs and one mechanical output.
  subsystem dc_motor{
      "DC Motor",
      {
          {/* name */ "armature", /* ea */ 24.0, /* ia */ 2.0,
           domain::electric, ioDirection::input},
          {/* name */ "field", /* ef */ 5.0, /* if */ 0.5,
           domain::electric, ioDirection::input},
          {/* name */ "shaft", /* tau */ 0.24, /* omega */ 180.0,
           domain::mechanicalRotation, ioDirection::output},
      },
      {},
      {
          {/* throttle */ 0.6, ioDirection::input},
      },
  };

  // Simple system: motor -> shaft -> gears -> pedal load.
  subsystem shaft{
      "Shaft",
      {
          {/* name */ "motor_in", /* tau_3 */ 0.24, /* w_3 */ 180.0,
           domain::mechanicalRotation, ioDirection::input},
          {/* name */ "gear_out", /* tau_4 */ 0.22, /* w_4 */ 160.0,
           domain::mechanicalRotation, ioDirection::output},
      },
      {},
      {},
  };

  subsystem gears{
      "Gears",
      {
          {/* name */ "shaft_in", /* tau_4 */ 0.22, /* w_4 */ 160.0,
           domain::mechanicalRotation, ioDirection::input},
          {/* name */ "load_out", /* tau_5 */ 0.40, /* w_5 */ 90.0,
           domain::mechanicalRotation, ioDirection::output},
      },
      {},
      {},
  };

  subsystem pedal_load{
      "Pedal Load",
      {
          {/* name */ "gear_in", /* tau_5 */ 0.40, /* w_5 */ 90.0,
           domain::mechanicalRotation, ioDirection::input},
      },
      {},
      {},
  };

  (void)dc_motor;
  (void)shaft;
  (void)gears;
  (void)pedal_load;
  return 0;
}
