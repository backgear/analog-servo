/* Host-only, synthetic Mode A simulation, not bench-validated.
 * Python validates numeric arguments. Compile with the real Control.c and
 * simmy sources; no HAL, firmware main(), or alternate controller copy.
 */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "Control.h"
#include "simmy.h"

static int32_t ControlAdapter(void *user, int32_t target, int32_t position,
                              int32_t velocity, uint32_t cycles) {
  (void)user;
  return ControlTick(target, position, velocity, cycles);
}

int main(int argc, char **argv) {
  if (argc != 10) {
    fprintf(stderr, "usage: %s config ticks rate ramp_seconds jitter pos_kp vel_kp vel_ki filter_shift\n", argv[0]);
    return 1;
  }
  const int ticks = atoi(argv[2]);
  const double rate = atof(argv[3]), ramp = atof(argv[4]), jitter = atof(argv[5]);
  const ControlGains gains = {
    .posGainQ8 = atoi(argv[6]), .velGainKpQ8 = atoi(argv[7]),
    .velGainKiQ8 = atoi(argv[8]), .velocityClamp = 200000,
    .torqueClampMin = -1000, .torqueClampMax = 1000,
    .velocityFilterShift = atoi(argv[9]),
  };
  SimmyChainParams params;
  SimmyAnalogChainParams analog;
  char err[256];
  if (SimmyChainParamsFromJsonFile(argv[1], &params, err, sizeof(err)) ||
      SimmyAnalogChainParamsFromJsonFile(argv[1], &analog, err, sizeof(err))) {
    fprintf(stderr, "%s: %s\n", argv[1], err);
    return 1;
  }
  SimmyChainState state;
  SimmyAnalogChainState analogState;
  SimmyPulseTrainState pulses;
  SimmyChainInit(&state, &params);
  SimmyAnalogChainInit(&analogState, 1);
  SimmyPulseTrainInit(&pulses, 12345);
  ControlInit(&gains);
  SimmyControllerFn controller = ControlAdapter;
  int32_t target = 0;
  puts("[");
  for (int i = 0; i < ticks; ++i) {
    const double t = i / 1000.0;
    const double currentRate = ramp > 0 ? rate * fmin(t / ramp, 1.0) : rate;
    target += SimmyPulseTrainTickDelta(&pulses, currentRate, 1000.0, jitter);
    const int32_t position = SimmyChainFeedbackPositionCounts(&state, &params);
    const int32_t velocity = SimmyChainFeedbackVelocityCounts(&state, &params);
    const int32_t torque = controller(NULL, target, position, velocity, 32000);
    /* Sample feedback BEFORE advancing the plant: no one-tick look-ahead.
     * cmd_velocity is the controller's cascade target, not pulse rate.
     * Fault.c is not simulated; fault_latched=false is a renderer placeholder.
     */
    printf("%s{\"t\":%.6f,\"cmd_position\":%" PRId32
           ",\"position\":%" PRId32 ",\"cmd_velocity\":%" PRId32
           ",\"velocity\":%" PRId32 ",\"torque\":%" PRId32
           ",\"fault_latched\":false,\"load_position\":%" PRId32
           ",\"load_velocity\":%" PRId32 "}\n",
           i ? "," : "", t, target, position, ControlGetLastVelocityTarget(),
           velocity, torque,
           SimmyChainBodyPositionCounts(&state, params.bodyCount - 1),
           SimmyChainBodyVelocityCounts(&state, params.bodyCount - 1));
    const double force = SimmyAnalogChainStep(&analogState, &analog, torque);
    SimmyChainStep(&state, &params, force);
  }
  puts("]");
  return 0;
}
