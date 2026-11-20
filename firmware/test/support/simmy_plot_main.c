/* Host-only, synthetic Mode A simulation, not bench-validated.
 * Python validates numeric arguments. Compile with the real Control.c and
 * simmy sources; no HAL, firmware main(), or alternate controller copy.
 */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Control.h"
#include "simmy.h"

typedef struct {
  int enabled;
  int64_t velocityQ8;
} Reference;

/* EXPERIMENTAL Mode A feedforward: only this tick's counted pulses and past
 * state, never the analytic pulse rate, future positions, or plant truth.
 */
static int32_t ControlAdapter(void *user, int32_t target, int32_t position,
                              int32_t velocity, uint32_t cycles) {
  const Reference *reference = user;
  if (reference->enabled)
    return ControlTickWithReferenceQ8(target, position, velocity, reference->velocityQ8, cycles);
  return ControlTick(target, position, velocity, cycles);
}

/* Fixed 1 s cruises and 2 s holds; integer boundaries avoid floating-point
 * phase ambiguity. Never reset controller, plant, or pulse phase at a stop.
 */
static double SequenceRate(int tick, int rampTicks, double rate, const char **phase) {
  const int legTicks = 2 * rampTicks + 3000;
  const int reverse = tick >= legTicks;
  int local = reverse ? tick - legTicks : tick;
  double direction = reverse ? -1.0 : 1.0;
  if (local < rampTicks) {
    *phase = reverse ? "reverse-accel" : "forward-accel";
    return direction * rate * local / rampTicks;
  }
  local -= rampTicks;
  if (local < 1000) {
    *phase = reverse ? "reverse-cruise" : "forward-cruise";
    return direction * rate;
  }
  local -= 1000;
  if (local < rampTicks) {
    *phase = reverse ? "reverse-decel" : "forward-decel";
    return direction * rate * (1.0 - (double)local / rampTicks);
  }
  *phase = reverse ? "final-hold" : "forward-hold";
  return 0;
}

int main(int argc, char **argv) {
  if (argc != 14) {
    fprintf(stderr, "usage: %s config ticks rate ramp_seconds jitter pos_kp vel_kp vel_ki filter_shift scenario feedforward ff_shift accel_ff_q8\n", argv[0]);
    return 1;
  }
  const int ticks = atoi(argv[2]);
  const double rate = atof(argv[3]), ramp = atof(argv[4]), jitter = atof(argv[5]);
  const int sequence = strcmp(argv[10], "sequence") == 0;
  const int rampTicks = (int)llround(ramp * 1000);
  if (sequence && rampTicks < 1) {
    fprintf(stderr, "sequence requires a ramp of at least 1 ms\n");
    return 1;
  }
  const int ffEnabled = strcmp(argv[11], "none") != 0;
  const int accelerationEnabled = strcmp(argv[11], "velocity-acceleration") == 0;
  const int ffShift = atoi(argv[12]);
  const ControlGains gains = {
    .posGainQ8 = atoi(argv[6]), .velGainKpQ8 = atoi(argv[7]),
    .velGainKiQ8 = atoi(argv[8]), .velocityClamp = 200000,
    .torqueClampMin = -1000, .torqueClampMax = 1000,
    .velocityFilterShift = atoi(argv[9]),
    .accelerationFeedforwardQ8 = accelerationEnabled ? atoi(argv[13]) : 0,
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
  Reference reference = {.enabled = ffEnabled, .velocityQ8 = 0};
  puts("[");
  for (int i = 0; i < ticks; ++i) {
    const double t = i / 1000.0;
    const char *phase = ramp > t ? "accel" : "cruise";
    const double currentRate = sequence ? SequenceRate(i, rampTicks, rate, &phase)
        : (ramp > 0 ? rate * fmin(t / ramp, 1.0) : rate);
    const int32_t delta = SimmyPulseTrainTickDelta(&pulses, currentRate, 1000.0, jitter);
    target += delta;
    if (ffEnabled)
      reference.velocityQ8 += ((int64_t)delta * 256 - reference.velocityQ8) / (1 << ffShift);
    const int32_t position = SimmyChainFeedbackPositionCounts(&state, &params);
    const int32_t velocity = SimmyChainFeedbackVelocityCounts(&state, &params);
    const int32_t torque = controller(&reference, target, position, velocity, 32000);
    /* Sample feedback BEFORE advancing the plant: no one-tick look-ahead.
     * cmd_velocity is the controller's cascade target, not pulse rate.
     * Fault.c is not simulated; fault_latched=false is a renderer placeholder.
     */
    printf("%s{\"t\":%.6f,\"cmd_position\":%" PRId32
           ",\"position\":%" PRId32 ",\"cmd_velocity\":%" PRId32
           ",\"velocity\":%" PRId32 ",\"torque\":%" PRId32
           ",\"fault_latched\":false,\"load_position\":%" PRId32
           ",\"load_velocity\":%" PRId32 ",\"phase\":\"%s\",\"target_rate\":%.9f"
           ",\"pulse_delta\":%" PRId32 ",\"reference_velocity_q8\":%" PRId64 "}\n",
           i ? "," : "", t, target, position, ControlGetLastVelocityTarget(),
           velocity, torque,
           SimmyChainBodyPositionCounts(&state, params.bodyCount - 1),
           SimmyChainBodyVelocityCounts(&state, params.bodyCount - 1), phase, currentRate,
           delta, reference.velocityQ8);
    const double force = SimmyAnalogChainStep(&analogState, &analog, torque);
    SimmyChainStep(&state, &params, force);
  }
  puts("]");
  return 0;
}
