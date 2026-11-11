#ifndef CONFIGSTORE_H
#define CONFIGSTORE_H

#include <stdint.h>

/* Flash-backed config persistence, single page, no ping-pong: magic+CRC32+
 * version, erase-then-program-with-readback, reject-on-mismatch, no
 * migration -- an old-version record is simply rejected and firmware
 * defaults apply. See AGENTS.md's ConfigStore convention. */

/* Default gains -- named (not bare literals in ConfigDataSetDefaults()) so a
 * native test can check them against Control.c's cascade without linking
 * ConfigStore.c's flash-dependent code. Deliberately weak (a quarter of
 * "unity," 256==1.0 in Q8.8): unity gains saturate the clamps almost
 * immediately for a modest position error; gains chosen too far below unity
 * quantize away to near-zero through the cascade's chained integer-
 * truncating fixed-point math. See test_control.c's
 * test_default_gains_avoid_saturation_and_quantization_deadzone. */
#define CONFIG_DEFAULT_POS_GAIN_Q8    64   /* 0.25 in Q8.8 */
#define CONFIG_DEFAULT_VEL_GAIN_KP_Q8 64   /* 0.25 in Q8.8 */
#define CONFIG_DEFAULT_VEL_GAIN_KI_Q8 0    /* integral action off until the proportional
                                               path is validated -- see ConfigStore.c */

/* Mode C (analog velocity) scale/hysteresis -- see Analog.h for the math,
 * Adc.h/gpio.h for the ADC pin + reverse pin these feed. Deliberately weak
 * and NOT bench-tuned -- the scale depends entirely on the real ADC
 * reference voltage and pot/source wiring. */
#define CONFIG_DEFAULT_ANALOG_SCALE_Q8        16   /* 0.0625 in Q8.8 -- max raw ADC code
                                                       (4095) maps to ~256 counts/tick */
#define CONFIG_DEFAULT_ANALOG_HYSTERESIS_COUNTS 8  /* raw ADC LSBs, ~0.2% of full scale */

/* FAULT_SUSTAINED_CURRENT config -- see Fault.h. Deliberately conservative
 * starting numbers, not bench-tuned: 10% of full scale (computed inline in
 * ConfigDataSetDefaults() from TORQUE_FULL_SCALE) for ~2s. minTorque <= 0
 * disables the check entirely. */
#define CONFIG_DEFAULT_SUSTAINED_CURRENT_WINDOW_TICKS 2000

/* Mode B's velocity ramp rate (Ramp.c's RampSetAcceleration) -- also governs
 * PositionMove.c's deceleration planning (`position.command`). Conservative
 * tier-1 default -- tune on the bench. */
#define CONFIG_DEFAULT_RAMP_ACCEL 1000

/* Soft proportional noise-band width; see Control.h. 0 disables shaping.
 * The full error always reaches I, including inside this band. */
#define CONFIG_DEFAULT_VELOCITY_ERROR_DEADBAND_COUNTS 0

/* output.topology=two_phase_analog placeholders (`two-phase-output` draft-PR
 * branch) -- pure guesses, NOT derived from any real motor, until
 * `commutation detect_poles` actually runs. See docs/two-phase-drive.md. */
#define CONFIG_DEFAULT_COMMUTATION_POLE_PAIRS 4
#define CONFIG_DEFAULT_COMMUTATION_COUNTS_PER_ELEC_REV 1000

typedef struct {
  uint8_t servoMode;                 /* 0 = Mode A (step/dir), 1 = Mode B (velocity
                                         command), 2 = Mode C (analog velocity) */
  uint8_t enabled;                    /* persisted axis.enabled -- restored at boot
                                          (main(), NOT `config load` at runtime) so a
                                          headless Mode C setup can come up armed with
                                          no CLI interaction; actual motion still waits
                                          on the hardware ENABLE pin. */
  uint8_t followingErrorHardFault;   /* see Fault.h */
  int32_t posGainQ8, velGainKpQ8, velGainKiQ8;
  int32_t torqueClampMin, torqueClampMax, velocityClamp;
  int32_t positionMoveFeedforwardQ8; /* 0..256; velocity/acceleration FF scale for point moves */
  int32_t velocityFilterShift;       /* P feedback only: 0 raw; 1..4 IIR alpha=2^-shift.
                                          I and encoder position remain unfiltered. */
  int32_t accelerationFeedforwardQ8;  /* torque / (counts/tick^2), Q8; 0 disables */
  int32_t velocityJerk;              /* counts/tick/sec^2; 0 retains trapezoidal jogs */
  int32_t commandVelocityClamp;      /* 0 follows velocityClamp; otherwise trajectory speed cap */
  int32_t followingErrorLimitCounts;
  uint8_t polarityLearned;
  int8_t  polaritySign;               /* +1 or -1, applied before OutputSetTorqueCommand */
  int32_t zeroTrim;                   /* signed offset, raw torque units, added to the
                                          commanded torque before it's scaled/split across
                                          the positive/negative channels, to correct residual
                                          analog-stage zero error; defaults to 0, tune on the
                                          bench -- see Output.h's OutputSetZeroTrim() */
  int32_t analogVelocityScaleQ8;        /* Mode C: raw ADC code -> velocity counts/tick,
                                            Q8.8 -- see Analog.h */
  int32_t analogVelocityHysteresisCounts; /* Mode C: raw ADC LSBs of hysteresis, see
                                              Analog.h */
  int32_t sustainedCurrentMinTorque;    /* FAULT_SUSTAINED_CURRENT threshold -- see Fault.h.
                                            <= 0 disables the check entirely. */
  int32_t sustainedCurrentWindowTicks;  /* FAULT_SUSTAINED_CURRENT duration, ticks @ 1kHz */
  int32_t rampAccelCountsPerTickPerSec; /* Mode B jog ramp + PositionMove.c's deceleration
                                            planning -- see CONFIG_DEFAULT_RAMP_ACCEL above */
  int32_t velocityErrorDeadbandCounts;  /* see Control.h's ControlGains field of the same name */
  uint8_t outputTopology;               /* see Output.h's OutputTopology -- added on the
                                            `two-phase-output` draft-PR branch */
  uint8_t commutationLearned;
  int32_t commutationPolePairs;          /* placeholder default until `commutation
                                             detect_poles` has actually run, see
                                             ConfigStore.c */
  int32_t commutationCountsPerElecRev;   /* ditto */
  int32_t commutationOffsetDeg;          /* 0 until a successful `commutation learn` */
} ConfigData;

/* Erase + program + readback-verify. Returns 0 on success, -1 on failure. */
int ConfigStoreSave(const ConfigData *cfg);

/* Validates magic+CRC32+version; -1 if absent/corrupt (leaves *out
 * untouched), 0 on success. No flash unlock needed -- this is a plain memory
 * read. */
int ConfigStoreLoad(ConfigData *out);

/* Page erase only, no programming. */
void ConfigStoreErase(void);

/* Firmware defaults, used when no valid flash record exists yet. */
void ConfigDataSetDefaults(ConfigData *cfg);

#endif /* CONFIGSTORE_H */
