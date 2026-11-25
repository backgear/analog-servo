#ifndef COMMUTATION_H
#define COMMUTATION_H

#include <stdint.h>

/* Electrical-angle math and Ia/Ib sine synthesis for
 * output.topology=two_phase_analog -- see docs/two-phase-drive.md's "New
 * commutation-angle stage". Pure fixed-point, no HAL, same convention as
 * Control.c/Autotune.c. Angles are plain integer degrees, wrapped to
 * 0..359 by every function here that returns or consumes one -- no radians,
 * no floating point, matching this project's existing fixed-point style. */

/* Hall sector (0..5, see Hall.h) -> coarse electrical angle, one of
 * {0,60,...,300}. This is the pre-Z-index angle source (see
 * docs/two-phase-drive.md) -- deliberately NOT offset-corrected: Hall-based
 * commutation is only ever used while the alignment offset is either being
 * learned (CommutationLearn* below) or not yet trusted, and 60-degree
 * coarseness is already the dominant error source at that point. */
int32_t CommutationElectricalAngleFromHallDeg(int hallSector);

/* Continuous electrical angle from the encoder's own position counter --
 * the post-Z-index angle source (see docs/two-phase-drive.md: a live
 * register read, no polling-rate-dependent staleness). countsPerElecRev is
 * the number of encoder counts per one electrical revolution
 * (encoder CPR / pole_pairs, see commutation.pole_pairs /
 * commutation.counts_per_elec_rev in Protocol.c) -- must be > 0, callers
 * guard this (0 is rejected at the CLI, see setCommutationCountsPerElecRev).
 * offsetDeg is the learned alignment offset (CommutationLearnEvaluate's
 * result, persisted via ConfigStore) -- 0 until a successful
 * `commutation learn` has run. */
int32_t CommutationElectricalAngleFromEncoderDeg(int32_t encoderPositionCounts,
                                                  int32_t countsPerElecRev,
                                                  int32_t offsetDeg);

/* Given a signed torque magnitude (+-TORQUE_FULL_SCALE, same scale as
 * Control.c's torque clamp / Output.c's OutputSetTorqueCommand) and the
 * current electrical angle in degrees, produces the two sinusoidal phase
 * commands 120 degrees apart that Output.c's OutputSetTwoPhaseCommand()
 * expects -- see docs/two-phase-drive.md. The third phase is NOT computed
 * here; the external drive infers it (-(Ia+Ib)), same as today. */
void CommutationComputePhases(int32_t torqueCommandQ, int32_t electricalAngleDeg,
                               int32_t *ia, int32_t *ib);

/* ---- Alignment-learning accumulator ---------------------------------------
 *
 * "Where does the motor's electrical zero sit relative to the encoder's
 * zero?" -- not known a priori per motor/encoder pairing, see
 * docs/two-phase-drive.md's "One-time alignment commissioning". Call
 * CommutationLearnReset() once at the start of a learning attempt (see
 * main.c's AppCommutationLearn), then CommutationLearnAccumulate() once per
 * observed Hall-sector transition during an open-loop Hall-driven spin,
 * passing the continuous encoder-derived angle (countsPerElecRev-scaled,
 * offsetDeg=0 -- the raw, not-yet-corrected angle) at the instant of the
 * transition and the sector just transitioned INTO. CommutationLearnEvaluate()
 * then reports whether enough transitions were observed to trust the
 * result, same "don't silently report a meaningless default" shape as
 * Polarity.h's PolarityLearnEvaluate.
 *
 * Internally a running circular mean of (rawEncoderAngleDeg -
 * expectedSectorAngleDeg) -- each new sample is unwrapped to within +-180
 * degrees of the running mean before being folded in, so the mean itself
 * never has to handle a 359/0 wraparound discontinuity. This only works
 * because real samples are expected to cluster near a single true offset,
 * not be spread uniformly around the circle -- true for a correctly-wired
 * motor, not a substitute for actually checking the result is sane. */
#define COMMUTATION_LEARN_MIN_TRANSITIONS 12   /* tune on the bench --
                                                   enough to average out one
                                                   electrical revolution's
                                                   worth of encoder noise,
                                                   not enough to need many
                                                   full mechanical revolutions */

void CommutationLearnReset(void);
void CommutationLearnAccumulate(int32_t rawEncoderAngleDeg, int sectorEnteredNow);

/* Returns 1 (learned; *outOffsetDeg set, 0..359) if at least
 * COMMUTATION_LEARN_MIN_TRANSITIONS samples were accumulated, else 0
 * (*outOffsetDeg left untouched -- caller must not persist it, and must not
 * overwrite any previously-learned offset on a failed attempt, same
 * convention as PolarityLearnEvaluate). */
int CommutationLearnEvaluate(int32_t *outOffsetDeg);

/* ---- Pole-count / counts-per-electrical-rev detection ---------------------
 *
 * `commutation.pole_pairs`/`commutation.counts_per_elec_rev` were pure
 * manual guesses until now -- see docs/two-phase-drive.md's "Detecting
 * pole count" section. Driven off the same live-Hall-sector open-loop test
 * pulse as CommutationLearn* above (see main.c's AppCommutationDetectPoles).
 * Requires the caller to already know the encoder's counts-per-mechanical-
 * revolution ("CPR"): averages encoder counts between consecutive
 * Hall-sector transitions (60 electrical degrees each); countsPerElecRev =
 * 6x that average, pole_pairs = CPR / countsPerElecRev. Doesn't need a full
 * mechanical revolution or a working Z index -- just enough transitions to
 * average out noise.
 *
 * A Z-index-based fallback for when CPR *isn't* known ahead of time was
 * designed but deliberately not implemented here -- see docs/
 * two-phase-drive.md's "Detecting pole count" section for the full idea
 * (not scheduled). */
#define COMMUTATION_POLE_LEARN_MIN_TRANSITIONS 12   /* same 2-electrical-revs
                                                        reasoning as
                                                        COMMUTATION_LEARN_MIN_TRANSITIONS */

void CommutationPoleLearnBySpacingReset(void);
void CommutationPoleLearnBySpacingAccumulate(int32_t encoderPositionCounts, int sectorEnteredNow);

/* encoderCountsPerMechRev must be > 0 (caller's known CPR). Returns 1
 * (outCountsPerElecRev/outPolePairs both set) if at least
 * COMMUTATION_POLE_LEARN_MIN_TRANSITIONS samples were accumulated, else 0
 * (outputs left untouched -- same "don't persist a meaningless result"
 * convention as CommutationLearnEvaluate). */
int CommutationPoleLearnBySpacingEvaluate(int32_t encoderCountsPerMechRev,
                                           int32_t *outCountsPerElecRev,
                                           int32_t *outPolePairs);

#endif /* COMMUTATION_H */
