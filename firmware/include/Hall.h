#ifndef HALL_H
#define HALL_H

/* Pure Hall-sector decode/encode -- see docs/two-phase-drive.md. Shared by
 * both directions: HallDecode() for reading real Hall lines (HALL_A/B/C,
 * see gpio.h), HallEncode() for the (unimplemented, not wired yet)
 * emulated-Hall-output idea in the same doc.
 *
 * Standard 6-step Gray-code BLDC Hall sequence: of the 8 possible 3-bit
 * (A,B,C) combinations, 6 are valid "sectors" and adjacent sectors differ
 * in exactly one bit; 000 and 111 are invalid (wiring fault or a line not
 * actually connected). Sector numbering direction (which physical state is
 * "sector 0") is an arbitrary convention here, NOT claimed to match any
 * particular motor's true electrical zero or rotation sense -- that's
 * exactly what Commutation.c's alignment-learning step is for. This table
 * only needs to be internally consistent (monotonic by one bit per step),
 * not independently correct per motor. */

/* Returns 0..5 (sector) for a valid Hall state, -1 for 000/111 (invalid --
 * caller should treat this as a wiring fault, not a sector to act on). */
int HallDecode(int a, int b, int c);

/* Inverse of HallDecode(): sector (wrapped to 0..5) -> the 3 output levels
 * for that sector. For emulated-Hall output -- see docs/two-phase-drive.md's
 * "Outputting emulated Halls" section (not wired to any real signal source
 * yet -- would need an absolute feedback source to drive it; see the
 * `tamagawa-feedback` draft branch). */
void HallEncode(int sector, int *a, int *b, int *c);

#endif /* HALL_H */
