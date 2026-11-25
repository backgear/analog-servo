/*
 * Hall-sector decode/encode. See Hall.h.
 */
#include <stdint.h>
#include "Hall.h"

/* Index by (a<<2)|(b<<1)|c. Sector-to-bit-pattern assignment (sector: abc):
 * 0:101 1:100 2:110 3:010 4:011 5:001 -- adjacent sectors (mod 6) differ in
 * exactly one bit, same shape as any standard BLDC Hall sequence. */
static const int8_t kHallDecodeTable[8] = {
  -1, /* 000 invalid */
   5, /* 001 */
   3, /* 010 */
   4, /* 011 */
   1, /* 100 */
   0, /* 101 */
   2, /* 110 */
  -1, /* 111 invalid */
};

static const uint8_t kHallEncodeTable[6] = {
  0x5, /* sector 0 -> 101 */
  0x4, /* sector 1 -> 100 */
  0x6, /* sector 2 -> 110 */
  0x2, /* sector 3 -> 010 */
  0x3, /* sector 4 -> 011 */
  0x1, /* sector 5 -> 001 */
};

int HallDecode(int a, int b, int c) {
  int idx = ((a ? 1 : 0) << 2) | ((b ? 1 : 0) << 1) | (c ? 1 : 0);
  return kHallDecodeTable[idx];
}

void HallEncode(int sector, int *a, int *b, int *c) {
  int s = ((sector % 6) + 6) % 6;
  uint8_t v = kHallEncodeTable[s];
  *a = (v >> 2) & 1;
  *b = (v >> 1) & 1;
  *c = v & 1;
}
