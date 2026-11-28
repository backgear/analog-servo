#include <string.h>
#include "BuildId.h"

#define NT_GNU_BUILD_ID 3U
#define NOTE_DESC_OFFSET 16U   /* 12-byte header + "GNU\0" */

static uint32_t Word(const uint8_t *p) {
  uint32_t w;
  memcpy(&w, p, sizeof(w));
  return w;
}

uint32_t BuildIdFromNote(const uint8_t *note, uint32_t avail, const uint8_t **id) {
  if (avail < NOTE_DESC_OFFSET) return 0U;
  uint32_t descLen = Word(note + 4);
  if (Word(note) != 4U || Word(note + 8) != NT_GNU_BUILD_ID) return 0U;
  if (memcmp(note + 12, "GNU", 4) != 0) return 0U;
  if (descLen > avail - NOTE_DESC_OFFSET) return 0U;
  *id = note + NOTE_DESC_OFFSET;
  return descLen;
}
