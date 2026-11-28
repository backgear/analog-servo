/**
 * GNU build-ID note decode, kept apart from main.c so the native tests can
 * run it against hand-built notes. main.c passes the bounds of the
 * .note.gnu.build-id section that ld/STM32*_FLASH.ld places in flash.
 */
#ifndef BUILD_ID_H
#define BUILD_ID_H

#include <stdint.h>

/* Note layout: namesz(4) descsz(4) type(4), name padded to 4 bytes, desc.
 * Points *id at desc and returns its length if [note, note+avail) holds a
 * whole NT_GNU_BUILD_ID note named "GNU"; returns 0 otherwise. */
uint32_t BuildIdFromNote(const uint8_t *note, uint32_t avail, const uint8_t **id);

#endif /* BUILD_ID_H */
