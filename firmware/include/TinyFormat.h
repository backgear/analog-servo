#ifndef TINY_FORMAT_H
#define TINY_FORMAT_H

#include <stdarg.h>
#include <stddef.h>

/* Drop-in replacement for THIS firmware's own snprintf()/vsnprintf() call
 * sites (Protocol.c's CLI/telemetry text, Monitor.c's schema JSON) -- not a
 * general libc printf replacement, and newlib's own printf family (used
 * nowhere in this codebase today) is untouched. Supports exactly the format
 * subset this project's src/ files actually uses: %d %u %ld %lu %lld (and
 * their unsigned/long-long siblings), %x %lx %llx (always zero-padded,
 * even digit counts only -- see TinyFormat.c), %s, and the one %04x call
 * site in Protocol.c's JSON-string escaper. No precision, no left-justify,
 * no '+'/' ' sign flags, no %c/%o/%p/%e/%f/%g -- see TinyFormat.c's own
 * comment for what's deliberately out of scope and why. Width is a single
 * decimal digit by default; define TINY_FORMAT_MULTI_DIGIT_WIDTH before
 * this header for arbitrary-length widths (e.g. %016llx), at a measured
 * +20 bytes of F103 flash.
 *
 * Signatures match snprintf()/vsnprintf() exactly (same truncate-safely-and-
 * NUL-terminate, same "returns the length that would have been written"
 * convention) so callers only need their function name changed. */

int TinyFormat(char *buf, size_t size, const char *fmt, ...);
int TinyVFormat(char *buf, size_t size, const char *fmt, va_list ap);

#endif /* TINY_FORMAT_H */
