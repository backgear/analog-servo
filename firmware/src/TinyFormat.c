#include "TinyFormat.h"
#include <stdint.h>

/* newlib nano's own snprintf/vsnprintf chain (_svfiprintf_r/_printf_i/
 * _printf_common) costs ~1.5KB on F103 even in its integer-only form; this
 * file replaces it with ~800 bytes of code, for a net ~1.2KB of flash back
 * on a board that's short on it. Not because it's faster -- it isn't,
 * meaningfully -- but because most of that 1.5KB is generality this
 * firmware's own call sites never use: no float support is linked
 * (confirmed -- no %f/%e/%g anywhere in this project's src/ files), and of
 * the integer conversions it does support, only %d/%u (with l/ll length)
 * and %s ever actually appear, plus exactly one %04x site. This file is
 * that minimal subset, written directly against this codebase's real
 * call sites rather than against the printf spec.
 *
 * Deliberately NOT supported, because no call site anywhere in this project's src/ files uses
 * them: %c, %o, %p, %e/%f/%g, precision (".N"), left-justify ('-'), '+'/' '
 * sign flags. An unrecognized specifier is emitted literally (e.g. "%q"
 * prints as "%q") rather than silently eaten, so a future caller that
 * needs one of these gets visibly wrong output instead of a quietly wrong
 * one.
 *
 * %x/%lx/%llx are built from PutHexByte, a fixed two-digit (one byte)
 * primitive with no loop and no width/pad params -- composing wider hex
 * from repeated calls to it beats a general digit-counting loop over a
 * wider int, PROVIDED two things every real call site here already does
 * also hold for any future one: width is always an even digit count (whole
 * bytes -- no %03x), and the field is always zero-padded (no bare, space-
 * padded %x). Trading those away drops leading-zero suppression entirely,
 * which is why a bare %x prints a minimum of 2 digits ("05", not printf's
 * "5") -- fine for the one real site (always "\u%04x"), wrong for a
 * hypothetical bare %x this codebase doesn't have. Given that, width is a
 * FLOOR, not a ceiling: TinyVFormat grows the byte count instead of
 * truncating whenever the value has nonzero bytes above the requested
 * window, so nothing is ever silently dropped -- the only divergence left
 * is that an odd natural digit count rounds up to the next even one (one
 * extra leading zero), the explicit cost of using a byte-sized primitive.
 *
 * Width is parsed as a single decimal digit by default (every real call
 * site needs exactly one: the "4" in "%04x"). Define
 * TINY_FORMAT_MULTI_DIGIT_WIDTH before including TinyFormat.h for
 * arbitrary-length widths (needed for e.g. %016llx) -- measured +20 bytes
 * of F103 flash; off by default since nothing calls for it today. */

typedef struct {
  char *buf;
  size_t size;
  size_t pos;
} Writer;

static void PutChar(Writer *w, char c) {
  if (w->pos + 1 < w->size) w->buf[w->pos] = c;
  w->pos++;
}

static void PutStr(Writer *w, const char *s) {
  while (*s) PutChar(w, *s++);
}

/* Unsigned 64-bit -> decimal. tmp[20] covers UINT64_MAX's 20 digits -- the
 * real call sites (e.g. RLS.c's p_trace) never get close, but the division
 * this needs is already linked into this firmware regardless (Control.c/
 * Commutation.c/Analog.c's own int64_t fixed-point math pulls in
 * __udivmoddi4 independently -- see this project's own measurement of it),
 * so there's no cost to being fully correct here instead of cutting a
 * corner that wouldn't save anything. */
static void PutUDec64(Writer *w, uint64_t v) {
  char tmp[20];
  int n = 0;
  if (v == 0) { PutChar(w, '0'); return; }
  while (v) { tmp[n++] = (char)('0' + (int)(v % 10)); v /= 10; }
  while (n > 0) PutChar(w, tmp[--n]);
}

static void PutSDec64(Writer *w, int64_t v) {
  uint64_t uv = (uint64_t)v;
  if (v < 0) { PutChar(w, '-'); uv = -uv; }
  PutUDec64(w, uv);
}

/* Exactly two hex digits, lower-case, always zero-padded -- a fixed-size
 * building block, not a general formatter: no loop, no width/zeroPad
 * params, nothing to branch on. See this file's top comment for why
 * composing wider hex from repeated calls to this beats one loop over a
 * wider int, and what it costs in printf-compatibility. */
static void PutHexByte(Writer *w, uint8_t v) {
  int hi = (v >> 4) & 0xF, lo = v & 0xF;
  PutChar(w, (char)(hi < 10 ? '0' + hi : 'a' + hi - 10));
  PutChar(w, (char)(lo < 10 ? '0' + lo : 'a' + lo - 10));
}

int TinyVFormat(char *buf, size_t size, const char *fmt, va_list ap) {
  Writer w;
  w.buf = buf;
  w.size = size;
  w.pos = 0;

  for (const char *p = fmt; *p; p++) {
    if (*p != '%') { PutChar(&w, *p); continue; }
    p++;
    if (*p == '%') { PutChar(&w, '%'); continue; }

    /* Only flag this codebase ever uses: a leading '0' zero-pad flag --
     * every real %x site has it, so it's consumed but not tracked (%x is
     * unconditionally zero-padded; see this file's top comment). */
    int width = 0;
    if (*p == '0') p++;
#ifdef TINY_FORMAT_MULTI_DIGIT_WIDTH
    while (*p >= '0' && *p <= '9') { width = width * 10 + (*p - '0'); p++; }
#else
    if (*p >= '0' && *p <= '9') { width = *p - '0'; p++; }
#endif

    int longCount = 0;
    while (*p == 'l') { longCount++; p++; }

    switch (*p) {
      case 'd': {
        int64_t v = (longCount >= 2) ? (int64_t)va_arg(ap, long long)
                  : (longCount == 1) ? (int64_t)va_arg(ap, long)
                                      : (int64_t)va_arg(ap, int);
        PutSDec64(&w, v);
        break;
      }
      case 'u': {
        uint64_t v = (longCount >= 2) ? (uint64_t)va_arg(ap, unsigned long long)
                   : (longCount == 1) ? (uint64_t)va_arg(ap, unsigned long)
                                        : (uint64_t)va_arg(ap, unsigned int);
        PutUDec64(&w, v);
        break;
      }
      case 'x': {
        uint64_t v = (longCount >= 2) ? (uint64_t)va_arg(ap, unsigned long long)
                   : (longCount == 1) ? (uint64_t)va_arg(ap, unsigned long)
                                       : (uint64_t)va_arg(ap, unsigned int);
        int numBytes = width / 2;
        if (numBytes < 1) numBytes = 1;
        /* Width is a floor: grow past it instead of truncating whenever
         * the value still has nonzero bytes above the current window. */
        while (numBytes < 8 && (v >> (8 * numBytes)) != 0) numBytes++;
        for (int i = numBytes - 1; i >= 0; i--)
          PutHexByte(&w, (uint8_t)(v >> (8 * i)));
        break;
      }
      case 's': {
        const char *s = va_arg(ap, const char *);
        PutStr(&w, s ? s : "(null)");
        break;
      }
      default:
        /* Unrecognized specifier: print literally rather than eat it, so a
         * future call site that needs something unsupported fails loudly
         * (visibly wrong CLI output) instead of silently. */
        PutChar(&w, '%');
        if (*p) PutChar(&w, *p);
        else p--;   /* don't walk past the trailing NUL */
        break;
    }
  }

  if (w.size > 0) {
    size_t end = (w.pos < w.size) ? w.pos : w.size - 1;
    w.buf[end] = '\0';
  }
  return (int)w.pos;
}

int TinyFormat(char *buf, size_t size, const char *fmt, ...) {
  va_list ap;
  int r;
  va_start(ap, fmt);
  r = TinyVFormat(buf, size, fmt, ap);
  va_end(ap);
  return r;
}
