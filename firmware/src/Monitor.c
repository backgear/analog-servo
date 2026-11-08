#include "Monitor.h"
#include "main.h"
#include "Encoder.h"
#include "Fault.h"
#include "usbd_cdc_composite_if.h"
#include "usbd_cdc_composite.h"
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define MONITOR_FRAME_SYNC 0xAAU

typedef struct __attribute__((packed)) {
  uint8_t  sync;             /* MONITOR_FRAME_SYNC -- resync aid, not a substitute for crc8 */
  uint32_t seq;               /* control-tick counter, NOT frame-sent counter --
                                  gaps reveal ticks dropped while a previous
                                  frame was still transmitting */
  int32_t  positionCounts;    /* MEASURED (encoder) */
  int32_t  velocityCounts;   /* per-tick delta, same value as velocity.raw -- MEASURED */
  int32_t  torqueCommand;
  int32_t  commandedPosition;   /* DESIRED, not measured -- AppGetLastCommandedPosition().
                                     Only updated by branches that actually pursue a
                                     position target -- stale (not wrong, just unchanged)
                                     during polarity learning, raw open-loop signal
                                     torque, disabled, or autotune. */
  int32_t  commandedVelocity;    /* DESIRED, not measured -- same staleness caveats */
  uint8_t  status;           /* bits0-1 = mode (0 stepdir/1 velocity/2 analog), bit2 =
                                 enabled, bit3 = fault latched, bit4 = reserved (always 0),
                                 bit5 = hardware ENABLE pin asserted, bit6 = synthetic
                                 (this tick's position/velocity/torque came from the
                                 polarity-learn test pulse, autotune, or the built-in
                                 signal generator, NOT the normal control loop), bit7 =
                                 cycle_start (set for exactly one tick at the start of
                                 each SIGNAL_SQUARE cycle, so a host-side viewer/decoder
                                 can trigger on a firmware-sourced marker). */
  uint8_t  faultReason;      /* FaultReason */
  uint8_t  crc8;             /* CRC-8 (poly 0x07, init 0, MSB-first, no reflect/xor-out)
                                 over bytes 0..26 of this struct -- see Crc8() below and
                                 docs/monitor-protocol.md. */
} MonitorFrame;

static MonitorFrame sFrame;
static uint32_t     sTickCounter;
static int          sEnabled;
static volatile int sPending;

/* Rolling flight-recorder history -- see Monitor.h's MonitorDumpHistory()
 * comment. MONITOR_HISTORY_LEN*sizeof(MonitorFrame) = 300*28 = 8400 B. */
#define MONITOR_HISTORY_LEN 300U
static MonitorFrame sHistory[MONITOR_HISTORY_LEN];
static uint32_t     sHistoryHead;    /* next write index */
static uint32_t     sHistoryCount;   /* valid entries recorded so far, caps at
                                          MONITOR_HISTORY_LEN once the buffer wraps */
static int           sDumpActive;
static uint32_t      sDumpReadIndex;        /* absolute sHistory[] index of the NEXT
                                                 frame MonitorPoll() should send */
static uint32_t       sDumpFramesRemaining;

/* Freeze-on-fault. MONITOR_FREEZE_POST_TICKS_MAX caps how much of the
 * buffer a configured post-fault window may consume, so a misconfigured
 * huge value can't eat the entire buffer and leave nothing of the
 * pre-fault lead-up it exists to show. */
#define MONITOR_FREEZE_POST_TICKS_DEFAULT 200U   /* ~0.2s @ 1kHz */
#define MONITOR_FREEZE_POST_TICKS_MAX     (MONITOR_HISTORY_LEN / 2U)
static int            sHistoryFrozen;
static int            sWasFaultLatchedForHistory;
static uint32_t        sFreezePostTicks = MONITOR_FREEZE_POST_TICKS_DEFAULT;
static int             sPostFaultCounting;         /* 1 while counting down the
                                                         configured post-fault window */
static uint32_t        sPostFaultTicksRemaining;

/* In-band command bytes received on channel 1 -- see MonitorRxFromIsrUsb()
 * (ISR, enqueues only) and ProcessRxCommands() (main superloop, drains and
 * acts). Sized generously for a handful of host keystrokes; commands are
 * rare, one-byte, and never queue up under normal use. */
#define MONITOR_RX_RING_SIZE 8U
static volatile uint8_t sRxRing[MONITOR_RX_RING_SIZE];
static volatile uint32_t sRxHead;   /* written by the ISR */
static uint32_t          sRxTail;   /* read by ProcessRxCommands() only */

/* Lazily-built, then cached -- see MonitorGetSchemaJson()'s own comment in
 * Monitor.h for why this is computed from the struct instead of hand-typed.
 * Sized generously; BuildSchemaJson() leaves room to confirm no truncation
 * happened (see its own assert-shaped comment below). */
#define MONITOR_SCHEMA_JSON_BUF_SIZE 1024U
static char sSchemaJson[MONITOR_SCHEMA_JSON_BUF_SIZE];
static int  sSchemaBuilt;
static int  sSchemaPending;   /* '?' seen, not yet sent -- highest priority in MonitorPoll() */

/* Table-less CRC-8, poly 0x07 (CRC-8/CCITT family). */
static uint8_t Crc8(const uint8_t *data, uint32_t len) {
  uint8_t crc = 0U;
  while (len--) {
    crc ^= *data++;
    for (int i = 0; i < 8; i++) {
      crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

void MonitorInit(void) {
  sTickCounter = 0U;
  sEnabled = 0;
  sPending = 0;
  sHistoryHead = 0U;
  sHistoryCount = 0U;
  sDumpActive = 0;
  sDumpReadIndex = 0U;
  sDumpFramesRemaining = 0U;
  sHistoryFrozen = 0;
  sWasFaultLatchedForHistory = 0;
  sPostFaultCounting = 0;
  sPostFaultTicksRemaining = 0U;
  sRxHead = 0U;
  sRxTail = 0U;
  sSchemaPending = 0;
  /* sFreezePostTicks and sSchemaJson/sSchemaBuilt deliberately NOT reset
   * here -- the former is a configured setting (`set
   * monitor.post_trigger_ticks`), not per-run state; the latter is a pure
   * function of MonitorFrame's layout, which doesn't change at runtime, so
   * there's never a reason to rebuild it after the first call. */
}

void MonitorSetFreezePostTicks(uint32_t ticks) {
  sFreezePostTicks = (ticks > MONITOR_FREEZE_POST_TICKS_MAX) ? MONITOR_FREEZE_POST_TICKS_MAX : ticks;
}
uint32_t MonitorGetFreezePostTicks(void) { return sFreezePostTicks; }

void MonitorSetEnabled(int enabled) { sEnabled = enabled ? 1 : 0; }
int  MonitorGetEnabled(void) { return sEnabled; }

void MonitorRxFromIsrUsb(const uint8_t *data, uint32_t len) {
  for (uint32_t i = 0; i < len; i++) {
    uint32_t next = (sRxHead + 1U) % MONITOR_RX_RING_SIZE;
    if (next == sRxTail) break;   /* full: drop rather than overwrite unread data,
                                        same convention as Protocol.c's RingPush() */
    sRxRing[sRxHead] = data[i];
    sRxHead = next;
  }
}

static void AppendSchema(size_t *n, const char *fmt, ...) {
  if (*n >= sizeof(sSchemaJson)) return;
  va_list ap;
  va_start(ap, fmt);
  int wrote = vsnprintf(sSchemaJson + *n, sizeof(sSchemaJson) - *n, fmt, ap);
  va_end(ap);
  if (wrote > 0) *n += (size_t)wrote;
  if (*n > sizeof(sSchemaJson)) *n = sizeof(sSchemaJson);   /* vsnprintf's return value
      counts what it WOULD have written, past a truncation -- clamp so a later
      call can't compute a negative/huge remaining size */
}

/* Built from MonitorFrame's real layout (sizeof/offsetof), not hand-typed,
 * so it can't drift from the struct it describes the way a doc table
 * could -- see Monitor.h's MonitorGetSchemaJson() comment. */
static void BuildSchemaJson(void) {
  size_t n = 0;
  AppendSchema(&n, "{\"frame_size\":%u,\"sync\":%u,\"fields\":[",
               (unsigned)sizeof(MonitorFrame), (unsigned)MONITOR_FRAME_SYNC);
  AppendSchema(&n, "{\"name\":\"sync\",\"offset\":%u,\"size\":1,\"type\":\"u8\"},",
               (unsigned)offsetof(MonitorFrame, sync));
  AppendSchema(&n, "{\"name\":\"seq\",\"offset\":%u,\"size\":4,\"type\":\"u32\"},",
               (unsigned)offsetof(MonitorFrame, seq));
  AppendSchema(&n, "{\"name\":\"position\",\"offset\":%u,\"size\":4,\"type\":\"i32\"},",
               (unsigned)offsetof(MonitorFrame, positionCounts));
  AppendSchema(&n, "{\"name\":\"velocity\",\"offset\":%u,\"size\":4,\"type\":\"i32\"},",
               (unsigned)offsetof(MonitorFrame, velocityCounts));
  AppendSchema(&n, "{\"name\":\"torque_command\",\"offset\":%u,\"size\":4,\"type\":\"i32\"},",
               (unsigned)offsetof(MonitorFrame, torqueCommand));
  AppendSchema(&n, "{\"name\":\"commanded_position\",\"offset\":%u,\"size\":4,\"type\":\"i32\"},",
               (unsigned)offsetof(MonitorFrame, commandedPosition));
  AppendSchema(&n, "{\"name\":\"commanded_velocity\",\"offset\":%u,\"size\":4,\"type\":\"i32\"},",
               (unsigned)offsetof(MonitorFrame, commandedVelocity));
  AppendSchema(&n, "{\"name\":\"status\",\"offset\":%u,\"size\":1,\"type\":\"u8\"},",
               (unsigned)offsetof(MonitorFrame, status));
  AppendSchema(&n, "{\"name\":\"fault_reason\",\"offset\":%u,\"size\":1,\"type\":\"u8\"},",
               (unsigned)offsetof(MonitorFrame, faultReason));
  AppendSchema(&n, "{\"name\":\"crc8\",\"offset\":%u,\"size\":1,\"type\":\"u8\"}",
               (unsigned)offsetof(MonitorFrame, crc8));
  AppendSchema(&n,
    "],\"status_bits\":{\"mode\":\"0-1\",\"enabled\":2,\"fault_latched\":3,"
    "\"hw_enable\":5,\"synthetic\":6,\"cycle_start\":7},"
    "\"fault_reasons\":[" FAULT_REASON_NAMES_JSON "],"
    "\"crc8\":{\"poly\":7,\"init\":0,\"msb_first\":true,\"reflect\":false,"
    "\"xorout\":0,\"covers_bytes\":[0,%u]},"
    "\"rx_commands\":{\"1\":\"enable\",\"0\":\"disable\",\"?\":\"request_this_schema\"}}\n",
    (unsigned)(sizeof(MonitorFrame) - 2U));
  sSchemaBuilt = 1;
}

const char *MonitorGetSchemaJson(void) {
  if (!sSchemaBuilt) BuildSchemaJson();
  return sSchemaJson;
}

static void BuildFrame(MonitorFrame *out, uint32_t seq) {
  int synthetic = AppPolarityLearnBusy() || AppAutotuneBusy() || AppGetSignalActive();
  int cycleStart = AppGetSignalCycleStart();

  out->sync = MONITOR_FRAME_SYNC;
  out->seq = seq;
  out->positionCounts = EncoderGetPositionCounts();
  out->velocityCounts = AppGetLastVelocityActual();
  out->torqueCommand  = AppGetLastTorqueCommand();
  out->commandedPosition = AppGetLastCommandedPosition();
  out->commandedVelocity = AppGetLastCommandedVelocity();
  out->status = (uint8_t)(((uint8_t)AppGetServoMode() & 0x3U) |
                           (((uint8_t)AppGetEnabled() & 0x1U) << 2) |
                           (((uint8_t)FaultIsLatched() & 0x1U) << 3) |
                           (((uint8_t)AppGetHwEnableAsserted() & 0x1U) << 5) |
                           (((uint8_t)synthetic & 0x1U) << 6) |
                           (((uint8_t)cycleStart & 0x1U) << 7));
  out->faultReason = (uint8_t)FaultGetReason();
  out->crc8 = Crc8((const uint8_t *)out, (uint32_t)(sizeof(*out) - 1U));
}

void MonitorTick(void) {
  /* Recording into the history ring buffer is ALWAYS-ON (independent of
   * sEnabled/`monitor start`). */
  uint32_t seq = sTickCounter++;

  MonitorFrame frame;
  BuildFrame(&frame, seq);

  /* Freeze the rolling history once a fault's own configured post-fault
   * window has elapsed -- NOT at the instant it latches, so a `monitor
   * dump` shows the immediate aftermath too, not just the lead-up. */
  if (!sHistoryFrozen) {
    sHistory[sHistoryHead] = frame;
    sHistoryHead = (sHistoryHead + 1U) % MONITOR_HISTORY_LEN;
    if (sHistoryCount < MONITOR_HISTORY_LEN) sHistoryCount++;
  }

  int faultNowLatched = FaultIsLatched();
  if (faultNowLatched && !sWasFaultLatchedForHistory) {
    sPostFaultCounting = 1;
    sPostFaultTicksRemaining = sFreezePostTicks;
  } else if (!faultNowLatched && sWasFaultLatchedForHistory) {
    /* Fault cleared -- re-arm immediately. */
    sHistoryFrozen = 0;
    sPostFaultCounting = 0;
  }
  sWasFaultLatchedForHistory = faultNowLatched;

  if (sPostFaultCounting) {
    if (sPostFaultTicksRemaining == 0U) {
      sHistoryFrozen = 1;
      sPostFaultCounting = 0;
    } else {
      sPostFaultTicksRemaining--;
    }
  }

  if (!sEnabled) return;
  if (sPending) return;      /* previous live frame not yet drained -- drop this sample */
  if (sDumpActive) return;   /* a `monitor dump` is draining -- let it finish first */

  sFrame = frame;
  sPending = 1;
}

/* Shared by the live-frame path, the history-dump path, and the schema
 * reply below. Returns 1 once fully sent (caller may move on), 0 if the
 * caller should retry the SAME bytes next poll (USB still busy) -- so
 * `data` must stay valid/unchanged across retries, which is why the
 * schema reply is served out of the persistent cached buffer rather than
 * a stack-local copy. */
static int TransmitBytes(const uint8_t *data, uint16_t len) {
  /* USBD_COMPOSITE_CDC_SetTxBuffer derefs pClassData with no NULL check
   * (safe at every OTHER call site because it's only ever called from the
   * class driver's own Init(), after pClassData is allocated) -- this is a
   * new call site that can run before enumeration ever completes, so
   * guard it here. */
  if (hUsbDeviceFS.pClassData == NULL) return 1;   /* no host attached -- count this frame as
    "done" anyway rather than stall forever */

  USBD_COMPOSITE_CDC_SetTxBuffer(&hUsbDeviceFS, 1U, (uint8_t *)(uintptr_t)data, len);
  if (USBD_COMPOSITE_CDC_TransmitPacket(&hUsbDeviceFS, 1U) == USBD_BUSY) return 0;   /* retry next poll */

  return 1;
}
static int TransmitFrame(const MonitorFrame *frame) {
  return TransmitBytes((const uint8_t *)frame, (uint16_t)sizeof(*frame));
}

static int RxRingPop(uint8_t *out) {
  if (sRxTail == sRxHead) return 0;
  *out = sRxRing[sRxTail];
  sRxTail = (sRxTail + 1U) % MONITOR_RX_RING_SIZE;
  return 1;
}

/* Drains MonitorRxFromIsrUsb()'s ring and acts -- see its comment in
 * Monitor.h for the one-byte command set. Called every MonitorPoll(), from
 * the main superloop, same split as everywhere else in this file (ISR
 * enqueues, superloop acts). */
static void ProcessRxCommands(void) {
  uint8_t b;
  while (RxRingPop(&b)) {
    switch (b) {
      case '1': MonitorSetEnabled(1); break;
      case '0': MonitorSetEnabled(0); break;
      case '?': sSchemaPending = 1; break;
      default: break;   /* unrecognized -- ignored; no error channel exists on this wire */
    }
  }
}

void MonitorPoll(void) {
  ProcessRxCommands();

  if (sSchemaPending) {
    /* Highest priority: tiny, one-shot, and the host is presumably waiting
     * on it right now -- pauses live streaming and any in-progress dump
     * exactly the way a dump already pauses live streaming below. */
    if (!sSchemaBuilt) BuildSchemaJson();
    if (!TransmitBytes((const uint8_t *)sSchemaJson, (uint16_t)strlen(sSchemaJson))) return;
    sSchemaPending = 0;
    return;
  }

  if (sDumpActive) {
    if (sDumpFramesRemaining == 0U) { sDumpActive = 0; return; }
    if (!TransmitFrame(&sHistory[sDumpReadIndex])) return;   /* busy -- retry this
                                                                  same frame next poll */
    sDumpReadIndex = (sDumpReadIndex + 1U) % MONITOR_HISTORY_LEN;
    sDumpFramesRemaining--;
    return;
  }

  if (!sPending) return;
  if (!TransmitFrame(&sFrame)) return;   /* retry next poll */
  sPending = 0;
}

void MonitorDumpHistory(void) {
  if (sDumpActive) return;   /* already dumping -- ignore a re-trigger */
  sDumpFramesRemaining = sHistoryCount;
  if (sDumpFramesRemaining == 0U) return;   /* nothing recorded yet this boot */
  /* Oldest-first chronological order: before the buffer has wrapped
   * (sHistoryCount < MONITOR_HISTORY_LEN), the oldest frame is still at
   * index 0; once wrapped, the oldest is whatever sHistoryHead is about
   * to overwrite next. */
  sDumpReadIndex = (sHistoryCount < MONITOR_HISTORY_LEN) ? 0U : sHistoryHead;
  sDumpActive = 1;
}

int      MonitorDumpActive(void) { return sDumpActive; }
uint32_t MonitorDumpFramesRemaining(void) { return sDumpFramesRemaining; }
