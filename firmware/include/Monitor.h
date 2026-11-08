#ifndef MONITOR_H
#define MONITOR_H

#include <stdint.h>

/* Channel-1 binary telemetry stream (the composite USB "monitor" port) --
 * see docs/monitor-protocol.md. Fixed 20-byte frame, one per 1kHz control
 * tick while enabled: sync byte + seq + position + velocity + torque +
 * mode/enabled/fault status + fault reason + pad. No checksum beyond the
 * frame's own CRC-8 (USB CDC is a reliable framed transport) -- the sync
 * byte + fixed size + CRC-8 are resync aids for a host that attaches
 * mid-stream.
 *
 * Toggled via Protocol.c's `monitor start`/`monitor stop` CLI command on
 * channel 0, OR via a single in-band command byte on channel 1 itself --
 * see MonitorRxFromIsrUsb() below. A host that only ever opens channel 1
 * doesn't need channel 0 at all for basic use.
 *
 * If monitoring is left enabled with nobody actually reading channel 1, the
 * in-flight frame never completes and every later tick gets dropped --
 * there's no timeout here to un-stick it. It self-recovers the moment a
 * reader attaches; expect a large `seq` jump right after connecting in that
 * case -- that's normal, not a dropped/corrupted packet. */

void MonitorInit(void);
void MonitorSetEnabled(int enabled);
int  MonitorGetEnabled(void);

/* In-band commands on channel 1 itself (host->device), fed from Ch1_Receive's
 * ISR context the same way Protocol.c's ProtocolRxFromIsrUsb() feeds its own
 * ring -- this call only enqueues, MonitorPoll() drains and acts on it from
 * the main superloop. Single ASCII bytes, no line assembly needed:
 *   '1' -- enable streaming, same effect as `monitor start`
 *   '0' -- disable streaming, same effect as `monitor stop`
 *   '?' -- queue one MonitorGetSchemaJson() reply (see below)
 * Any other byte is ignored. This exists so a tool that only has channel 1
 * open -- no CLI -- can still start itself and learn the frame layout; it
 * does NOT turn channel 1 into a general command channel, no CLI-shaped
 * parsing was added here on purpose. */
void MonitorRxFromIsrUsb(const uint8_t *data, uint32_t len);

/* A JSON description of MonitorFrame's wire layout -- field names, byte
 * offsets, sizes and types, plus the status-bit map, the fault-reason
 * name list, and the CRC-8 parameters -- built from the real struct via
 * sizeof()/offsetof(), not hand-copied, so it can't silently drift out of
 * sync with the frame it describes the way a doc table could. Lets a host
 * tool (or an offline capture converter) decode frames without hardcoding
 * today's struct, so it keeps working if the frame format ever changes,
 * AS LONG AS that tool re-fetches this instead of caching an old copy.
 * Null-terminated, single line (no embedded newline except the trailing
 * one), built lazily on first call and cached after that -- cheap to call
 * repeatedly. Served in-band on channel 1 in response to '?' (above) and
 * over the CLI as `monitor schema`. */
const char *MonitorGetSchemaJson(void);

/* Called once per 1kHz control tick from AppControlTick() (TIM6 ISR
 * context) -- snapshots the tick's telemetry into a pending frame only, no
 * USB calls here. The control tick runs at a HIGHER NVIC priority than the
 * USB IRQ, so it can preempt the USB stack mid-transfer; calling into the
 * USB class driver from here would race that. MonitorPoll() does the
 * actual push from the main superloop instead. */
void MonitorTick(void);

/* Call from the main superloop (alongside ProtocolPoll/IndicatorPoll) --
 * pushes one pending frame over channel 1 if one is waiting and the
 * previous transmit has drained. No-op if disabled, nothing pending, or
 * the device isn't enumerated/configured yet. */
void MonitorPoll(void);

/* Rolling "flight recorder" history, ALWAYS being recorded by MonitorTick()
 * every control tick -- independent of MonitorSetEnabled()/`monitor start`.
 * MonitorDumpHistory() (Protocol.c's `monitor dump`) queues the whole
 * buffer for one-shot transmission over channel 1, oldest frame first,
 * using the IDENTICAL MonitorFrame wire format live streaming uses.
 *
 * While a dump is draining (MonitorDumpActive()), MonitorTick() stops
 * queuing NEW live frames (recording into the history buffer continues
 * regardless) -- live streaming automatically resumes once the dump
 * finishes. Calling MonitorDumpHistory() again while one is already active
 * is a no-op. */
void     MonitorDumpHistory(void);
int      MonitorDumpActive(void);
uint32_t MonitorDumpFramesRemaining(void);   /* 0 once a dump has fully drained --
                                                  also 0, meaninglessly, whenever
                                                  !MonitorDumpActive() */

/* How many ticks of recording continue PAST the instant a fault latches,
 * before the history buffer freezes -- lets a dump show the immediate
 * aftermath, not just the lead-up. Clamped to at most
 * MONITOR_HISTORY_LEN/2 (Monitor.c). Re-arms (resumes normal recording)
 * the instant the fault clears. Not persisted. */
void     MonitorSetFreezePostTicks(uint32_t ticks);
uint32_t MonitorGetFreezePostTicks(void);

#endif /* MONITOR_H */
