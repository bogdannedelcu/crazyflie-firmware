/*
 * sentai_bridge.c — Crazyflie deck driver for the SentAI board on UART2.
 *
 * Bidirectional bridge between the radio Crazyradio link (host PC) and
 * the SentAI Coral Dev Board on UART2:
 *
 *           host PC                        drone STM32             SentAI board
 *  cf.send_packet(port=0x0E)  ─►  on_radio_packet (CRTP cb) ─► UART2 raw 0xAA frame
 *                                                              SentAI RX state machine
 *                                                              → MicroPython process_message
 *
 *  cf.add_port_callback(0x0E) ◄── crtpSendPacket(...) ◄──── UART2 raw 0xAA frame
 *                                                              ◄── sentai.crazy.send()
 *
 * Pattern follows Bitcraze deck-driver howto exactly. NO Appchannel,
 * NO CPX UART transport, NO flow-control state machine. Each direction
 * is one short non-blocking handler:
 *
 *   * inbound (host → board) runs in the high-priority CRTP RX task
 *     (Crazyflie firmware dispatch). Bounded body, stack buffer,
 *     uart2SendData is DMA — never blocks the radio scheduler.
 *
 *   * outbound (board → host) is a dedicated FreeRTOS task that reads
 *     UART2 byte-stream into a tiny state machine, then ships a
 *     CRTPPacket on the CRTP TX queue. The task is CRTP-priority so it
 *     plays well with the radio link.
 *
 * Wire format on UART2 (both directions):
 *
 *   +------+-----+----+--------+-----+
 *   | 0xAA | LEN | CH | DATA…  | CRC |
 *   +------+-----+----+--------+-----+
 *
 *   0xAA  start byte (distinct from CPX 0xFF so the two stacks could
 *         theoretically cohabit on the same UART)
 *   LEN   uint8_t  CH + DATA length (1..32)
 *   CH    uint8_t  CRTP channel (0..3)
 *   DATA  payload (≤30 bytes — CRTP MAX_PAYLOAD)
 *   CRC   XOR of all preceding bytes including 0xAA + LEN
 */
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "deck_constants.h"
#include "param.h"
#include "crtp.h"
#include "uart2.h"
#include "system.h"
#include "estimator.h"
#include "stabilizer_types.h"
#include "log.h"

#define DEBUG_MODULE "SENTAI"
#include "debug.h"

/* Build-time guard: this deck driver assumes EXCLUSIVE ownership of
 * UART2. CPX_UART_TRANSPORT (compiled when ENABLE_CPX is on) calls
 * uart2SendData() and reads from rxStream too — both writers race for
 * the static txBuffer/txIdx/txSize globals in uart2.c, the second TX
 * clobbers the first's in-flight state, TX_DONE never fires for our
 * deck, on_radio_packet wedges in xEventGroupWaitBits, the entire
 * CRTP RX task stalls. Empirically validated 2026-05-06 — the fix
 * was `CONFIG_ENABLE_CPX=n` in app-config. Don't undo it without
 * deeper refactor (e.g. dedicated TX queue/task). */
#if defined(CONFIG_ENABLE_CPX) && CONFIG_ENABLE_CPX
#error "ENABLE_CPX must be 'n' for this deck driver — see app-config note"
#endif

/* Wire format on UART2 channel 2 carries float32 little-endian.
 * Both ends are Cortex-M (LE) so direct memcpy works; assert it. */
_Static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__,
               "telem float wire format assumes Cortex-M LE byte order");

#define LINK_PORT       0x0E
#define WIRE_START      0xAAu
#define UART_BAUDRATE   576000u
#define CRTP_MAX_PAY    30u
#define WIRE_BODY_MAX   31u   /* CH(1) + DATA(30) */

/* Channel multiplexing on the 0xAA wire format.
 *
 *   CH=0  REPL / text — bidirectional, forwarded over the radio CRTP
 *         link on port 0x0E. Host PC interacts with the SentAI board
 *         via this channel.
 *   CH=1  Optical-flow measurement (board → drone only). Binary
 *         flow_pkt_t (16 bytes), consumed locally by the drone's EKF
 *         via estimatorEnqueueFlow(). Never forwarded to the radio.
 *   CH=2  Drone telemetry query (board ↔ drone). Single-byte cmd
 *         from board, drone replies with [cmd][float32]. See the
 *         telemetry-cmd table below for cmd codes. Cached log var
 *         IDs are resolved on first call so repeated queries are
 *         O(1) lookups in the log subsystem.
 *   CH=3  reserved
 */
#define CH_REPL    0u
#define CH_FLOW    1u
#define CH_TELEM   2u

/* Telemetry commands (channel 2). Reply on the same channel is always
 * `[cmd_echo:1][float32:4]` little-endian on the wire. Unknown cmds
 * reply with NaN so the board can flag them. */
#define TELEM_BARO_ASL     0x01u  /* baro.asl       — barometric altitude (m) */
#define TELEM_STATE_Z      0x02u  /* stateEstimate.z — fused altitude  (m)    */
#define TELEM_BATTERY_V    0x03u  /* pm.vbat         — battery voltage  (V)   */
#define TELEM_BATTERY_PCT  0x04u  /* pm.batteryLevel — battery level    (%)   */
#define TELEM_TEMP_C       0x05u  /* baro.temp       — barometer temp   (°C)  */
#define TELEM_PRESSURE     0x06u  /* baro.pressure   — pressure         (mbar)*/

typedef struct __attribute__((packed)) {
    float dpx;     /* accumulated pixel motion x since last sample */
    float dpy;     /* accumulated pixel motion y since last sample */
    float dt;      /* seconds elapsed for the accumulation window  */
    float std;     /* measurement standard deviation                */
} flow_pkt_t;

#define RX_TASK_NAME    "sentaiRx"
#define RX_TASK_STACK   256   /* 1024 bytes — generous to avoid overflow */
#define RX_TASK_PRIO    2     /* slightly above appMain so we drain UART2 */

/* Diagnostic counters (PARAM-exposed, single-writer per field). */
static bool     s_isInit                = false;
static uint32_t s_radio_to_uart_pkts    = 0;
static uint32_t s_radio_to_uart_drops   = 0;
static uint32_t s_uart_to_radio_pkts    = 0;
static uint32_t s_uart_crc_errors       = 0;
static uint32_t s_uart_bad_len          = 0;
static uint32_t s_uart_radio_drops      = 0;
static uint32_t s_flow_injected         = 0;
static uint32_t s_flow_rejected         = 0;
static uint32_t s_unknown_channel       = 0;
static uint32_t s_telem_queries         = 0;
static uint32_t s_telem_unknown_cmd     = 0;

/* ------------------------------------------------------------------ */
/*  Telemetry helpers.                                                */
/* ------------------------------------------------------------------ */

/* Quiet-NaN bit pattern, IEEE-754 binary32. Returned for unknown cmd
 * codes or unresolved drone log vars so the board side can detect
 * "no real value here" without ambiguity (NaN compares unequal to
 * itself). Builds via union — safe under -fstrict-aliasing. */
static inline float nan_f(void) {
    union { uint32_t u; float f; } w = { .u = 0x7FC00000u };
    return w.f;
}

/* Resolve a Bitcraze log var ID lazily, caching only on SUCCESS. If
 * the log subsystem hasn't fully initialized when the first query
 * arrives at boot, an early failure used to permanently cache the
 * not-found sentinel — every subsequent query returned NaN even
 * after the var became available. Now we retry on each call until
 * resolution succeeds (cheap: a hash-table lookup in log.c). */
static logVarId_t telem_resolve(logVarId_t* cached_slot,
                                const char* group, const char* name) {
    if (*cached_slot != 0xFFFF) return *cached_slot;
    logVarId_t id = logGetVarId(group, name);
    if (id != 0xFFFF) *cached_slot = id;   /* cache only on success */
    return id;
}

/* Resolve drone log var → float on demand. Returns NaN for unknown
 * cmd or unresolved log var. */
static float telem_read(uint8_t cmd) {
    static logVarId_t id_baro_asl    = 0xFFFF;
    static logVarId_t id_state_z     = 0xFFFF;
    static logVarId_t id_battery_v   = 0xFFFF;
    static logVarId_t id_battery_pct = 0xFFFF;
    static logVarId_t id_temp_c      = 0xFFFF;
    static logVarId_t id_pressure    = 0xFFFF;

    logVarId_t* slot; const char* group; const char* name;
    switch (cmd) {
        case TELEM_BARO_ASL:    slot=&id_baro_asl;    group="baro";          name="asl";          break;
        case TELEM_STATE_Z:     slot=&id_state_z;     group="stateEstimate"; name="z";            break;
        case TELEM_BATTERY_V:   slot=&id_battery_v;   group="pm";            name="vbat";         break;
        case TELEM_BATTERY_PCT: slot=&id_battery_pct; group="pm";            name="batteryLevel"; break;
        case TELEM_TEMP_C:      slot=&id_temp_c;      group="baro";          name="temp";         break;
        case TELEM_PRESSURE:    slot=&id_pressure;    group="baro";          name="pressure";     break;
        default:
            s_telem_unknown_cmd++;
            return nan_f();
    }
    logVarId_t id = telem_resolve(slot, group, name);
    if (id == 0xFFFF) return nan_f();
    return logGetFloat(id);
}

/* Build + send a [cmd_echo][float32 LE] reply on CH_TELEM. The
 * counter is bumped *before* uart2SendData so a wedged TX still
 * shows up in the stat as "attempted" — cleaner than the earlier
 * post-send increment which conflated "called" with "delivered".
 * For "delivered" diagnostics we'd need a TX-completion hook from
 * the bounded-uart2SendData refactor (TODO). */
static void telem_reply(uint8_t cmd, float value) {
    s_telem_queries++;

    union { float f; uint32_t u; } w = { .f = value };

    uint8_t frame[1 + 1 + 1 + 5 + 1];   /* AA LEN CH cmd float32 CRC */
    uint8_t idx = 0;
    frame[idx++] = WIRE_START;
    frame[idx++] = (uint8_t)(1 + 5);    /* CH + body = 6 */
    frame[idx++] = CH_TELEM;
    frame[idx++] = cmd;                 /* cmd echo */
    memcpy(&frame[idx], &w.u, 4);       /* float32 LE */
    idx = (uint8_t)(idx + 4);
    uint8_t crc = 0;
    for (uint8_t i = 0; i < idx; ++i) crc ^= frame[i];
    frame[idx++] = crc;

    uart2SendData(idx, frame);
}

/* ------------------------------------------------------------------ */
/*  RADIO → UART  (CRTP RX task callback, runs at high priority)      */
/* ------------------------------------------------------------------ */

static void on_radio_packet(CRTPPacket *pk) {
    if (pk == NULL) return;
    const uint8_t bodyLen = (uint8_t)pk->size;
    if (bodyLen > CRTP_MAX_PAY) {
        s_radio_to_uart_drops++;
        return;
    }
    /* Frame on stack — no shared buffer, no race. */
    uint8_t frame[1 + 1 + 1 + CRTP_MAX_PAY + 1];
    uint8_t idx = 0;
    frame[idx++] = WIRE_START;
    frame[idx++] = (uint8_t)(1u + bodyLen);                   /* CH + DATA */
    frame[idx++] = (uint8_t)(pk->channel & 0x03u);
    if (bodyLen > 0) {
        memcpy(&frame[idx], pk->data, bodyLen);
        idx = (uint8_t)(idx + bodyLen);
    }
    uint8_t crc = 0;
    for (uint8_t i = 0; i < idx; ++i) crc ^= frame[i];
    frame[idx++] = crc;

    uart2SendData(idx, frame);
    s_radio_to_uart_pkts++;
}

/* ------------------------------------------------------------------ */
/*  UART → RADIO  (dedicated FreeRTOS task drains UART2 RX)           */
/* ------------------------------------------------------------------ */

typedef enum {
    RX_WAIT_START = 0,
    RX_LEN,
    RX_CH,
    RX_DATA,
    RX_CRC,
} rx_state_t;

static void uart_rx_task(void *param) {
    (void)param;
    rx_state_t state = RX_WAIT_START;
    uint8_t   length    = 0;     /* CH + DATA bytes expected */
    uint8_t   buf_idx   = 0;     /* into frame_buf[0..length-1] */
    uint8_t   frame_buf[WIRE_BODY_MAX];   /* CH + DATA only */
    uint8_t   running_xor = 0;

    /* Block until system fully initialized — required by Bitcraze
     * deck-task pattern (see drivers/src/test/uart2test.c). Without
     * this, the task can spin against an uninitialized rxStream and
     * miss the first wave of incoming bytes. */
    systemWaitStart();
    DEBUG_PRINT("rx task running\n");
    uint32_t bytes_seen = 0;
    uint32_t last_log_ms = 0;

    while (1) {
        uint8_t b;
        /* Block-with-timeout so the task is scheduler-friendly even
         * during long quiet periods on UART. */
        if (uart2GetDataWithTimeout(1, &b, M2T(50)) == false) {
            uint32_t now_ms = T2M(xTaskGetTickCount());
            if (now_ms - last_log_ms >= 30000u) {
                DEBUG_PRINT("rx idle: bytes_seen=%lu\n",
                            (unsigned long)bytes_seen);
                last_log_ms = now_ms;
            }
            continue;
        }
        bytes_seen++;

        switch (state) {
        case RX_WAIT_START:
            if (b == WIRE_START) {
                running_xor = b;
                state = RX_LEN;
            }
            break;

        case RX_LEN:
            if (b == 0u || b > WIRE_BODY_MAX) {
                s_uart_bad_len++;
                state = RX_WAIT_START;
                break;
            }
            length = b;
            running_xor ^= b;
            buf_idx = 0;
            state = RX_CH;
            break;

        case RX_CH:
            frame_buf[0] = b;             /* channel */
            running_xor ^= b;
            buf_idx = 1;
            state = (buf_idx >= length) ? RX_CRC : RX_DATA;
            break;

        case RX_DATA:
            if (buf_idx < WIRE_BODY_MAX) {
                frame_buf[buf_idx] = b;
            }
            running_xor ^= b;
            buf_idx++;
            if (buf_idx >= length) state = RX_CRC;
            break;

        case RX_CRC:
            if (b != running_xor) {
                s_uart_crc_errors++;
                state = RX_WAIT_START;
                break;
            }
            /* Frame valid — dispatch on channel.
             *   CH_REPL → wrap CRTPPacket and ship over the radio link.
             *   CH_FLOW → parse flow_pkt_t, inject into the EKF locally.
             *             NEVER forwarded over radio — flow is a tight
             *             estimator-only signal, the host has no use
             *             for it on the radio link.                    */
            {
                const uint8_t channel = frame_buf[0] & 0x03u;
                const uint8_t dataLen = (uint8_t)(length - 1);

                if (channel == CH_REPL) {
                    CRTPPacket out = {0};
                    out.port    = LINK_PORT;
                    out.channel = channel;
                    out.size    = dataLen;
                    if (dataLen > 0) {
                        memcpy(out.data, &frame_buf[1], dataLen);
                    }
                    if (crtpSendPacket(&out) == 0) {
                        s_uart_to_radio_pkts++;
                    } else {
                        s_uart_radio_drops++;
                    }
                } else if (channel == CH_FLOW) {
                    if (dataLen != sizeof(flow_pkt_t)) {
                        s_flow_rejected++;
                    } else {
                        flow_pkt_t fp;
                        memcpy(&fp, &frame_buf[1], sizeof(fp));
                        /* Sanity-reject NaNs/out-of-range so the EKF
                         * never sees garbage. */
                        if (fp.dt <= 0.0f || fp.dt > 1.0f
                            || fp.std <= 0.0f || fp.std > 100.0f
                            || fp.dpx != fp.dpx || fp.dpy != fp.dpy) {
                            s_flow_rejected++;
                        } else {
                            flowMeasurement_t fm = {0};
                            fm.dpixelx = fp.dpx;
                            fm.dpixely = fp.dpy;
                            fm.stdDevX = fp.std;
                            fm.stdDevY = fp.std;
                            fm.dt      = fp.dt;
                            estimatorEnqueueFlow(&fm);
                            s_flow_injected++;
                        }
                    }
                } else if (channel == CH_TELEM) {
                    /* Single-byte cmd request, drone replies with
                     * [cmd][float32]. Tighten to exact length (1) so
                     * future protocol additions don't silently mask
                     * malformed queries; bump unknown-cmd counter on
                     * mismatch to make it observable. */
                    if (dataLen == 1) {
                        uint8_t cmd = frame_buf[1];
                        float v = telem_read(cmd);
                        telem_reply(cmd, v);
                    } else {
                        s_telem_unknown_cmd++;
                    }
                } else {
                    s_unknown_channel++;
                }
            }
            state = RX_WAIT_START;
            break;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Deck driver core                                                  */
/* ------------------------------------------------------------------ */

static void sentaiInit(DeckInfo *info) {
    (void)info;
    if (s_isInit) return;
    DEBUG_PRINT("init: CRTP port 0x%02X, UART2 @ %u baud\n",
                LINK_PORT, (unsigned)UART_BAUDRATE);

    uart2Init(UART_BAUDRATE);
    crtpRegisterPortCB(LINK_PORT, on_radio_packet);
    xTaskCreate(uart_rx_task, RX_TASK_NAME, RX_TASK_STACK,
                NULL, RX_TASK_PRIO, NULL);

    s_isInit = true;
}

static bool sentaiTest(void) {
    return s_isInit;
}

static const DeckDriver sentaiDeck = {
    .name       = "sentai",
    .usedPeriph = DECK_USING_UART2,
    .init       = sentaiInit,
    .test       = sentaiTest,
};

DECK_DRIVER(sentaiDeck);

/* Diagnostic params — visible from cfclient PARAM tab. */
PARAM_GROUP_START(deck)
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, sentaiR2U,    &s_radio_to_uart_pkts)
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, sentaiR2Udrp, &s_radio_to_uart_drops)
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, sentaiU2R,    &s_uart_to_radio_pkts)
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, sentaiU2Rdrp, &s_uart_radio_drops)
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, sentaiUcrc,   &s_uart_crc_errors)
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, sentaiUbad,   &s_uart_bad_len)
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, sentaiTelem,  &s_telem_queries)
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, sentaiTelBad, &s_telem_unknown_cmd)
PARAM_GROUP_STOP(deck)
