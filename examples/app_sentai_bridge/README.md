# SentAI deck driver

Crazyflie firmware deck driver that bridges the radio CRTP link with a
Coral Dev Board Micro ("SentAI") attached on UART2. Lets a host PC
talk to the on-board MicroPython REPL over Crazyradio while reserving
a fast UART path for board → drone control (flow injection, future
setpoint commands).

This is a complete **deck driver** (not an `app_*` firmware app). It
plugs into Crazyflie's deck-core boot sequence so:

* `init` runs at deck-init time, before user-space scheduler tasks. No
  interference with the radio CRTP TOC handshake (which previously
  starved out our app-firmware variants and left `LED_RED_R` dark).
* The CRTP RX callback runs in the high-priority CRTP RX task itself.
  Bounded body, stack-only buffer, `uart2SendData` is DMA-driven — no
  way to wedge the radio scheduler.
* Resource ownership is declared via `.usedPeriph = DECK_USING_UART2`.

## Wire format on UART2

```
+------+-----+----+--------+-----+
| 0xAA | LEN | CH | DATA…  | CRC |
+------+-----+----+--------+-----+
```

* `0xAA` start byte (intentionally distinct from CPX's `0xFF`).
* `LEN`  uint8, value = 1 + DATA bytes. Range 1..31.
* `CH`   uint8, channel multiplexing (see below).
* `DATA` ≤30 bytes (CRTP MAX_PAYLOAD).
* `CRC`  XOR of every byte before it, including `0xAA` and `LEN`.

No flow control, no ack-per-packet, no CTS/CTR state machine. The
link is point-to-point with the SentAI deck; on a byte loss the next
`0xAA` resyncs.

### Channel multiplexing

| CH | Purpose                                       | Direction       | Drone behavior |
|----|-----------------------------------------------|-----------------|----------------|
| 0  | REPL / text traffic                           | bidirectional   | forward to/from radio CRTP port `0x0E` |
| 1  | Optical-flow measurement (`flow_pkt_t`)       | board → drone   | `estimatorEnqueueFlow()` locally; **never** echoed to radio |
| 2  | reserved (planned: CRTP setpoint injection)   | —               | — |
| 3  | reserved                                      | —               | — |

`flow_pkt_t` is a packed 16-byte struct: `float dpx, dpy, dt, std`.
NaNs and out-of-range values are dropped before reaching the EKF.

## Routing on the radio side

Host PC uses CRTP port `0x0E` (`LINK_PORT`) for all SentAI bridge
traffic. Channel matches the wire-format CH above (only CH=0 is
bridged to the radio side).

```python
from cflib.crtp.crtpstack import CRTPPacket
pk = CRTPPacket()
pk.port = 0x0E
pk.channel = 0
pk.data = b'>>> sentai.io.led_on()'
cf.send_packet(pk)
cf.add_port_callback(0x0E, on_reply)   # receives board → drone → radio
```

Standard Crazyflie ports (commander, log, param, supervisor, …)
continue to work normally over radio — they never touch UART2.

## Build & flash

```bash
cd examples/app_sentai_bridge
make clean && make           # produces build/cf21bl.bin (or cf2.bin)
cfloader flash build/cf21bl.bin stm32-fw
```

The deck is force-loaded via `CONFIG_DECK_FORCE="sentai"` in
`app-config` because the SentAI board has no 1-wire memory.

## Diagnostic params

Visible from `cfclient → PARAM` tab under group `deck`:

| Param           | Meaning                                       |
|-----------------|-----------------------------------------------|
| `sentaiR2U`     | radio→UART packets forwarded                  |
| `sentaiR2Udrp`  | radio→UART drops (oversized / queue full)     |
| `sentaiU2R`     | UART→radio packets forwarded                  |
| `sentaiU2Rdrp`  | UART→radio drops (CRTP TX queue full)         |
| `sentaiUcrc`    | UART RX CRC errors                            |
| `sentaiUbad`    | UART RX bad-length frames                     |
| `sentaiFlow`    | flow measurements injected into EKF           |
| `sentaiFlowDrp` | flow packets rejected (bad size / NaN / range)|

## Why a deck driver and not an `app_*` firmware app?

Empirical: with `CONFIG_APP_ENABLE=y` and Appchannel polling in
`appMain` (low priority), the radio CRTP TOC handshake at
`open_link()` times out and `SYS_LED` (LED_RED_R) goes dark — the
STM32 scheduler can't keep up. Deck drivers run in the high-priority
CRTP RX task path so the radio scheduler is never starved. This was
verified by switching the implementation from `app_appchannel_test`
style to deck-driver pattern — the symptoms disappeared completely.

## Counterpart on the SentAI board

See [examples/sentai_runtime/sentai_crazy.cc](https://github.com/...)
in the [coralmicro fork](https://github.com/...) — it implements the
peer side of the 0xAA wire format and exposes:

```python
sentai.crazy.init()                        # configure UART2 @ 576000 baud
sentai.crazy.poll_event(timeout_ms=100)    # read incoming bytes (channel 0)
sentai.crazy.link_send(channel, data)      # send back over the link
```
