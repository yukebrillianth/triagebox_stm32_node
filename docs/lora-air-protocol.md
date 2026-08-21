# TriageBox LoRa air protocol — station ⇄ STM32 node

The station half and the node half are both written as of 2026-08-18; neither has
met the other on hardware yet. Every claim here is either in `lora_poll.h` /
`lora_vital.h` (both copied verbatim into both projects) or in
`main/sx1278.c` / `main/station_poll.c` on the station and `ServiceLoRaPoll()` in
`Core/Src/main.c` on the node.

Written 2026-08-18. If this and the headers disagree, the headers win — but say
so, because it means one of the two copies is stale.

## The one thing that changed

**The node no longer transmits on its own.** It used to send a vital every 750 ms
whether anyone was listening or not. That is now removed: the node sits in
receive, and answers exactly once each time the station asks it to.

Free-running nodes on a shared channel collide, and the collision probability
grows with the square of the node count. At the 20-node target most packets would
be lost, and lost invisibly — a collided frame fails CRC and simply never
arrives, so it looks like flaky hardware rather than a design problem. Polling
makes the channel deterministic: one radio transmits at a time because the
station said so.

This was decided with the tradeoff on the table. The cost is latency, and it is
accepted: see "Urgent readings" below.

## Radio configuration — must match exactly

| Parameter | Value |
|---|---|
| Frequency | 433 MHz |
| Spreading factor | SF7 |
| Bandwidth | 125 kHz |
| Coding rate | 4/5 |
| Preamble | 8 symbols |
| Header | explicit |
| CRC | on |
| Sync word | 0x12 (the SX1278 reset default) |
| TX power | 20 dBm, PA_BOOST |
| OCP | 100 mA |

These are the node library's `newLoRa()` **constructor defaults**, which is where
they came from — the station was configured to match the node, not the other way
round. Two consequences:

The `#define LORA_FREQUENCY 433000000u` in the node's `main.c` is dead code. The
library takes frequency in **MHz** (`frequency = 433`), so that define is never
read. Deleting it is safe; "fixing" the library to read it is not, because the
station is now aligned to 433 MHz either way.

Neither end writes the sync word. Both rely on the 0x12 reset default, so they
agree by doing nothing. Do not write 0x34 — that is the LoRaWAN value and it is
the classic way to make two correctly configured radios never hear each other.

A single mismatched parameter yields **silence, not an error**. If the link does
not work, dump `RegModemConfig1/2/3` and `RegSyncWord` on both ends and diff them
before suspecting anything in the protocol.

## Downlink: the poll (station → node, 5 bytes)

`lora_poll_t` from `Core/Inc/lora_poll.h`, packed, little-endian:

| Offset | Field | Value |
|---|---|---|
| 0 | `magic` | always 0x00 |
| 1 | `version` | `LORA_POLL_VERSION`, currently 0x01 |
| 2 | `station_id` | which station is asking, 1..254 |
| 3 | `node_id` | who must answer, 1..20 |
| 4 | `command` | `LORA_POLL_CMD_REPORT` = 0x01 |

Use `lora_poll_for_me(p, len, my_id)` rather than hand-rolling the checks. It
verifies the length, that `my_id` is non-zero, the magic byte and the version
before comparing the node id.

**Node ids are 1-based and 0 is reserved.** Byte 0 of a poll is always 0 and byte
0 of a vital is the sender's node id, so one byte at a fixed offset tells downlink
from uplink with no length test and no guessing. Every node hears every other
node's reply on this shared channel, so cheap rejection matters. Do not assign
node id 0 to real hardware — and note that `lora_poll_for_me()` returns false for
`me == 0` rather than matching, so a node misconfigured that way goes quiet
instead of answering polls addressed to everyone.

**Ignore unknown `command` values silently.** Do not NAK — a NAK from every node
at once is itself a collision. The byte exists so "sync your clock" or "restart"
can be added later without a version bump on every deployed node.

**Ignore a poll whose `version` differs.** That means one side has a stale copy of
the header, and acting on a packet you cannot parse is worse than dropping it.

`station_id` is a radio address (`CONFIG_TB_STATION_ADDR`, default 1) and is
**not** the MQTT topic segment (`st-01`). The backend never sees it. A node may
accept polls from any station, or filter on this field if two stations ever share
a channel — the station does not care which, so this is the node's call.

## Uplink: the reply (node → station, 18–38 bytes)

`lora_vital_t` from `Core/Inc/lora_vital.h`, unchanged from the free-running
design — only *when* it is sent has changed, not what it contains. Transmit
`lora_vital_len(&v)` bytes: the 18-byte fixed part plus however many characters
of RFID tag there are, which is usually zero.

The station validates with `lora_vital_valid()` and drops anything that fails, so
the fields that must be right are `version` (0x01) and `victim_rfid_len` (≤ 20 and
consistent with the length actually transmitted).

`node_id` in the reply must be the node's own address. **The station checks it
against the address it polled and drops the packet if they differ** — a reply that
arrives late lands in the next node's slot, and publishing it under the polled id
would attribute one patient's vitals to another. That check is the reason the
field is there.

`priority` 0xFF means "not scored yet"; the station omits the JSON key rather than
publishing a level. Do not send 0 for unscored, because 0 is BLACK. Same for
`battery`: 0xFF means "no fuel gauge", 0 means a flat battery.

`flags` decides which vitals are published at all. The station tests
`TB_FLAG_HR_VALID`, `TB_FLAG_SPO2_VALID`, `TB_FLAG_RR_VALID` and
`TB_FLAG_BP_VALID` and omits the key when the bit is clear. A stale reading with
the flag set is published as current; a fresh reading with the flag clear is
thrown away. Set them honestly.

## Timing — the part that bites

```
station:  [--- poll ~30ms ---]                          [--- next poll ---]
node:                          (snapshot + turnaround)
node:                            [------ reply 51-82ms ------]
                          |<----------- 250ms slot ----------->|
```

`LORA_POLL_SLOT_MS` is 250 ms and the station starts counting when its own
transmit completes. Inside that window the node must switch from RX to TX, take
its measurement snapshot, and get the whole packet into the air.

**The node's own reply deadline must stay strictly under 250 ms.** A late reply
does not merely miss its own slot — it lands during the *next* node's slot and
takes that node's reply down with it. One slow node degrades its neighbour, which
is a genuinely confusing failure to debug. If a snapshot cannot be taken in time,
send the previous values with their validity flags as they stand; do not stretch
the slot.

250 ms is roughly double the measured worst case, deliberately. A slot that is too
tight fails as a silent timeout indistinguishable from a dead node.

A full cycle is `LORA_POLL_PERIOD_MS` = 15 s regardless of how many nodes answer,
so the node can expect to be polled every 15 s. Do not depend on that for
anything: if the station reboots or its config changes, the gap changes.

## Two traps the node implementation hit

Both cost nothing to avoid and are invisible once wrong, so they are recorded
here rather than only in the node's source.

**Never call the receive function speculatively.** The node's LoRa library
(`LoRa_receive()`) drops the modem to standby to inspect `RegIrqFlags`, and a
mode change *aborts a reception already in progress*. Called once per superloop
pass — every ~10 ms — against a ~30 ms poll frame, it would abort almost every
poll mid-air. Both radios would be configured perfectly and the link would look
dead. The node therefore reads **DIO0 as a level** first and only touches the
radio once a whole packet has landed: `RegDioMapping1` bits 7:6 = 00 is RxDone in
the RX modes, the flag latches until cleared, and `EXTI1` is not enabled in the
NVIC, so polling the level is both sufficient and cheaper than an edge. Any
library with the same standby-then-peek shape has the same trap.

**Check the CRC flag yourself if the library does not.** That library clears the
whole flags register without ever testing `PayloadCrcError` (bit 5), so a corrupt
frame is handed back as if it were good. The node tests bit 5 before reading, and
counts it separately: silence means nobody transmitted, a CRC error means somebody
did and the link is marginal, which is the one an antenna fixes. The station makes
the same distinction (`ESP_ERR_INVALID_CRC`).

## A node may decline to answer

The node measures how long a poll may have sat unnoticed — bounded by the
interval between two consecutive DIO0 checks, i.e. one superloop pass — and if
that exceeds `LORA_REPLY_DEADLINE_MS` (150 ms in `Core/Src/main.c`) it **does not
reply at all**. 150 + ~90 ms of reply airtime stays inside the 250 ms slot; a
reply built on a staler poll might not.

From the station this is indistinguishable from a miss, and that is the intended
trade: a missed slot costs one reading, while a late reply lands in the next
node's slot and costs a second node's reading too. The node counts these in
`mon_lora_stale`, which is the only place that says the cause was the node's own
latency rather than a dead link — worth asking for if a node looks intermittently
absent. The known cause of a slow pass is an RFID scan, which blocks ~120 ms
inside `Pn532_Service`.

## Newest reading only, no history

A poll returns **one packet with the newest reading**. Nodes keep no ring buffer.
A reading taken between polls is overwritten and lost, and that is correct: a
15-second-old vital has no clinical value once a newer one exists, and buffering
would need sequence numbers, a gap-recovery rule and multi-packet replies for data
nobody reads.

`packet_counter` still increments once per transmit and wraps at 65535, so the
backend can see that a reply was lost even though the reading itself is gone.

If gap-free history is ever wanted, that is a version bump plus a record-count
field and a multi-packet reply — a protocol change, not a tweak. Ask before
building toward it.

## Urgent readings

**A freshly scored RED or BLACK does not get to jump the queue.** Nodes never
transmit unpolled, no exceptions. An unsolicited packet can land on top of
whichever node is currently answering and destroy both frames, so one urgent
patient would cost another patient's reading.

Worst-case latency for an urgent result is therefore one full cycle, about 15 s.
This was offered as an explicit either/or and strict polling was chosen. Do not
add a "just this once" fast path on the node — it will work perfectly in a
two-node bench test and fail at deployment scale, which is the worst possible
place for it to fail.

## Airtime budget

At the settings above: a poll is ~30 ms, an empty vital ~51 ms, a vital with a
20-character tag ~82 ms. A 20-node cycle is about 5 s of traffic inside a 15 s
period, so the station transmits roughly 4% duty and each node well under 1% —
comfortably under the 10% ceiling common on this band.

Adding a field to `lora_poll_t` costs airtime 20 times per cycle. Weigh new
downlink fields against that, not against the reply.

## What the station does with a missing reply

Three consecutive misses (`LORA_POLL_MISS_LIMIT`) and the node is published as
`{"status":"OFFLINE"}` on MQTT — about 45 s of silence. One miss is normal on a
radio link, so the station does not flap on a single dropped frame. A CRC failure
counts as a miss but is logged differently: silence means nobody answered, a CRC
error means somebody did and the link is marginal.

Nothing else happens. There is no retry inside a cycle and no backoff. The node
does not need to know or care whether its last reply arrived.

## Node checklist

Done on this node as of 2026-08-18, in `ServiceLoRaPoll()` in `Core/Src/main.c`.
Kept as a checklist because it is what a second node type would have to satisfy.

1. Delete the free-running transmit and `NODE_REPORT_INTERVAL`.
2. Sit in RX continuous. Do not set the sync word or the frequency define.
3. On a packet, `lora_poll_for_me(p, len, MY_NODE_ID)`. False → back to RX, no log.
4. Unknown command → back to RX, silently.
5. `LORA_POLL_CMD_REPORT` → snapshot, fill `lora_vital_t`, `node_id` = own id,
   `version` = 0x01, flags set honestly, unscored priority = 0xFF, no gauge
   battery = 0xFF.
6. Transmit `lora_vital_len(&v)` bytes, well inside 250 ms of the poll arriving.
7. Straight back to RX. Never transmit from anywhere else in the firmware.

## Bringing the link up for the first time

Neither half has seen the other, so expect the first attempt to fail silently —
that is what a radio mismatch looks like.

What *has* been verified is only the node's control flow: `ServiceLoRaPoll()` is
lifted out of `main.c` by `tools/run_selftests.sh` and run on the host against a
scripted fake radio, so the accept/reject/decline decisions, the CRC path and the
first-pass case are all executed and asserted. Nothing about SPI, timing on real
silicon, or the air itself is covered by that, and `main.c` has never been
compiled for ARM in this environment. Treat the protocol logic as tested and
everything physical as untested.

The node exposes five counters over SWD in STM32CubeMonitor (all `mon_lora_*`),
and they localise the fault without a spectrum analyser:

| Symptom | Meaning |
|---|---|
| `rx` == 0 | nothing heard at all. Station not transmitting, or the radios disagree on frequency/SF/BW/CR/sync word. Dump `RegModemConfig1/2/3` and `RegSyncWord` on both ends and diff them. |
| `rx` climbing, `polls` == 0 | frames arrive but none are ours. `NODE_ID` outside the range the station polls (`CONFIG_TB_NODE_COUNT`), a stale `lora_poll.h` on one side, or simply other nodes' replies. |
| `crc` climbing | somebody is transmitting and the link is marginal. Antennas, distance, supply. |
| `stale` climbing | polls arrive but the superloop notices them too late to answer in the slot. A node problem, not a link problem. |
| `polls` > `frames` + `stale` | the reply transmit is timing out. Check the antenna is attached — TX into an open PA can hang. |
| `reply_ms` near 250 | the timing contract is about to break. Fix the superloop, not the slot. |

On the station side the matching signals are in the log: silence produces no
message at all (an absent node is expected), `rx node N: ESP_ERR_INVALID_CRC`
means marginal, and `polled N, node M answered` means a node is replying into
someone else's slot.
