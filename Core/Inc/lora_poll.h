#ifndef LORA_POLL_H
#define LORA_POLL_H

#include <stdint.h>

#include "tb_regs.h" /* TB_PACKED -- shared, not copied */

/*
 * LoRa downlink: receiving station -> STM32F411 node. The poll that asks one
 * node for its newest vital.
 *
 * ONE SOURCE OF TRUTH, TWO COPIES, same rule as tb_regs.h and lora_vital.h:
 * platform-neutral, meant to be copied verbatim into the station project.
 * LORA_POLL_VERSION exists so a stale copy is rejected, not misparsed.
 *
 * WHY POLLED AT ALL. Free-running nodes on one channel collide, and the
 * collision probability grows with the square of the node count -- at 20 nodes
 * transmitting whenever they liked, most packets would be lost and the loss
 * would be invisible (a corrupted frame fails CRC and simply never arrives).
 * Polling makes the channel deterministic: exactly one radio transmits at a
 * time because the station tells it to. The cost is latency, bounded by one
 * cycle. Decided 2026-08-18; the node's previous free-running transmit was
 * removed the same day.
 *
 * A POLL ASKS FOR THE NEWEST READING, ONE PACKET, NOT A HISTORY. Nodes keep no
 * ring buffer: a reading taken between polls is overwritten and lost, which is
 * correct for triage -- a 15-second-old vital has no clinical value once a
 * newer one exists, and buffering would need sequence numbers, a gap-recovery
 * rule and multi-packet replies for data nobody reads. Chosen deliberately
 * 2026-08-18. If gap-free history is ever wanted, that is a version bump and a
 * record-count field, not a tweak.
 *
 * NODES NEVER TRANSMIT UNPOLLED -- no exceptions, including a freshly scored
 * RED or BLACK patient. An unsolicited packet can land on top of whichever node
 * is answering its poll and destroy both. The worst-case latency for an urgent
 * result is therefore one full cycle.
 */

/* Bump on ANY layout change. Both ends must check this before acting. */
#define LORA_POLL_VERSION 0x01U

/*
 * Byte 0 of a poll is always 0, and byte 0 of a vital is the sender's node_id
 * -- which is why NODE IDS ARE 1-BASED AND 0 IS RESERVED. Both directions share
 * one frequency and one sync word, so a node sitting in receive hears the
 * replies of whichever node is currently being polled, and the station hears
 * nothing else but must still reject its own protocol's downlink if a stray
 * copy arrives. One byte at a fixed offset separates the two without a length
 * test or a guess. Do not assign node_id 0 to a real node.
 */
#define LORA_POLL_MAGIC 0x00U

/** Nodes per cycle. 20 is the deployment target, and the slot budget below is
 * sized so a full cycle of 20 still fits comfortably inside the period. */
#define LORA_POLL_NODE_MAX 20U

/*
 * ponytail: one command exists. The byte is here because a downlink with no
 * command field is not a protocol, it is a hardcoded message -- adding "sync
 * your clock" or "restart" later would otherwise force a version bump on every
 * node in the field. Unknown commands MUST be ignored silently by the node
 * (not NAKed: a NAK from every node at once is a collision).
 */
#define LORA_POLL_CMD_REPORT 0x01U

/*
 * Five bytes, and the size is the point: at the shared SF7 / 125kHz / CR4-5 /
 * 8-symbol preamble this is ~30ms of airtime, against ~51-82ms for the vital
 * reply. A station polling 20 nodes therefore spends ~600ms of every cycle
 * transmitting -- 4% duty at the 15s period below, under the 10% ceiling that
 * is common on this band, with the nodes contributing well under 1% each.
 * Adding fields here costs airtime 20 times per cycle, so weigh them against
 * that rather than against the reply.
 */
typedef struct TB_PACKED {
	uint8_t magic; /**< LORA_POLL_MAGIC; distinguishes downlink from uplink */
	uint8_t version; /**< LORA_POLL_VERSION; ignore the packet if it differs */
	uint8_t station_id; /**< which station is asking; 1..254 */
	uint8_t node_id; /**< who must answer; 1..LORA_POLL_NODE_MAX */
	uint8_t command; /**< LORA_POLL_CMD_*; ignore unknown values silently */
} lora_poll_t;

#define LORA_POLL_LEN 5U

/*
 * TIMING CONTRACT. The node must have its reply in the air before the station
 * gives up, so these two live here rather than in either firmware:
 *
 *   SLOT_MS   one poll plus one reply plus turnaround. ~30ms out, ~82ms back
 *             worst case, plus the node's RX->TX switch and its I2C snapshot.
 *             250ms is roughly double the measured worst case, because a slot
 *             that is too tight fails as a silent timeout that looks exactly
 *             like a dead node.
 *   PERIOD_MS one full cycle. 20 nodes x 250ms = 5s of traffic, so a 15s period
 *             is two thirds idle -- deliberate headroom, and it matches the
 *             nominal MQTT vital cadence one-for-one, so the station publishes
 *             exactly what it polls with no decimation anywhere.
 *
 * The node's reply timeout must stay strictly under SLOT_MS: a reply that
 * arrives late does not merely miss, it lands during the NEXT node's slot and
 * takes that node's reply down with it.
 */
#define LORA_POLL_SLOT_MS 250U
#define LORA_POLL_PERIOD_MS 15000U

/** Consecutive missed polls before a node is reported OFFLINE on MQTT. Three
 * tolerates a lost packet or two -- one miss is normal on a radio link and
 * flapping a node's status on every dropped frame is noise, not information. */
#define LORA_POLL_MISS_LIMIT 3U

/** Is this a poll addressed to @p me? Length-checked first, so a truncated
 * frame is rejected rather than read past. Station-independent on purpose:
 * the caller decides whether it accepts polls from more than one station.
 *
 * @p me == 0 never matches. Node id 0 is reserved as the magic byte, so a node
 * misconfigured as 0 would otherwise treat every poll on the channel -- for any
 * node -- as its own and transmit into other nodes' slots. Failing closed makes
 * that node look dead instead, which is the harmless direction: it costs one
 * node's readings rather than corrupting the whole cycle. */
static inline int lora_poll_for_me(const lora_poll_t *p, uint8_t len, uint8_t me)
{
	return (len >= LORA_POLL_LEN) && (me != 0U)
			&& (p->magic == LORA_POLL_MAGIC)
			&& (p->version == LORA_POLL_VERSION) && (p->node_id == me);
}

#endif /* LORA_POLL_H */
