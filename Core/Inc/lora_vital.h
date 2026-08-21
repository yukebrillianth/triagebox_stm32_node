#ifndef LORA_VITAL_H
#define LORA_VITAL_H

#include <stddef.h> /* NULL, returned by lora_vital_priority_name */
#include <stdint.h>

#include "tb_regs.h" /* TB_PACKED, TB_FLAG_*, TB_SENSOR_* -- shared, not copied */

/*
 * LoRa uplink: STM32F411 node -> receiving station.
 *
 * ONE SOURCE OF TRUTH, TWO COPIES, same rule as tb_regs.h: this file is
 * platform-neutral (no HAL, no malloc, no float) and is meant to be copied
 * verbatim into the station project alongside tb_regs.h, which it needs for the
 * validity and sensor-health bit definitions. LORA_VITAL_VERSION exists so a
 * stale copy is rejected instead of silently misparsed.
 *
 * SHAPED TO MATCH THE MQTT VITAL, FIELD FOR FIELD. The station's only job with
 * this packet is to turn it into the canonical vital JSON, so every field here
 * is named after the JSON key it becomes and is already in that key's units.
 * The station renames nothing and converts nothing except priority (number to
 * string) and confidence (percent to 0..1) -- both of which have a helper below
 * so neither conversion gets re-derived by hand.
 *
 *   {"victim_rfid","hr","spo2","rr","bp_sys","bp_dia","battery",
 *    "priority","confidence","reasons","ts"}
 *
 * NOT ON THE WIRE, DELIBERATELY:
 *   ts        stamped by the station on receipt. A node with no RTC would send
 *             a boot-relative time, which is worse than no time at all.
 *   reasons   always [] -- the scoring that would populate it happens on the
 *             ESP32, which reports a priority and a confidence, not reasons.
 *   station_id / node_id-as-JSON
 *             these live in the MQTT topic, not the payload. node_id is still
 *             on the LoRa wire below, because one station radio hears several
 *             nodes and has to know which topic to publish under.
 *   rssi, snr belong to the node *status* message and are measured by the
 *             station's own radio, so the node cannot send them anyway.
 */

/* Bump on ANY layout change. The station must check this before parsing. */
#define LORA_VITAL_VERSION 0x01U

/*
 * Up to 20 characters, not TB_RFID_MAX's 31: the only source is a PN532 UID
 * rendered as hex, and ISO14443-3 caps a UID at 10 bytes. main.c asserts that
 * against PN532_UID_MAX at compile time. The I2C link still carries 31 because
 * the ESP32 may one day publish a longer identifier of its own.
 */
#define LORA_VITAL_RFID_MAX 20U

/*
 * priority uses the LoRa numeric alias -- 0=BLACK 1=RED 2=YELLOW 3=GREEN --
 * which is NOT the ESP32's ui_priority_t declaration order. It arrives here
 * already converted (the ESP32 calls tb_frame_priority_to_wire() before writing
 * TB_REG_PRIORITY), so the node passes it through untouched.
 *
 * NONE is 0xFF and not 0 because 0 is BLACK. A packet transmitted before the
 * ESP32 has scored anything would otherwise report every patient as dead, which
 * is the one failure mode in this protocol that could actually hurt someone.
 * The station MUST omit "priority" and "confidence" from the JSON when it sees
 * this value rather than defaulting them.
 */
#define LORA_VITAL_PRIORITY_BLACK  0x00U
#define LORA_VITAL_PRIORITY_RED    0x01U
#define LORA_VITAL_PRIORITY_YELLOW 0x02U
#define LORA_VITAL_PRIORITY_GREEN  0x03U
#define LORA_VITAL_PRIORITY_NONE   0xFFU

/* battery: percent, or this if the board has no fuel gauge (it currently does
 * not). The station emits null / omits the key; it must not publish 0, which
 * reads as a flat battery. Same convention as TB_REG_BATTERY. */
#define LORA_VITAL_BATTERY_NONE 0xFFU

/*
 * The fixed part, up to but not including victim_rfid. A packet is
 * LORA_VITAL_FIXED_LEN + victim_rfid_len bytes -- see lora_vital_len().
 *
 * WHY THE TAG IS A VARIABLE-LENGTH TAIL: airtime. At the configured SF7 /
 * 125kHz / CR4-5 / 8-symbol preamble, a packet costs about 51ms with no tag,
 * 82ms with a 20-character one, and 98ms if the 31-byte field were always sent
 * in full. Each node's own duty cycle is negligible under polling (one reply per
 * LORA_POLL_PERIOD_MS), but the channel is shared: 20 replies plus 20 polls per
 * cycle occupy ~2.2s of the 15s period at 82ms, and ~2.6s if the field were
 * always full -- so the empty case, which is most of them, is the one worth
 * keeping cheap, and it is the station's slot budget that pays for the rest. The
 * station pays nothing for the variable length itself: the SX1278 explicit
 * header carries the payload length, so the receiver already knows how many
 * bytes arrived before it looks at the struct.
 */
#define LORA_VITAL_FIXED_LEN 18U

/*
 * Little-endian, packed, uint16_t fields at even offsets, so both ends can cast
 * this straight over the received bytes. Field order is chosen for that
 * alignment, not for readability -- the JSON key order is the comment above.
 */
typedef struct TB_PACKED {
	uint8_t node_id; /**< becomes the {node_id} topic segment, not a JSON key */
	uint8_t version; /**< LORA_VITAL_VERSION; reject the packet if it differs */
	uint16_t packet_counter; /**< ++ per transmit, wraps; -> "packet_counter" */
	uint8_t flags; /**< TB_FLAG_*: which vitals below are real */
	uint8_t device_status; /**< TB_SENSOR_* bitmask -> "device_status" */
	uint16_t hr; /**< bpm -> "hr" */
	uint8_t spo2; /**< percent -> "spo2" */
	uint8_t rr; /**< WHOLE breaths/min -> "rr"; the I2C link uses tenths */
	uint16_t bp_sys; /**< mmHg -> "bp_sys" */
	uint16_t bp_dia; /**< mmHg -> "bp_dia" */
	uint8_t battery; /**< percent or LORA_VITAL_BATTERY_NONE -> "battery" */
	uint8_t priority; /**< LORA_VITAL_PRIORITY_* -> "priority" (as a string) */
	uint8_t confidence; /**< 0..100 -> "confidence" (as 0..1) */
	uint8_t victim_rfid_len; /**< characters below; 0 = no tag scanned */
	char victim_rfid[LORA_VITAL_RFID_MAX]; /**< ASCII hex, NOT NUL-terminated */
} lora_vital_t;

/*
 * VALIDITY IS A BITMASK, NOT A MAGIC ZERO, for hr/spo2/rr/bp. The canonical
 * JSON treats 0 as "no reading" for hr and spo2, and that is fine for those,
 * but bp_sys/bp_dia are structurally present and permanently 0 on this hardware
 * -- nothing measures pressure. A station that published 0/0 as a reading would
 * be inventing a blood pressure. Check TB_FLAG_BP_VALID (clear today) and omit
 * the keys, and prefer the flags over the zero test for the others too.
 */

/** Bytes to transmit for @p v. Clamps a bad length rather than reading past the
 * tag, since victim_rfid_len is what a malformed sender gets wrong first. */
static inline uint8_t lora_vital_len(const lora_vital_t *v)
{
	uint8_t n = v->victim_rfid_len;

	if (n > LORA_VITAL_RFID_MAX) {
		n = LORA_VITAL_RFID_MAX;
	}
	return (uint8_t) (LORA_VITAL_FIXED_LEN + n);
}

/** Station side: the JSON string for a wire priority, or NULL for NONE and for
 * anything out of range. NULL means omit the key -- never substitute a level. */
static inline const char *lora_vital_priority_name(uint8_t wire)
{
	switch (wire) {
	case LORA_VITAL_PRIORITY_BLACK:
		return "BLACK";
	case LORA_VITAL_PRIORITY_RED:
		return "RED";
	case LORA_VITAL_PRIORITY_YELLOW:
		return "YELLOW";
	case LORA_VITAL_PRIORITY_GREEN:
		return "GREEN";
	default:
		return NULL;
	}
}

/** Station side: is this a packet we can parse at all? Length must cover the
 * fixed part and the tag it claims, so a truncated frame is rejected here
 * rather than parsed into a short identifier. */
static inline int lora_vital_valid(const lora_vital_t *v, uint8_t len)
{
	return (len >= LORA_VITAL_FIXED_LEN) && (v->version == LORA_VITAL_VERSION)
			&& (v->victim_rfid_len <= LORA_VITAL_RFID_MAX)
			&& (len >= (uint8_t) (LORA_VITAL_FIXED_LEN + v->victim_rfid_len));
}

#endif /* LORA_VITAL_H */
