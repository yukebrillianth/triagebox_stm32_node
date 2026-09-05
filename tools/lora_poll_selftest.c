/* Host test for ServiceLoRaPoll() in Core/Src/main.c -- the node's whole
 * poll-response path.
 *
 * IT TESTS THE REAL FUNCTION, NOT A COPY. run_selftests.sh cuts the config
 * defines and the function body straight out of Core/Src/main.c with sed and
 * drops them in svc_defs.inc / svc_body.inc, so editing main.c changes what is
 * tested here and there is no second copy to drift. That is worth the awkward
 * generated-include shape: the alternative is a transcription that passes
 * forever while the firmware changes underneath it.
 *
 * Everything below the includes is a fake radio scripted by hand. The fakes
 * reproduce the real library's behaviour where that behaviour is the point:
 * LoRa_receive() returns 0 unless RxDone is set and clears every flag when it
 * does read, and LoRa_write() only ever accepts RegIrqFlags/0xFF. What it
 * cannot reproduce is the trap that shaped the function -- that a mode change
 * aborts a reception in progress -- so the DIO0-low case asserts the radio is
 * not touched at all rather than asserting on a consequence.
 *
 * Reaches main.c's statics through the fakes' own file-scope variables of the
 * same names; that is why this file is compiled standalone and never linked
 * against main.o. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "lora_poll.h"
#include "lora_vital.h"

#include "svc_defs.inc" /* NODE_ID, LORA_* -- generated from main.c */

/* ---- the bits of the HAL / library the function touches ---- */
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define LORA_OK 200
#define RegIrqFlags 0x12
#define IRQ_RXDONE 0x40

typedef int GPIO_TypeDef;
typedef struct {
	GPIO_TypeDef *DIO0_port;
	uint16_t DIO0_pin;
} LoRa;

static LoRa hlora;
static uint16_t lora_status = LORA_OK;
static GPIO_TypeDef port_a;

/* counters, copied from main.c */
static volatile uint16_t mon_lora_frames;
static volatile uint32_t mon_lora_rx, mon_lora_polls, mon_lora_crc;
static volatile uint32_t mon_lora_stale, mon_lora_reply_ms;
static volatile int16_t mon_lora_rssi;
static lora_vital_t mon_lora_vital;
/* This node's address, which main.c resolves from the UID table below rather than
 * from a compile-time constant. 1 here so the scripted polls in main() are
 * addressed to this node; the no-address case sets it to 0 explicitly. */
static volatile uint8_t mon_node_id = 1U;

/* The table and the lookup, cut out of main.c by run_selftests.sh -- the same
 * arrangement as svc_body.inc, so main.c stays the only copy. Needs NODE_ID from
 * svc_defs.inc above, which is 0 here: no -DNODE_ID on this build line, so an
 * unknown UID has nowhere to fall back to, which is the case worth testing. */
#include "node_id.inc"

/* ---- scripted radio ---- */
static uint32_t fake_tick;
static int dio0;
static uint8_t irq_flags;
static uint8_t air[64];
static uint8_t air_len;
static unsigned tx_count, flags_cleared, reads;
static uint8_t tx_last[64];
static uint8_t tx_last_len;

static uint32_t HAL_GetTick(void) { return fake_tick; }

static int HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t pin)
{
	(void) p; (void) pin;
	return dio0 ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

static uint8_t LoRa_read(LoRa *l, uint8_t addr)
{
	(void) l;
	assert(addr == RegIrqFlags);
	++reads;
	return irq_flags;
}

static void LoRa_write(LoRa *l, uint8_t addr, uint8_t val)
{
	(void) l;
	assert(addr == RegIrqFlags);
	assert(val == 0xFFU);
	irq_flags = 0;
	dio0 = 0;
	++flags_cleared;
}

/* Mirrors the real library: standby-then-peek, returns 0 unless RxDone is set,
 * clears every flag when it does read, truncates to the caller's buffer. */
static uint8_t LoRa_receive(LoRa *l, uint8_t *buf, uint8_t len)
{
	uint8_t n;
	(void) l;
	if ((irq_flags & IRQ_RXDONE) == 0U) {
		return 0;
	}
	n = air_len < len ? air_len : len;
	memcpy(buf, air, n);
	irq_flags = 0;
	dio0 = 0;
	return n;
}

static uint16_t LoRa_transmit(LoRa *l, uint8_t *buf, uint8_t len, uint16_t tout)
{
	(void) l;
	assert(tout == LORA_TX_TIMEOUT);
	assert(len <= sizeof(tx_last));
	memcpy(tx_last, buf, len);
	tx_last_len = len;
	++tx_count;
	fake_tick += 82U; /* worst-case airtime */
	return 1;
}

static void PackTelemetry(void)
{
	memset(&mon_lora_vital, 0, sizeof(mon_lora_vital));
	mon_lora_vital.node_id = 0x01U;
	mon_lora_vital.version = LORA_VITAL_VERSION;
	mon_lora_vital.packet_counter = 7U;
}

/*
 * The real LoRa_getRSSI() is `-164 + RegPktRssiValue`, so it returns whatever
 * the byte gives it -- including values int8_t cannot hold. Scriptable here
 * precisely so the clamp in ServiceLoRaPoll() can be tested with those: the
 * failure it exists to stop is -164 narrowing to +92, which passes every
 * plausibility test and displays as an unusually strong signal.
 */
static int fake_rssi = -97;
static unsigned rssi_reads;

static int LoRa_getRSSI(LoRa *l)
{
	(void) l;
	++rssi_reads;
	return fake_rssi;
}

/* What ServiceLoRaPoll() handed to the I2C link, and whether it handed anything
 * at all -- the ordering matters as much as the value (see the test). */
static int8_t published_rssi;
static unsigned rssi_publishes;

static void tb_slave_set_rssi(int8_t dbm)
{
	published_rssi = dbm;
	++rssi_publishes;
	/* The reply has not gone out yet when this is called. Asserted here rather
	 * than in the test body because the ordering is the whole reason the latch
	 * sits where it does: LoRa_transmit() returns the radio to RX continuous,
	 * where the next packet heard overwrites RegPktRssiValue -- very likely
	 * another node answering its own poll on this shared channel. */
	assert(tx_count == 0U);
}

#include "svc_body.inc" /* the function under test, generated from main.c */

/* ---- driving it ---- */
static void arm_poll(uint8_t node, uint8_t cmd)
{
	lora_poll_t p;
	memset(&p, 0, sizeof(p));
	p.magic = LORA_POLL_MAGIC;
	p.version = LORA_POLL_VERSION;
	p.station_id = 1U;
	p.node_id = node;
	p.command = cmd;
	memcpy(air, &p, sizeof(p));
	air_len = LORA_POLL_LEN;
	irq_flags = IRQ_RXDONE;
	dio0 = 1;
}

static void reset_counters(void)
{
	mon_lora_rx = mon_lora_polls = mon_lora_crc = 0;
	mon_lora_stale = mon_lora_reply_ms = 0;
	mon_lora_frames = 0;
	mon_lora_rssi = 0;
	tx_count = flags_cleared = reads = 0;
	tx_last_len = 0;
	rssi_reads = rssi_publishes = 0;
	published_rssi = 0;
}

/* One pass of the superloop: dt ms since the previous pass. */
static void pass(uint32_t dt)
{
	fake_tick += dt;
	ServiceLoRaPoll();
}

/*
 * NodeIdFromUid(): which node this board thinks it is.
 *
 * The one value in this firmware whose failure is invisible AND destructive. Two
 * boards on one address is a capture-effect lottery -- the stronger reply passes
 * CRC carrying the node_id the station polled for, and two patients' vitals
 * alternate under one MQTT identity with nothing anywhere reporting it. So the
 * miss case is asserted as hard as the hit: an unknown UID must resolve to 0 and
 * stay silent, never fall back to 1.
 *
 * The test supplies its OWN table because the shipped one cannot contain a real
 * board's UID -- inventing one would hand a real board somebody else's identity --
 * so a hit can only be exercised with a UID the test owns.
 */
static void test_node_id_from_uid(void)
{
	static const struct node_uid tab[] = {
		{ { 0x33, 0x00, 0x35, 0x00, 0x0F, 0x51, 0x37, 0x38, 0x33, 0x31, 0x33,
				0x39 }, 1U },
		{ { 0x41, 0x00, 0x2A, 0x00, 0x11, 0x51, 0x37, 0x38, 0x33, 0x31, 0x33,
				0x39 }, 7U },
	};
	const size_t n = sizeof(tab) / sizeof(tab[0]);
	uint8_t uid[TB_REG_UID_LEN];
	size_t i;

	/* A hit returns that row's id, from either position in the table. */
	assert(NodeIdFromUid(tab[0].uid, tab, n) == 1U);
	assert(NodeIdFromUid(tab[1].uid, tab, n) == 7U);

	/*
	 * A MISS MUST BE 0, NOT 1, and that is this function's whole reason to
	 * exist. NODE_ID is 0 on this build line, exactly as it is in the Makefile,
	 * so an unprovisioned board has nowhere to fall back to -- and
	 * lora_poll_for_me() refusing a 0 id is what turns that into silence.
	 */
	memcpy(uid, tab[0].uid, sizeof(uid));
	uid[TB_REG_UID_LEN - 1U] ^= 0x01U; /* one bit off a known board */
	assert(NodeIdFromUid(uid, tab, n) == 0U);
	assert(!lora_poll_for_me((const lora_poll_t *) tab[0].uid, LORA_POLL_LEN,
			NodeIdFromUid(uid, tab, n)));

	/* Every byte is compared, so a UID matching on a PREFIX is still a miss.
	 * That matters more than it looks: STM32 UIDs from one wafer share whole
	 * words, so a prefix match would collide across a production batch. */
	for (i = 0U; i < TB_REG_UID_LEN; ++i) {
		memcpy(uid, tab[1].uid, sizeof(uid));
		uid[i] ^= 0x80U;
		assert(NodeIdFromUid(uid, tab, n) == 0U);
	}

	/* All zeros -- a failed read, or a struct nobody filled -- must miss as well.
	 * It is also the shipped table's placeholder row, which carries id 0, so both
	 * paths agree on "no address" rather than disagreeing quietly. */
	memset(uid, 0, sizeof(uid));
	assert(NodeIdFromUid(uid, tab, n) == 0U);

	/* A row that is itself out of range fails the same way an unknown UID does
	 * rather than being passed through: 0 would answer every node's poll and an
	 * id above LORA_POLL_NODE_MAX is never polled at all. */
	{
		static const struct node_uid bad[] = {
			{ { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
					0x0C }, LORA_POLL_NODE_MAX + 1U },
			{ { 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B,
					0x1C }, 0U },
			{ { 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B,
					0x2C }, LORA_POLL_NODE_MAX },
		};

		assert(NodeIdFromUid(bad[0].uid, bad, 3U) == 0U);
		assert(NodeIdFromUid(bad[1].uid, bad, 3U) == 0U);
		/* ...and the largest legal id still passes, so the bound is a bound and
		 * not an off-by-one that quietly retires the last address. */
		assert(NodeIdFromUid(bad[2].uid, bad, 3U) == LORA_POLL_NODE_MAX);
	}

	/* The SHIPPED table, checked for the mistakes hand-editing invites: every
	 * row inside the legal range, and no id or UID appearing twice -- two boards
	 * on one address is the failure this table was added to end. */
	for (i = 0U; i < (sizeof(k_node_uids) / sizeof(k_node_uids[0])); ++i) {
		size_t j;

		assert(k_node_uids[i].id <= LORA_POLL_NODE_MAX);
		for (j = i + 1U; j < (sizeof(k_node_uids) / sizeof(k_node_uids[0])); ++j) {
			assert(k_node_uids[i].id == 0U || k_node_uids[i].id
					!= k_node_uids[j].id);
			assert(memcmp(k_node_uids[i].uid, k_node_uids[j].uid,
					TB_REG_UID_LEN) != 0);
		}
	}
}

int main(void)
{
	hlora.DIO0_port = &port_a;
	hlora.DIO0_pin = 2U;
	fake_tick = 1000U;

	/* The address lookup, before anything that uses the address: if this is
	 * wrong, every scripted poll below is testing the wrong node. */
	test_node_id_from_uid();

	/* First pass ever: a poll that latched during boot is answered, not
	 * counted stale -- the `primed` guard. */
	reset_counters();
	arm_poll(0x01U, LORA_POLL_CMD_REPORT);
	pass(5000U); /* a huge apparent gap; primed==0 must swallow it */
	assert(mon_lora_polls == 1U);
	assert(mon_lora_stale == 0U);
	assert(tx_count == 1U);
	assert(tx_last[0] == 0x01U);
	assert(tx_last_len == lora_vital_len(&mon_lora_vital));
	assert(mon_lora_frames == 1U);
	assert(dio0 == 0); /* receive cleared it */

	/* DIO0 low: the radio is never touched at all. This is the one that
	 * matters -- LoRa_receive() would abort a reception in progress. */
	reset_counters();
	dio0 = 0;
	pass(10U);
	pass(10U);
	assert(reads == 0U && flags_cleared == 0U && tx_count == 0U);
	assert(mon_lora_rx == 0U);

	/* CRC error: counted, flags cleared, nothing read, nothing sent. */
	reset_counters();
	arm_poll(0x01U, LORA_POLL_CMD_REPORT);
	irq_flags |= 0x20U;
	pass(10U);
	assert(mon_lora_crc == 1U);
	assert(flags_cleared == 1U);
	assert(mon_lora_rx == 0U && tx_count == 0U);
	assert(dio0 == 0);

	/* DIO0 high but RxDone clear: flags cleared anyway, or DIO0 stays high
	 * forever and every pass talks to the radio. */
	reset_counters();
	dio0 = 1;
	irq_flags = 0x08U; /* something that is not RxDone */
	air_len = 0U;
	pass(10U);
	assert(flags_cleared == 1U);
	assert(mon_lora_rx == 0U && tx_count == 0U);
	assert(dio0 == 0);

	/* A poll for somebody else: heard, not answered, not counted as a poll. */
	reset_counters();
	arm_poll(0x02U, LORA_POLL_CMD_REPORT);
	pass(10U);
	assert(mon_lora_rx == 1U);
	assert(mon_lora_polls == 0U && tx_count == 0U);

	/* Another node's reply on the shared channel: byte 0 is its node_id, so
	 * it can never look like a poll. */
	reset_counters();
	{
		lora_vital_t v;
		memset(&v, 0, sizeof(v));
		v.node_id = 0x03U;
		v.version = LORA_VITAL_VERSION;
		memcpy(air, &v, LORA_POLL_LEN);
		air_len = LORA_POLL_LEN;
		irq_flags = IRQ_RXDONE;
		dio0 = 1;
	}
	pass(10U);
	assert(mon_lora_rx == 1U && mon_lora_polls == 0U && tx_count == 0U);

	/* Unknown command: silent, and it is rejected before the poll counter. */
	reset_counters();
	arm_poll(0x01U, 0x7FU);
	pass(10U);
	assert(mon_lora_rx == 1U);
	assert(mon_lora_polls == 0U && tx_count == 0U);

	/* Stale: the previous pass was longer ago than the deadline. */
	reset_counters();
	arm_poll(0x01U, LORA_POLL_CMD_REPORT);
	pass(LORA_REPLY_DEADLINE_MS + 1U);
	assert(mon_lora_polls == 1U);
	assert(mon_lora_stale == 1U);
	assert(tx_count == 0U);

	/* Exactly at the deadline is still answered (the test is `>`). */
	reset_counters();
	arm_poll(0x01U, LORA_POLL_CMD_REPORT);
	pass(LORA_REPLY_DEADLINE_MS);
	assert(mon_lora_stale == 0U && tx_count == 1U);
	/* and the bound the counter reports stays inside the slot */
	assert(mon_lora_reply_ms < LORA_POLL_SLOT_MS);

	/* Radio absent: nothing is touched, ever. */
	reset_counters();
	lora_status = 0;
	arm_poll(0x01U, LORA_POLL_CMD_REPORT);
	pass(10U);
	assert(reads == 0U && tx_count == 0U && flags_cleared == 0U);
	lora_status = LORA_OK;

	/* ---- RSSI latch ---- */

	/* A normal poll: read once, published once, value passed through. */
	reset_counters();
	fake_rssi = -97;
	arm_poll(0x01U, LORA_POLL_CMD_REPORT);
	pass(10U);
	assert(rssi_reads == 1U && rssi_publishes == 1U);
	assert(published_rssi == -97);
	assert(mon_lora_rssi == -97);

	/*
	 * THE CLAMP. RegPktRssiValue == 0 makes LoRa_getRSSI() return -164, and
	 * -164 narrowed to int8_t is +92 -- a value the ESP32's tb_rssi_valid()
	 * would ACCEPT and display as an extremely strong signal, which is the
	 * worst possible direction for a number someone is using to judge range.
	 * Pinned to -128 so it lands outside the valid window instead.
	 */
	reset_counters();
	fake_rssi = -164;
	arm_poll(0x01U, LORA_POLL_CMD_REPORT);
	pass(10U);
	assert(published_rssi == -128);
	assert(mon_lora_rssi == -164); /* CubeMonitor sees the raw figure */

	/* The other end of the library's range, which is also not a real level. */
	reset_counters();
	fake_rssi = 91;
	arm_poll(0x01U, LORA_POLL_CMD_REPORT);
	pass(10U);
	assert(published_rssi == 91); /* fits int8_t; the ESP32 rejects it by range */

	/* A poll for another node must NOT move it: that packet's level describes
	 * the station's link to somebody else, and on a 20-node channel it is the
	 * common case, so adopting it would make this node's own reading wander. */
	reset_counters();
	fake_rssi = -60;
	arm_poll(0x02U, LORA_POLL_CMD_REPORT);
	pass(10U);
	assert(rssi_reads == 0U && rssi_publishes == 0U);

	/* Stale is rejected AFTER the latch: the poll was genuinely heard, so its
	 * level is real even though the reply was suppressed. Range-testing at the
	 * edge is exactly when replies start missing their slot, and that is the
	 * moment the number matters most. */
	reset_counters();
	fake_rssi = -118;
	arm_poll(0x01U, LORA_POLL_CMD_REPORT);
	pass(LORA_REPLY_DEADLINE_MS + 1U);
	assert(mon_lora_stale == 1U && tx_count == 0U);
	assert(rssi_publishes == 1U && published_rssi == -118);

	/* A node with NO ADDRESS answers nothing. This is the unprovisioned board:
	 * its UID is not in the table, so mon_node_id is 0, and lora_poll_for_me()
	 * refuses every poll on the channel -- including a poll addressed to 0, which
	 * a station has no reason to send but which would otherwise be answered by
	 * every unprovisioned board at once. It still HEARS the traffic, so
	 * mon_lora_rx climbing with polls stuck at 0 is the diagnosis. */
	reset_counters();
	mon_node_id = 0U;
	arm_poll(0x01U, LORA_POLL_CMD_REPORT);
	pass(10U);
	assert(mon_lora_rx == 1U);
	assert(mon_lora_polls == 0U && tx_count == 0U);
	reset_counters();
	arm_poll(0x00U, LORA_POLL_CMD_REPORT);
	pass(10U);
	assert(mon_lora_polls == 0U && tx_count == 0U);
	mon_node_id = 1U;

	printf("lora_poll_selftest: OK (ServiceLoRaPoll, NodeIdFromUid)\n");
	return 0;
}
