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
static lora_vital_t mon_lora_vital;

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
	tx_count = flags_cleared = reads = 0;
	tx_last_len = 0;
}

/* One pass of the superloop: dt ms since the previous pass. */
static void pass(uint32_t dt)
{
	fake_tick += dt;
	ServiceLoRaPoll();
}

int main(void)
{
	hlora.DIO0_port = &port_a;
	hlora.DIO0_pin = 2U;
	fake_tick = 1000U;

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

	printf("lora_poll_selftest: OK (ServiceLoRaPoll)\n");
	return 0;
}
