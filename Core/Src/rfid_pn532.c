/**
 ******************************************************************************
 * @file    rfid_pn532.c
 * @brief   Minimal PN532 UID reader over I2C.
 *
 * Frame format (PN532 User Manual UM0701-02, section 6.2.1.1):
 *
 *   00 00 FF LEN LCS TFI PD0..PDn DCS 00
 *
 * where LEN counts TFI plus the data bytes, LCS is the low byte of -(LEN),
 * TFI is 0xD4 host-to-PN532 or 0xD5 PN532-to-host, and DCS is the low byte of
 * -(TFI + all PDn). Both checksums are one's-complement style sums that must
 * make the running total end in 0x00.
 *
 * The I2C transport adds one wrinkle that is not in the SPI/UART flow: every
 * read from the module returns a leading status byte, and bit 0 of it is the
 * "ready" flag. Reading before it is set returns stale or zero data, so a
 * response has to be polled for rather than simply read.
 ******************************************************************************
 */

#include "rfid_pn532.h"
#include <string.h>

/* 7-bit address 0x24, so the HAL's 8-bit form is 0x48. Fixed in silicon. */
#define PN532_I2C_ADDR (0x24u << 1)

#define PN532_PREAMBLE 0x00u
#define PN532_START1 0x00u
#define PN532_START2 0xFFu
#define PN532_HOST_TO_PN532 0xD4u
#define PN532_PN532_TO_HOST 0xD5u

#define PN532_CMD_GET_FIRMWARE_VERSION 0x02u
#define PN532_CMD_SAM_CONFIGURATION 0x14u
#define PN532_CMD_RF_CONFIGURATION 0x32u
#define PN532_CMD_IN_LIST_PASSIVE_TARGET 0x4Au

/* RFConfiguration CfgItem 5 = MaxRetries {MxRtyATR, MxRtyPSL,
 * MxRtyPassiveActivation}. See Handshake() for why the last one is 1 and not
 * the module's default 0xFF. */
#define PN532_CFG_MAX_RETRIES 0x05u

/* 106 kbps type A, the mode every Mifare Classic and NTAG tag answers. */
#define PN532_BRTY_ISO14443A 0x00u

/* Per-transfer HAL timeout. Generous: a wrong-mode module does not answer at
 * all and fails on the address phase, so this only bounds a genuinely slow
 * reply, not the common failure. */
#define PN532_I2C_TIMEOUT 25u
/* How long to wait for the ready flag. InListPassiveTarget with a 1-target
 * limit answers in well under 50ms when a card is present; the module's own
 * timeout ends the poll when one is not. */
#define PN532_READY_TIMEOUT 100u

/* Address-phase probes before giving up. More than one because the PN532 NACKs
 * while it comes out of low-power state, so a single shot can fail on a module
 * that is perfectly healthy a millisecond later. */
#define PN532_PROBE_TRIES 3u

/* Longest frame this driver ever receives: status + preamble/start (3) + LEN +
 * LCS + TFI + 0x4B + NbTg + Tg + SENS_RES(2) + SEL_RES + UID len + 10 UID +
 * DCS + postamble. 32 is comfortable and keeps the buffer off the heap. */
#define PN532_RX_MAX 32u

/* What the scan actually asks ReadFrame for. One less than the buffer, because
 * ReadFrame reads want+1 bytes to consume the leading I2C status byte. */
#define PN532_SCAN_READ (PN532_RX_MAX - 1u)

static I2C_HandleTypeDef *pn_bus;
static volatile uint8_t pn_scan_requested;
/* 0 until the module has answered GetFirmwareVersion once. Latched so the
 * handshake is not repeated on every scan. */
static uint8_t pn_configured;

/* Where the last attempt stopped, and the last bytes the module actually sent.
 * Diagnostics only, but they are the difference between "the handshake failed"
 * and knowing which of eleven steps failed and what arrived instead. Cheap
 * enough to leave in permanently: one byte plus a 16-byte buffer. */
static uint8_t pn_stage;
static uint8_t pn_raw[PN532_RAW_MAX];
static uint8_t pn_raw_len;

void Pn532_Bind(I2C_HandleTypeDef *hi2c)
{
	pn_bus = hi2c;
	pn_scan_requested = 0;
	pn_configured = 0;
}

void Pn532_RequestScan(void)
{
	pn_scan_requested = 1;
}

uint8_t Pn532_LastStage(void)
{
	return pn_stage;
}

uint8_t Pn532_LastRaw(uint8_t *out, uint8_t max)
{
	if (out == NULL || max == 0U) {
		return 0;
	}
	const uint8_t n = (pn_raw_len < max) ? pn_raw_len : max;
	memcpy(out, pn_raw, n);
	return n;
}

uint32_t Pn532_PackUid(const uint8_t *uid, uint8_t len)
{
	if (uid == NULL || len == 0U) {
		return 0U;
	}
	/* Big-endian over up to 4 bytes: uid[0] lands in the most significant byte,
	 * so printing the result as hex gives the same digits in the same order as
	 * the ASCII the ESP32 displays. A shorter UID left-aligns rather than
	 * right-aligns for the same reason. */
	uint32_t v = 0U;
	const uint8_t n = (len < 4U) ? len : 4U;
	for (uint8_t i = 0; i < n; ++i) {
		v = (v << 8) | (uint32_t) uid[i];
	}
	return v;
}

/** Sends one command frame. @p body is TFI's payload: command byte then args. */
static uint8_t SendFrame(const uint8_t *body, uint8_t body_len)
{
	uint8_t buf[16];
	/* body_len + TFI must fit LEN, and buf must hold the wrapper. */
	if (body_len > (uint8_t) (sizeof(buf) - 7U)) {
		return 0;
	}

	const uint8_t len = (uint8_t) (body_len + 1U); /* +1 for TFI */
	uint8_t i = 0;
	buf[i++] = PN532_PREAMBLE;
	buf[i++] = PN532_START1;
	buf[i++] = PN532_START2;
	buf[i++] = len;
	buf[i++] = (uint8_t) (~len + 1U); /* LCS: LEN + LCS ends in 0x00 */
	buf[i++] = PN532_HOST_TO_PN532;

	uint8_t sum = PN532_HOST_TO_PN532;
	for (uint8_t k = 0; k < body_len; ++k) {
		buf[i++] = body[k];
		sum = (uint8_t) (sum + body[k]);
	}
	buf[i++] = (uint8_t) (~sum + 1U); /* DCS */
	buf[i++] = PN532_PREAMBLE; /* postamble */

	return (HAL_I2C_Master_Transmit(pn_bus, PN532_I2C_ADDR, buf, i,
			PN532_I2C_TIMEOUT) == HAL_OK) ? 1U : 0U;
}

/**
 * Waits for the module's ready flag, reading only the one status byte.
 *
 * The previous version polled by attempting the whole read and discarding it
 * when the flag was clear. That works on a compliant module but throws away a
 * byte of the output buffer on every discarded attempt, so a module that sets
 * ready mid-read can be left permanently one byte out of step. One byte in, one
 * decision out.
 */
static uint8_t WaitReady(uint32_t timeout_ms)
{
	const uint32_t start = HAL_GetTick();
	do {
		uint8_t st = 0;
		if (HAL_I2C_Master_Receive(pn_bus, PN532_I2C_ADDR, &st, 1U,
				PN532_I2C_TIMEOUT) == HAL_OK) {
			if ((st & 0x01u) != 0U) {
				return 1;
			}
		}
		/* Do not hammer the bus while the module works on the RF exchange. */
		HAL_Delay(2);
	} while ((HAL_GetTick() - start) < timeout_ms);

	return 0;
}

/**
 * Reads @p want bytes once the module reports ready, into @p buf.
 * @p want excludes the leading I2C status byte, which is consumed here.
 */
static uint8_t ReadFrame(uint8_t *buf, uint8_t want, uint32_t timeout_ms)
{
	uint8_t raw[PN532_RX_MAX];
	if ((uint32_t) want + 1U > sizeof(raw)) {
		return 0;
	}

	if (!WaitReady(timeout_ms)) {
		pn_raw_len = 0U; /* nothing arrived; do not leave the previous frame
		                  * sitting in the diagnostics looking current */
		return 0;
	}
	if (HAL_I2C_Master_Receive(pn_bus, PN532_I2C_ADDR, raw,
			(uint16_t) (want + 1U), PN532_I2C_TIMEOUT) != HAL_OK) {
		pn_raw_len = 0U;
		return 0;
	}

	/* Keep what arrived before validating it -- when the parse fails, these are
	 * the only bytes that say why. Status byte included, deliberately: a status
	 * of 0x01 with junk behind it is a different fault from a status of 0x00. */
	pn_raw_len = (want + 1U > PN532_RAW_MAX) ? PN532_RAW_MAX
			: (uint8_t) (want + 1U);
	memcpy(pn_raw, raw, pn_raw_len);

	memcpy(buf, &raw[1], want);
	return 1;
}

/**
 * Finds the 0x00 0xFF start-code pair and returns the index of the 0xFF, or
 * 0xFF if it is not there.
 *
 * The frame is documented as starting immediately, but the PN532 may emit
 * padding zeros ahead of the preamble and clone modules routinely do. Assuming
 * a fixed offset makes one stray leading byte look exactly like a dead module:
 * the start-code test fails and the whole handshake is reported absent. Search,
 * do not assume.
 */
static uint8_t FindStart(const uint8_t *f, uint8_t f_len)
{
	for (uint8_t i = 0; (uint8_t) (i + 1U) < f_len; ++i) {
		if (f[i] == 0x00u && f[i + 1U] == 0xFFu) {
			return (uint8_t) (i + 1U);
		}
	}
	return 0xFFu;
}

/** Consumes the ACK frame 00 00 FF 00 FF 00 the module sends first. Read two
 *  bytes longer than the frame so a padded ACK still contains all of it. */
static uint8_t ReadAck(void)
{
	uint8_t ack[8];
	if (!ReadFrame(ack, sizeof(ack), PN532_READY_TIMEOUT)) {
		return 0;
	}
	const uint8_t sc = FindStart(ack, sizeof(ack));
	if (sc == 0xFFu || (uint8_t) (sc + 2U) >= sizeof(ack)) {
		return 0;
	}
	/* After the start code an ACK is 00 FF, a NACK is FF 00. Only the former
	 * means "command accepted". */
	return (ack[sc + 1U] == 0x00u && ack[sc + 2U] == 0xFFu) ? 1U : 0U;
}

/**
 * Validates a response frame header and returns the payload length (the byte
 * count after TFI), or 0 if the frame is malformed. Writes the payload offset
 * to @p payload_at.
 */
static uint8_t ParseHeader(const uint8_t *f, uint8_t f_len, uint8_t *payload_at)
{
	const uint8_t sc = FindStart(f, f_len);
	if (sc == 0xFFu) {
		return 0;
	}
	/* LEN, LCS and TFI follow the 0xFF, and at least one payload byte after. */
	const uint8_t len_at = (uint8_t) (sc + 1U);
	if ((uint32_t) len_at + 3U > f_len) {
		return 0;
	}

	const uint8_t len = f[len_at];
	if ((uint8_t) (len + f[len_at + 1U]) != 0x00u) {
		return 0; /* LEN + LCS must wrap to zero */
	}
	if (len < 1U || f[len_at + 2U] != PN532_PN532_TO_HOST) {
		return 0;
	}
	/* LEN counts TFI plus the payload, so the payload ends len-1 bytes after
	 * TFI; all of it must be inside what was actually read. */
	if ((uint32_t) len_at + 2U + len > f_len) {
		return 0; /* frame claims more data than was read */
	}

	*payload_at = (uint8_t) (len_at + 3U);
	return (uint8_t) (len - 1U); /* exclude TFI */
}

/**
 * Discards one frame the module is already holding, if any.
 *
 * Costs one status-byte read when nothing is pending, which is the normal case.
 * It exists because a desync is SILENT AND STICKY: if a response is ever left
 * unread, the next scan's ReadAck() consumes it instead of the ACK and every
 * read after that is one frame behind for the rest of the 30 s scan window --
 * a card that reads instantly one moment and not at all the next, with no
 * wiring difference. The MaxRetries setting in Handshake() removes the known
 * cause; this makes any remaining cause self-healing rather than permanent.
 */
static void DrainPending(void)
{
	uint8_t st = 0;

	if (HAL_I2C_Master_Receive(pn_bus, PN532_I2C_ADDR, &st, 1U,
			PN532_I2C_TIMEOUT) != HAL_OK) {
		return;
	}
	if ((st & 0x01u) == 0U) {
		return; /* nothing waiting, which is what a healthy scan looks like */
	}
	uint8_t junk[PN532_RX_MAX];
	(void) HAL_I2C_Master_Receive(pn_bus, PN532_I2C_ADDR, junk, sizeof(junk),
			PN532_I2C_TIMEOUT);
}

/**
 * Address-phase-only check: does anything at 0x24 ACK?
 *
 * Worth separating from the handshake because the two failures look identical
 * from the outside (both give PN532_ERR_ABSENT) but mean opposite things. A
 * failing probe is hardware -- bus mode strap, wiring, supply. A passing probe
 * with a failing handshake means the chip is on the bus and talking, and the
 * problem is the frame protocol or a module that answers addressing while
 * browning out under RF load. Guessing between those two costs hours.
 */
static uint8_t Probe(void)
{
	return (HAL_I2C_IsDeviceReady(pn_bus, PN532_I2C_ADDR, PN532_PROBE_TRIES,
			PN532_I2C_TIMEOUT) == HAL_OK) ? 1U : 0U;
}

/** GetFirmwareVersion then SAMConfiguration: proves the module is there and
 *  puts it in normal reader mode with the watchdog off. */
static uint8_t Handshake(void)
{
	uint8_t cmd = PN532_CMD_GET_FIRMWARE_VERSION;
	pn_stage = PN532_STAGE_FW_SEND;
	if (!SendFrame(&cmd, 1U)) {
		return 0;
	}
	pn_stage = PN532_STAGE_FW_ACK;
	if (!ReadAck()) {
		return 0;
	}
	/* Reply: 00 00 FF 06 FA D5 03 IC Ver Rev Support DCS 00, plus slack for any
	 * leading padding the module inserts. */
	uint8_t rx[16];
	pn_stage = PN532_STAGE_FW_READ;
	if (!ReadFrame(rx, sizeof(rx), PN532_READY_TIMEOUT)) {
		return 0;
	}
	uint8_t at = 0;
	pn_stage = PN532_STAGE_FW_PARSE;
	if (ParseHeader(rx, sizeof(rx), &at) < 2U) {
		return 0;
	}
	if (rx[at] != (PN532_CMD_GET_FIRMWARE_VERSION + 1U)) {
		return 0;
	}

	/* SAMConfiguration: mode 0x01 = normal, timeout byte ignored in that mode,
	 * 0x01 = use IRQ pin. Sending it is required before InListPassiveTarget
	 * will work reliably after a cold start. */
	const uint8_t sam[4] = { PN532_CMD_SAM_CONFIGURATION, 0x01u, 0x14u, 0x01u };
	pn_stage = PN532_STAGE_SAM_SEND;
	if (!SendFrame(sam, sizeof(sam))) {
		return 0;
	}
	pn_stage = PN532_STAGE_SAM_ACK;
	if (!ReadAck()) {
		return 0;
	}
	uint8_t sam_rx[12]; /* 00 00 FF 02 FE D5 15 DCS 00 + padding slack */
	pn_stage = PN532_STAGE_SAM_READ;
	if (!ReadFrame(sam_rx, sizeof(sam_rx), PN532_READY_TIMEOUT)) {
		return 0;
	}
	pn_stage = PN532_STAGE_SAM_PARSE;
	if (ParseHeader(sam_rx, sizeof(sam_rx), &at) < 1U) {
		return 0;
	}
	if (sam_rx[at] != (PN532_CMD_SAM_CONFIGURATION + 1U)) {
		return 0;
	}

	/*
	 * MxRtyPassiveActivation = 1, and this is the fix for "sometimes the card is
	 * not detected".
	 *
	 * The module's default is 0xFF, which the user manual defines as RETRY
	 * FOREVER: InListPassiveTarget with no card in the field never answers at
	 * all. ReadFrame then gives up after PN532_READY_TIMEOUT and this driver
	 * reports NO_CARD -- correct as an answer, but THE MODULE IS STILL RUNNING
	 * THAT COMMAND. ServiceRfid() calls back one superloop pass later, sends a
	 * fresh InListPassiveTarget, and the module's eventual reply to the FIRST one
	 * arrives where the second one's ACK was expected. From there every read is
	 * one frame behind: ReadAck() sees a response instead of an ACK, the scan
	 * fails, and the desync persists for the rest of the 30 s window. That is
	 * exactly the reported symptom -- a card that reads instantly sometimes and
	 * not at all other times, with no wiring difference.
	 *
	 * With one retry the module always answers within ~50 ms (NbTg = 0 when the
	 * field is empty), so every command gets exactly one response and the frame
	 * stream cannot drift. The retry loop that matters is ServiceRfid()'s own
	 * 30 s window, which re-sends the command each pass anyway -- so nothing is
	 * lost; the retrying just moves to where it can be bounded and observed.
	 *
	 * ATR and PSL retries are left at their defaults (0xFF): they apply to
	 * activation steps this driver never reaches, and changing what is not
	 * understood is how a working handshake breaks.
	 *
	 * Not fatal if it fails. A module that answers GetFirmwareVersion and
	 * SAMConfiguration but rejects this is still usable at the old behaviour, and
	 * refusing to scan at all would be strictly worse than scanning imperfectly.
	 */
	{
		const uint8_t cfg[5] = { PN532_CMD_RF_CONFIGURATION,
				PN532_CFG_MAX_RETRIES, 0xFFu, 0xFFu, 0x01u };
		uint8_t cfg_rx[12];

		if (SendFrame(cfg, sizeof(cfg)) && ReadAck()) {
			(void) ReadFrame(cfg_rx, sizeof(cfg_rx), PN532_READY_TIMEOUT);
		}
	}

	pn_stage = PN532_STAGE_READY;
	return 1;
}

uint8_t Pn532_Service(Pn532_Tag *out)
{
	if (out != NULL) {
		memset(out, 0, sizeof(*out));
	}
	if (!pn_scan_requested) {
		return PN532_IDLE;
	}
	pn_scan_requested = 0;

	if (pn_bus == NULL) {
		if (out != NULL) {
			out->status = PN532_ERR_ABSENT;
		}
		return PN532_ERR_ABSENT;
	}

	if (!pn_configured) {
		/* Probe first so the two causes are distinguishable: no ACK at all is
		 * ABSENT (hardware), an ACK followed by a failed handshake is PROTO. */
		pn_stage = PN532_STAGE_PROBE;
		if (!Probe()) {
			if (out != NULL) {
				out->status = PN532_ERR_ABSENT;
			}
			return PN532_ERR_ABSENT;
		}
		if (!Handshake()) {
			if (out != NULL) {
				out->status = PN532_ERR_PROTO;
			}
			return PN532_ERR_PROTO;
		}
		pn_configured = 1;
	}

	/*
	 * Anything the module is still holding is dropped before asking again --
	 * see DrainPending(). One status read in the common case.
	 */
	DrainPending();

	/* InListPassiveTarget, MaxTg = 1: stop at the first tag. Scanning for two
	 * doubles the worst-case blocking time for no benefit when one patient
	 * wears one band. */
	const uint8_t cmd[3] = { PN532_CMD_IN_LIST_PASSIVE_TARGET, 0x01u,
			PN532_BRTY_ISO14443A };
	pn_stage = PN532_STAGE_SCAN_SEND;
	if (!SendFrame(cmd, sizeof(cmd))) {
		if (out != NULL) {
			out->status = PN532_ERR_I2C;
		}
		return PN532_ERR_I2C;
	}
	pn_stage = PN532_STAGE_SCAN_ACK;
	if (!ReadAck()) {
		if (out != NULL) {
			out->status = PN532_ERR_I2C;
		}
		return PN532_ERR_I2C;
	}

	/*
	 * Reply with one 4-byte-UID target is 20 bytes; a 7-byte UID is 23. Read the
	 * largest frame that fits, so a short read cannot leave bytes in the module
	 * for the next scan to trip over.
	 *
	 * PN532_SCAN_READ is one less than the buffer because ReadFrame prepends the
	 * I2C status byte. Asking for the full buffer made ReadFrame's own bounds
	 * check fail, so the read never happened and every scan returned NO_CARD --
	 * indistinguishable from an empty field, which is why it went unnoticed.
	 */
	uint8_t rx[PN532_RX_MAX];
	pn_stage = PN532_STAGE_SCAN_READ;
	if (!ReadFrame(rx, PN532_SCAN_READ, PN532_READY_TIMEOUT)) {
		/* No response at all within the window. The module ends its own RF
		 * poll, so this is the empty-field case, not a bus fault. */
		if (out != NULL) {
			out->status = PN532_NO_CARD;
		}
		return PN532_NO_CARD;
	}

	uint8_t at = 0;
	pn_stage = PN532_STAGE_SCAN_PARSE;
	const uint8_t payload = ParseHeader(rx, PN532_SCAN_READ, &at);
	/* Payload: 4B NbTg [Tg SENS_RES(2) SEL_RES UIDLen UID...] = 2 minimum. */
	if (payload < 2U || rx[at] != (PN532_CMD_IN_LIST_PASSIVE_TARGET + 1U)) {
		if (out != NULL) {
			out->status = PN532_ERR_PROTO;
		}
		return PN532_ERR_PROTO;
	}
	if (rx[at + 1U] == 0U) {
		if (out != NULL) {
			out->status = PN532_NO_CARD;
		}
		return PN532_NO_CARD;
	}

	/* Target record starts at at+2: Tg, SENS_RES hi/lo, SEL_RES, UIDLen, UID.
	 * Bounds are PN532_SCAN_READ, not sizeof(rx): only that many bytes were
	 * filled, so the last byte of the buffer is uninitialised. */
	const uint8_t rec = (uint8_t) (at + 2U);
	if ((uint32_t) rec + 5U > PN532_SCAN_READ) {
		if (out != NULL) {
			out->status = PN532_ERR_PROTO;
		}
		return PN532_ERR_PROTO;
	}
	const uint8_t uid_len = rx[rec + 4U];
	/* Bound the length from the card before using it as a copy size: it is
	 * attacker-controlled in the sense that any tag can claim any value, and a
	 * bogus 0xFF would otherwise read past the buffer. */
	if (uid_len == 0U || uid_len > PN532_UID_MAX
			|| ((uint32_t) rec + 5U + uid_len) > PN532_SCAN_READ) {
		if (out != NULL) {
			out->status = PN532_ERR_PROTO;
		}
		return PN532_ERR_PROTO;
	}

	if (out != NULL) {
		memcpy(out->uid, &rx[rec + 5U], uid_len);
		out->uid_len = uid_len;
		out->status = PN532_FOUND;
	}
	return PN532_FOUND;
}
