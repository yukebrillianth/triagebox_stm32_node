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
#define PN532_CMD_IN_LIST_PASSIVE_TARGET 0x4Au

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

/* Longest frame this driver ever receives: status + preamble/start (3) + LEN +
 * LCS + TFI + 0x4B + NbTg + Tg + SENS_RES(2) + SEL_RES + UID len + 10 UID +
 * DCS + postamble. 32 is comfortable and keeps the buffer off the heap. */
#define PN532_RX_MAX 32u

static I2C_HandleTypeDef *pn_bus;
static volatile uint8_t pn_scan_requested;
/* 0 until the module has answered GetFirmwareVersion once. Latched so the
 * handshake is not repeated on every scan. */
static uint8_t pn_configured;

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

uint32_t Pn532_HashUid(const uint8_t *uid, uint8_t len)
{
	if (uid == NULL || len == 0U) {
		return 0U;
	}
	/* FNV-1a, 32-bit. Chosen over a CRC because it is four lines and has no
	 * table; UID collision resistance is all that is needed here. */
	uint32_t h = 2166136261u;
	for (uint8_t i = 0; i < len; ++i) {
		h ^= (uint32_t) uid[i];
		h *= 16777619u;
	}
	/* Fold the length in so a 4-byte UID cannot collide with a 7-byte one
	 * sharing its first four bytes. */
	h ^= (uint32_t) len;
	h *= 16777619u;
	/* 0 is reserved for "no tag", so never return it. */
	return (h == 0U) ? 1U : h;
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
 * Reads @p want bytes once the module reports ready, into @p buf.
 * @p want excludes the leading I2C status byte, which is consumed here.
 */
static uint8_t ReadFrame(uint8_t *buf, uint8_t want, uint32_t timeout_ms)
{
	uint8_t raw[PN532_RX_MAX];
	if ((uint32_t) want + 1U > sizeof(raw)) {
		return 0;
	}

	const uint32_t start = HAL_GetTick();
	do {
		if (HAL_I2C_Master_Receive(pn_bus, PN532_I2C_ADDR, raw,
				(uint16_t) (want + 1U), PN532_I2C_TIMEOUT) == HAL_OK) {
			/* Bit 0 of the status byte is the ready flag. Anything read before
			 * it is set is stale and must be discarded, not parsed. */
			if ((raw[0] & 0x01u) != 0U) {
				memcpy(buf, &raw[1], want);
				return 1;
			}
		}
		/* Do not hammer the bus while the module works on the RF exchange. */
		HAL_Delay(2);
	} while ((HAL_GetTick() - start) < timeout_ms);

	return 0;
}

/** Consumes the 6-byte ACK frame 00 00 FF 00 FF 00 the module sends first. */
static uint8_t ReadAck(void)
{
	static const uint8_t expect[6] = { 0x00u, 0x00u, 0xFFu, 0x00u, 0xFFu,
			0x00u };
	uint8_t ack[6];
	if (!ReadFrame(ack, sizeof(ack), PN532_READY_TIMEOUT)) {
		return 0;
	}
	return (memcmp(ack, expect, sizeof(expect)) == 0) ? 1U : 0U;
}

/**
 * Validates a response frame header and returns the payload length (the byte
 * count after TFI), or 0 if the frame is malformed.
 */
static uint8_t ParseHeader(const uint8_t *f, uint8_t f_len, uint8_t *payload_at)
{
	if (f_len < 7U) {
		return 0;
	}
	/* 00 00 FF LEN LCS TFI ... */
	if (f[0] != 0x00u || f[1] != 0x00u || f[2] != 0xFFu) {
		return 0;
	}
	const uint8_t len = f[3];
	if ((uint8_t) (len + f[4]) != 0x00u) {
		return 0; /* LEN + LCS must wrap to zero */
	}
	if (len < 1U || f[5] != PN532_PN532_TO_HOST) {
		return 0;
	}
	if ((uint32_t) 6U + len > f_len) {
		return 0; /* frame claims more data than was read */
	}
	*payload_at = 6U;
	return (uint8_t) (len - 1U); /* exclude TFI */
}

/** GetFirmwareVersion then SAMConfiguration: proves the module is there and
 *  puts it in normal reader mode with the watchdog off. */
static uint8_t Handshake(void)
{
	uint8_t cmd = PN532_CMD_GET_FIRMWARE_VERSION;
	if (!SendFrame(&cmd, 1U)) {
		return 0;
	}
	if (!ReadAck()) {
		return 0;
	}
	/* Reply: 00 00 FF 06 FA D5 03 IC Ver Rev Support DCS 00 */
	uint8_t rx[14];
	if (!ReadFrame(rx, sizeof(rx), PN532_READY_TIMEOUT)) {
		return 0;
	}
	uint8_t at = 0;
	if (ParseHeader(rx, sizeof(rx), &at) < 2U) {
		return 0;
	}
	if (rx[at + 1U] != (PN532_CMD_GET_FIRMWARE_VERSION + 1U)) {
		return 0;
	}

	/* SAMConfiguration: mode 0x01 = normal, timeout byte ignored in that mode,
	 * 0x01 = use IRQ pin. Sending it is required before InListPassiveTarget
	 * will work reliably after a cold start. */
	const uint8_t sam[4] = { PN532_CMD_SAM_CONFIGURATION, 0x01u, 0x14u, 0x01u };
	if (!SendFrame(sam, sizeof(sam))) {
		return 0;
	}
	if (!ReadAck()) {
		return 0;
	}
	uint8_t sam_rx[9]; /* 00 00 FF 02 FE D5 15 DCS 00 */
	if (!ReadFrame(sam_rx, sizeof(sam_rx), PN532_READY_TIMEOUT)) {
		return 0;
	}
	if (ParseHeader(sam_rx, sizeof(sam_rx), &at) < 1U) {
		return 0;
	}
	return (sam_rx[at] == (PN532_CMD_SAM_CONFIGURATION + 1U)) ? 1U : 0U;
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
		if (!Handshake()) {
			if (out != NULL) {
				out->status = PN532_ERR_ABSENT;
			}
			return PN532_ERR_ABSENT;
		}
		pn_configured = 1;
	}

	/* InListPassiveTarget, MaxTg = 1: stop at the first tag. Scanning for two
	 * doubles the worst-case blocking time for no benefit when one patient
	 * wears one band. */
	const uint8_t cmd[3] = { PN532_CMD_IN_LIST_PASSIVE_TARGET, 0x01u,
			PN532_BRTY_ISO14443A };
	if (!SendFrame(cmd, sizeof(cmd))) {
		if (out != NULL) {
			out->status = PN532_ERR_I2C;
		}
		return PN532_ERR_I2C;
	}
	if (!ReadAck()) {
		if (out != NULL) {
			out->status = PN532_ERR_I2C;
		}
		return PN532_ERR_I2C;
	}

	/* Reply with one 4-byte-UID target is 20 bytes; a 7-byte UID is 23. Always
	 * read the maximum: a short read would leave bytes in the module for the
	 * next scan to trip over. */
	uint8_t rx[PN532_RX_MAX];
	if (!ReadFrame(rx, (uint8_t) sizeof(rx), PN532_READY_TIMEOUT)) {
		/* No response at all within the window. The module ends its own RF
		 * poll, so this is the empty-field case, not a bus fault. */
		if (out != NULL) {
			out->status = PN532_NO_CARD;
		}
		return PN532_NO_CARD;
	}

	uint8_t at = 0;
	const uint8_t payload = ParseHeader(rx, (uint8_t) sizeof(rx), &at);
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

	/* Target record starts at at+2: Tg, SENS_RES hi/lo, SEL_RES, UIDLen, UID */
	const uint8_t rec = (uint8_t) (at + 2U);
	if ((uint32_t) rec + 5U > sizeof(rx)) {
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
			|| ((uint32_t) rec + 5U + uid_len) > sizeof(rx)) {
		if (out != NULL) {
			out->status = PN532_ERR_PROTO;
		}
		return PN532_ERR_PROTO;
	}

	if (out != NULL) {
		memcpy(out->uid, &rx[rec + 5U], uid_len);
		out->uid_len = uid_len;
		out->uid_hash = Pn532_HashUid(out->uid, uid_len);
		out->status = PN532_FOUND;
	}
	return PN532_FOUND;
}
