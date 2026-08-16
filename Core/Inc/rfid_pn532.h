#ifndef RFID_PN532_H
#define RFID_PN532_H

#include "main.h"
#include <stdint.h>

/**
 * Minimal PN532 driver over I2C for patient identification.
 *
 * Scope is deliberately one operation: read the UID of whatever ISO14443A tag
 * is presented. No Mifare authentication, no block reads, no card emulation,
 * no P2P. Triage needs an identifier to tie a telemetry record to a patient,
 * and the UID is transmitted in the clear anyway, so reading data off the card
 * would add attack surface and code for no benefit.
 *
 * Wiring: the module must be strapped for I2C. Cheap PN532 breakouts ship with
 * DIP switches or solder jumpers selecting HSU (UART), I2C or SPI, and the
 * factory default is usually HSU. In the wrong mode every call here fails with
 * PN532_ERR_ABSENT. I2C also needs pull-ups on SDA/SCL; most breakouts fit
 * them, but a bare PN532 does not.
 */

/** Longest UID ISO14443-3 allows. 4 bytes for Mifare Classic, 7 for
 *  NTAG/Ultralight, 10 is the theoretical maximum. */
#define PN532_UID_MAX 10u

/** Bytes of the last reply kept for inspection. 16 covers the longest handshake
 *  frame, which is all that matters for diagnosing a handshake. */
#define PN532_RAW_MAX 16u

/**
 * How far the last handshake attempt got. The status codes below say a
 * handshake failed; this says which of its steps failed, which is the
 * difference between a five-minute fix and an afternoon of guessing.
 *
 * Mirrored to mon_rfid_stage for CubeMonitor.
 */
typedef enum Pn532_Stage {
	PN532_STAGE_NONE = 0, /**< never attempted */
	PN532_STAGE_PROBE = 1, /**< address probe; failing here is hardware */
	PN532_STAGE_FW_SEND = 2, /**< sending GetFirmwareVersion */
	PN532_STAGE_FW_ACK = 3, /**< awaiting its ACK frame */
	PN532_STAGE_FW_READ = 4, /**< awaiting its reply */
	PN532_STAGE_FW_PARSE = 5, /**< reply arrived, validating it */
	PN532_STAGE_SAM_SEND = 6, /**< sending SAMConfiguration */
	PN532_STAGE_SAM_ACK = 7,
	PN532_STAGE_SAM_READ = 8,
	PN532_STAGE_SAM_PARSE = 9,
	PN532_STAGE_READY = 10, /**< handshake complete; scanning from here on */
	PN532_STAGE_SCAN_SEND = 11, /**< sending InListPassiveTarget */
	PN532_STAGE_SCAN_ACK = 12,
	PN532_STAGE_SCAN_READ = 13,
	PN532_STAGE_SCAN_PARSE = 14
} Pn532_Stage;

/** Result of the last scan. Mirrored to mon_rfid_status for CubeMonitor. */
typedef enum Pn532_Status {
	PN532_IDLE = 0, /**< no scan requested since boot */
	PN532_FOUND = 1, /**< a tag was read; uid/uid_hash are valid */
	PN532_NO_CARD = 2, /**< module answered, no tag in the field */
	PN532_ERR_I2C = 3, /**< module ACKed once but a transfer failed */
	PN532_ERR_ABSENT = 4, /**< address 0x24 never ACKed: bus mode, wiring, power */
	PN532_ERR_PROTO = 5 /**< the module ACKed but its reply was wrong or absent:
	                     *   right bus mode and wiring, wrong protocol state --
	                     *   or a supply that collapses once the RF field runs */
} Pn532_Status;

typedef struct Pn532_Tag {
	uint8_t uid[PN532_UID_MAX];
	uint8_t uid_len; /**< 0 when no tag was read */
	uint32_t uid_hash; /**< Pn532_HashUid(uid, uid_len), 0 when no tag */
	uint8_t status; /**< Pn532_Status */
} Pn532_Tag;

/**
 * Binds the driver to an I2C bus. Does not touch the bus: the module is probed
 * lazily on the first scan, so a missing or mis-strapped module costs nothing
 * while no one is scanning.
 */
void Pn532_Bind(I2C_HandleTypeDef *hi2c);

/** Marks a scan as wanted. Safe from an ISR or a command handler. */
void Pn532_RequestScan(void);

/**
 * Performs a requested scan, or returns PN532_IDLE immediately if none is
 * pending. Call from the main loop only.
 *
 * Blocks for up to ~120ms when a scan is pending, and not at all otherwise.
 * That is safe here because ADC and PPG samples are collected by interrupts,
 * so a stalled main loop delays processing without losing data; the margin is
 * the 4s ECG window, which is 30x longer than the worst case.
 *
 * @p out may be NULL. Returns the Pn532_Status for this call.
 */
uint8_t Pn532_Service(Pn532_Tag *out);

/**
 * FNV-1a over the UID, folding any length into 32 bits so the LoRa payload's
 * uint32_t rfid_uid field fits 7-byte and 10-byte UIDs as well as 4-byte ones.
 *
 * The length is hashed in as well, so a 4-byte UID cannot collide with a
 * 7-byte UID that happens to share its first four bytes.
 *
 * Consequence to be aware of at the receiver: this is not the number printed
 * on the card, so it cannot be cross-checked by hand. mon_rfid_uid[] holds the
 * raw bytes for that. Being 32 bits, two different tags collide with ~50%
 * probability only once you have ~77000 of them in one dataset; for a triage
 * incident that is far beyond any realistic patient count.
 *
 * Returns 0 for a NULL or zero-length UID, so 0 always means "no tag".
 */
uint32_t Pn532_HashUid(const uint8_t *uid, uint8_t len);

/** Pn532_Stage the last attempt reached. Diagnostics only. */
uint8_t Pn532_LastStage(void);

/**
 * Copies up to @p max bytes of the last reply read from the module, including
 * its leading I2C status byte, and returns how many were copied.
 *
 * Read this when a scan fails: a frame shifted by padding, a status byte of
 * 0x00, and all-0xFF from an open bus are three different faults that the
 * status code alone cannot tell apart.
 */
uint8_t Pn532_LastRaw(uint8_t *out, uint8_t max);

#endif /* RFID_PN532_H */
