#ifndef GYROSENSOR_DATA_BOX_H
#define GYROSENSOR_DATA_BOX_H

/* ============================================================================
 * G-sensor / IMU metadata track (mett trak) box builders.
 *
 * These build all the ISO-BMFF boxes for the metadata trak that carries the
 * G-sensor / EIS samples:
 *
 *   trak -> tkhd
 *        -> mdia -> mdhd
 *                -> hdlr (handler_type = 'meta', name = "SensorData")
 *                -> minf -> nmhd
 *                        -> dinf
 *                        -> stbl -> stsd -> mett (sample entry)
 *                                -> stts
 *                                -> stsc
 *                                -> stsz
 *                                -> stco
 *
 * Moved out of mp4_muxer.c into this file (gyrosensor folder).
 * ========================================================================== */

#include "mp4_muxer.h"

#define METADATA_TRACK_ID 3

/* Shared box helpers (defined in mp4_muxer.c / mp4_muxer.h) */
int create_box(unsigned char *buf, char *str, int size);
int create_box_full(unsigned char *buf, char *str, int size, unsigned int version, unsigned int flag);
void Save32BigEndian(unsigned int src, void *DES);
void update_box_size(pmp4_context mp4_ctx, unsigned char *buf, int size);
void update_udtabox_size(pmp4_context mp4_ctx, unsigned char *buf, int size);
int dinf_box(unsigned char *moov_buf, pmp4_context mp4_ctx);

/* ---- Camera calibration udta (udta -> meta -> hdlr/keys/ilst) ---- */
/* Buffer that must be large enough for the built udta box (9 keys). */
#define CAMERA_CALIB_UDTA_SIZE 1024

/* Buffer that holds the built metadata (mett) trak box (like CAMERA_CALIB_UDTA_SIZE).
   The moov box has generous headroom, so a few KB here is safe. */
#define META_TRAK_BOX_SIZE   32768

/* Write a 32-bit big-endian value into buf (ISO BMFF). */
void calib_put32_be(unsigned char *buf, uint32_t v);

/* Build meta -> hdlr('mdta') / keys / ilst. Called by camera_calib_set_udta()
   in lifetime_recording_initialize.c. Returns size. */
int camera_calib_meta_box(unsigned char *buf);

/* ---- Metadata-track box builders (each returns new moov_position) ---- */
int tkhd_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx);
int hdlr_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx);
int nmhd_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx);
int mett_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx);
int stsd_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx);
int stts_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx);
int stsc_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx);
int stsz_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx);
int stco_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx);
int stbl_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx);
int minf_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx);
int mdhd_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx);
int mdia_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx);
int trak_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx);

#endif /* GYROSENSOR_DATA_BOX_H */
