/* ============================================================================
 * G-sensor / IMU metadata track (mett trak) box builders.
 *
 * Extracted from mp4_muxer.c. Builds all the boxes for the metadata trak:
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
 * ========================================================================== */

#include "gyrosensor_data_box.h"

#include <string.h>

#include "../stabilize/stabilize_config.h"

#define MEDIA_TIME_SCALE 90000

int tkhd_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
    int size = create_box_full(moov_buf, "tkhd", 92, 0, 1);
    Save32BigEndian(1, moov_buf + 8); // Version(1Byte)+Flags(3Bytes)

    // Track ID for metadata
    Save32BigEndian(METADATA_TRACK_ID, moov_buf + 20);

    // orignal // Duration based on first/last metadata sample timestamps
    // if (mp4_ctx->metadata_len > 0) {
    //     Save32BigEndian(
    //         mp4_ctx->metadata_timestamp_buffer[mp4_ctx->metadata_len - 1] -
    //         mp4_ctx->metadata_timestamp_buffer[0],
    //         moov_buf + 28
    //     );
    // } else {
    //     Save32BigEndian(0, moov_buf + 28);
    // }

    // Duration based on first/last metadata sample timestamps (in 90kHz ticks)
    if (mp4_ctx->metadata_len > 0) {
        unsigned int duration = mp4_ctx->video_timestamp_buffer[mp4_ctx->root.video_len - 1] - mp4_ctx->metadata_timestamp_buffer[0];
        Save32BigEndian(duration, moov_buf + 28);
    } else {
        Save32BigEndian(0, moov_buf + 28);
    }

    // Matrix defaults
    Save32BigEndian(0x00010000, moov_buf + 48);
    Save32BigEndian(0x00010000, moov_buf + 64);
    Save32BigEndian(0x40000000, moov_buf + 80);

    // Track width/height = 0 for metadata
    Save32BigEndian(0, moov_buf + 84);
    Save32BigEndian(0, moov_buf + 88);

    return size;
}

int hdlr_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
    int size = 0;
    int moov_position = 0;
    const char *hdlname = "SensorData";

    size = create_box_full(moov_buf + moov_position, "hdlr", 32 + strlen(hdlname) + 1, 0, 0);
    strncpy((char *)(moov_buf + moov_position + 16), "meta", 4); // handler type
    strcpy((char *)(moov_buf + moov_position + 32), hdlname); // handler name
    moov_position += size;
    return moov_position;
}

int nmhd_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
    int size = 0;
    int moov_position = 0;
    size = create_box_full(moov_buf + moov_position, "nmhd", 16, 0, 0);
    // 4 bytes reserved (set to 0)
    Save32BigEndian(0, moov_buf + moov_position + 12);
    moov_position += size;
    return moov_position;
}

int mett_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
    int size = 0;
    int moov_position = 0;
    /* mett sample entry (ISO 14496-12 MetaDataSampleEntry):
       8 (box header: size + 'mett')
       + 8 (SampleEntry base: 6 reserved + 2 data_reference_index)
       + content_encoding (empty, null-terminated) = 1
       + mime_format ("text", null-terminated)     = 5
       + meta_format ("xml", null-terminated)      = 4
       total = 26 */
    size = 8 + 8 + 1 + 5 + 4;
    size = create_box(moov_buf + moov_position, "mett", size);
    moov_position += 8; // box header

    /* SampleEntry base: 6 reserved (0) + 2 data_reference_index = 1.
       Writing the whole 32-bit at +6 keeps reserved bytes zeroed and
       places data_reference_index in bytes 6-7. */
    Save32BigEndian(1, moov_buf + moov_position + 6);
    moov_position += 8;

    /* content_encoding = empty string (single null terminator) */
    moov_buf[moov_position++] = 0;

    /* mime_format = "text" + null */
    memcpy(moov_buf + moov_position, "text", 4);
    moov_position += 4;
    moov_buf[moov_position++] = 0;

    /* meta_format = "xml" + null */
    memcpy(moov_buf + moov_position, "xml", 3);
    moov_position += 3;
    moov_buf[moov_position++] = 0;

    return moov_position;
}

int stsd_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
    int size = 0;
    int moov_position = 0;
    size = create_box_full(moov_buf + moov_position, "stsd", 16, 0, 0);
    Save32BigEndian(1, moov_buf + moov_position + 12); // Entry count = 1
    moov_position += size;
    size = mett_box_metadata(moov_buf + moov_position, mp4_ctx);
    moov_position += size;

    update_box_size(mp4_ctx, moov_buf, moov_position);
    return moov_position;
}

int stts_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
    int size = 0;
    int moov_position = 0;
    size = create_box_full(moov_buf + moov_position, "stts", 16 + 8 * mp4_ctx->metadata_len, 0, 0);
    Save32BigEndian(mp4_ctx->metadata_len, moov_buf + moov_position + 12);
    for (unsigned int i = 0; i < mp4_ctx->metadata_len; i++) {
        Save32BigEndian(1, moov_buf + moov_position + 16 + 8 * i); // Sample count
        if (i == 0) {
            Save32BigEndian(1, moov_buf + moov_position + 20 + 8 * i); // Sample delta
        } else {
            Save32BigEndian(mp4_ctx->metadata_timestamp_buffer[i] - mp4_ctx->metadata_timestamp_buffer[i - 1],
                            moov_buf + moov_position + 20 + 8 * i); // Sample delta
        }
    }
    moov_position += size;
    return moov_position;
}

int stsc_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
    int size = 0;
    int moov_position = 0;
    size = create_box_full(moov_buf + moov_position, "stsc", 16 + 12 * mp4_ctx->metadata_len, 0, 1);
    Save32BigEndian(mp4_ctx->metadata_len, moov_buf + moov_position + 12);
    for (unsigned int i = 0; i < mp4_ctx->metadata_len; i++) {
        Save32BigEndian(i + 1, moov_buf + moov_position + 16 + 12 * i); // first_chunk
        Save32BigEndian(1, moov_buf + moov_position + 20 + 12 * i); // samples_per_chunk
        Save32BigEndian(1, moov_buf + moov_position + 24 + 12 * i); // sample_description_index
    }
    moov_position += size;
    return moov_position;
}

int stsz_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
    if (!mp4_ctx || mp4_ctx->metadata_len <= 0 ||
        !mp4_ctx->metadata_buffer_size) {
        // Nothing to write, return 0
        return 0;
    }
    int size = 0;
    int moov_position = 0;
    size = create_box_full(moov_buf + moov_position, "stsz", 20 + 4 * mp4_ctx->metadata_len, 0, 0);
    Save32BigEndian(mp4_ctx->metadata_len, moov_buf + moov_position + 16); // sample_count
    for (unsigned int i = 0; i < mp4_ctx->metadata_len; i++) {
        Save32BigEndian(mp4_ctx->metadata_buffer_size[i], moov_buf + moov_position + 20 + 4 * i); // entry_size
    }
    moov_position += size;
    return moov_position;
}

int stco_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
    if (!mp4_ctx || mp4_ctx->metadata_len <= 0 ||
        !mp4_ctx->metadata_buffer_index) {
        // Nothing to write, return 0
        return 0;
    }
    int size = 0;
    int moov_position = 0;
    size = create_box_full(moov_buf + moov_position, "stco", 16 + 4 * mp4_ctx->metadata_len, 0, 0);
    Save32BigEndian(mp4_ctx->metadata_len, moov_buf + moov_position + 12);
    for (unsigned int i = 0; i < mp4_ctx->metadata_len; i++) {
        Save32BigEndian(mp4_ctx->metadata_buffer_index[i], moov_buf + moov_position + 16 + 4 * i); // chunk offset
    }
    moov_position += size;
    return moov_position;
}

int stbl_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
    int size = 0;
    int moov_position = 0;
    size = create_box(moov_buf + moov_position, "stbl", 8);
    moov_position += size;
    size = stsd_box_metadata(moov_buf + moov_position, mp4_ctx);
    moov_position += size;
    size = stts_box_metadata(moov_buf + moov_position, mp4_ctx);
    moov_position += size;
    size = stsc_box_metadata(moov_buf + moov_position, mp4_ctx);
    moov_position += size;
    size = stsz_box_metadata(moov_buf + moov_position, mp4_ctx);
    moov_position += size;
    size = stco_box_metadata(moov_buf + moov_position, mp4_ctx);
    moov_position += size;

    update_box_size(mp4_ctx, moov_buf, moov_position);
    return moov_position;
}

int minf_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
    int size = 0;
    int moov_position = 0;
    size = create_box(moov_buf + moov_position, "minf", 8);
    moov_position += size;
    size = nmhd_box_metadata(moov_buf + moov_position, mp4_ctx);
    moov_position += size;
    size = dinf_box(moov_buf + moov_position, mp4_ctx);
    moov_position += size;
    size = stbl_box_metadata(moov_buf + moov_position, mp4_ctx);
    moov_position += size;

    update_box_size(mp4_ctx, moov_buf, moov_position);
    return moov_position;
}

int mdhd_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
    int size = 0;
    int moov_position = 0;

    size = create_box_full(moov_buf + moov_position, "mdhd", 32, 0, 1);

    // timescale for metadata track
    Save32BigEndian(MEDIA_TIME_SCALE, moov_buf + moov_position + 20);

    // // orignal duration based on metadata timestamps
    // if (mp4_ctx->metadata_len > 0) {
    //     Save32BigEndian(mp4_ctx->metadata_timestamp_buffer[mp4_ctx->metadata_len - 1] -
    //                     mp4_ctx->metadata_timestamp_buffer[0],
    //                     moov_buf + moov_position + 24);
    // } else {
    //     Save32BigEndian(0, moov_buf + moov_position + 24);
    // }

    // duration based on metadata timestamps (in 90kHz ticks)
    if (mp4_ctx->metadata_len > 0) {
        unsigned int duration = mp4_ctx->video_timestamp_buffer[mp4_ctx->root.video_len - 1] - mp4_ctx->metadata_timestamp_buffer[0];
        Save32BigEndian(duration, moov_buf + moov_position + 24);
    } else {
        Save32BigEndian(0, moov_buf + moov_position + 24);
    }

    // language = 'und' (undetermined), quality = 0
    Save32BigEndian(0x55c40000, moov_buf + moov_position + 28);

    moov_position += size;
    return moov_position;
}

int mdia_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
    int size = 0;
    int moov_position = 0;
    size = create_box(moov_buf + moov_position, "mdia", 8);
    moov_position += size;
    size = mdhd_box_metadata(moov_buf + moov_position, mp4_ctx);
    moov_position += size;
    size = hdlr_box_metadata(moov_buf + moov_position, mp4_ctx);
    moov_position += size;
    size = minf_box_metadata(moov_buf + moov_position, mp4_ctx);
    moov_position += size;

    update_box_size(mp4_ctx, moov_buf, moov_position);
    return moov_position;
}

int trak_box_metadata(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
    int size = 0;
    int moov_position = 0;
    size = create_box(moov_buf + moov_position, "trak", 8);
    moov_position += size;
    size = tkhd_box_metadata(moov_buf + moov_position, mp4_ctx);
    moov_position += size;
    size = mdia_box_metadata(moov_buf + moov_position, mp4_ctx);
    moov_position += size;

    update_box_size(mp4_ctx, moov_buf, moov_position);
    return moov_position;
}

/* ============================================================================
 * Camera calibration -> movie-level 'udta' box (calib only, no GPS)
 *
 *   udta
 *   └── meta (fullbox v0/flags0)
 *       ├── hdlr  handler_type='mdta', name='appl'
 *       ├── keys  (namespace 'mdta') -> 9 keys "com.realtek.eis.ldc.*"
 *       └── ilst  one item per key, UTF-8 decimal-string value
 *
 * Box sizes / types / counts are BIG-ENDIAN (ISO BMFF).
 * Values are UTF-8 number strings, readable via av_dict_get / strtod.
 * ========================================================================== */

void calib_put32_be(unsigned char *buf, uint32_t v)
{
      buf[0] = (unsigned char)(v >> 24);
      buf[1] = (unsigned char)(v >> 16);
      buf[2] = (unsigned char)(v >> 8);
      buf[3] = (unsigned char)(v & 0xFF);
}

/* ilst 'data' atom:  [size]['data'][type=1][locale=0][value UTF-8]
   size = 16 + len (8 box header + 4 type + 4 locale + len value). */
static int calib_data_atom(unsigned char *buf, const char *str)
{
      int len = (int)strlen(str);
      int pos = 0;
      calib_put32_be(buf + pos, 16 + len); /* size */
      pos += 4;
      memcpy(buf + pos, "data", 4);               /* fourcc */
      pos += 4;
      calib_put32_be(buf + pos, 0x00000001);      /* type = 1 (UTF-8) */
      pos += 4;
      calib_put32_be(buf + pos, 0);               /* locale = 0 */
      pos += 4;
      memcpy(buf + pos, str, len);                /* value string */
      pos += len;
      return pos;
}

/* Build meta -> hdlr('mdta') / keys / ilst in buf. Returns size.
   Non-static: called from lifetime_recording_initialize.c (camera_calib_set_udta). */
int camera_calib_meta_box(unsigned char *buf)
{
      int pos = 0;
      int calib_cnt = (int)CAMERA_CALIB_KEY_COUNT;

      /* meta (fullbox) */
      int meta_start = pos;
      calib_put32_be(buf + pos, 0);
      pos += 4;
      memcpy(buf + pos, "meta", 4);
      pos += 4;
      calib_put32_be(buf + pos, 0); /* version/flags = 0 */
      pos += 4;

      /* hdlr: handler_type MUST be 'mdta' for the keys/mdta mechanism */
      int hdlr_start = pos;
      calib_put32_be(buf + pos, 0);
      pos += 4;
      memcpy(buf + pos, "hdlr", 4);
      pos += 4;
      calib_put32_be(buf + pos, 0);         /* version/flags */
      pos += 4;
      calib_put32_be(buf + pos, 0);         /* pre_defined */
      pos += 4;
      memcpy(buf + pos, "mdta", 4);         /* handler_type = mdta */
      pos += 4;
      memset(buf + pos, 0, 12);             /* reserved[3] */
      pos += 12;
      memcpy(buf + pos, "appl", 4);         /* name */
      pos += 4;
      buf[pos] = 0;                         /* null terminator */
      pos += 1;
      calib_put32_be(buf + hdlr_start, (uint32_t)(pos - hdlr_start));

      /* keys (FullBox): [size]['keys'][version/flags=0][entry_count][entries] */
      int keys_start = pos;
      calib_put32_be(buf + pos, 0);
      pos += 4;
      memcpy(buf + pos, "keys", 4);
      pos += 4;
      calib_put32_be(buf + pos, 0);         /* version/flags = 0 */
      pos += 4;
      calib_put32_be(buf + pos, (uint32_t)calib_cnt);   /* entry count */
      pos += 4;

      for (int i = 0; i < calib_cnt; i++) {
              const char *key = g_camera_calib_keys[i].key;
              int key_len = (int)strlen(key);
              /* Standard keys entry (ISO 14496-12):
                 [4B entry_size = 8 + len(key)]
                 [4B 'mdta']                <- key namespace
                 [key 名字符串, bare]        <- NO length prefix, NO repeat 'mdta' */
              calib_put32_be(buf + pos, (uint32_t)(8 + key_len));  /* entry_size */
              pos += 4;
              memcpy(buf + pos, "mdta", 4);  /* namespace */
              pos += 4;
              memcpy(buf + pos, key, key_len);  /* bare key name */
              pos += key_len;
      }
      calib_put32_be(buf + keys_start, (uint32_t)(pos - keys_start));

      /* ilst: one item per key, index 1..N, each a UTF-8 string data atom */
      int ilst_start = pos;
      calib_put32_be(buf + pos, 0);
      pos += 4;
      memcpy(buf + pos, "ilst", 4);
      pos += 4;

      char val_str[32];
      for (int i = 0; i < calib_cnt; i++) {
              /* ilst item (two-level box):
                 [4B item_size][4B key_index][ data box ] */
              int item_start = pos;
              calib_put32_be(buf + pos, 0);             /* item_size patched */
              pos += 4;
              calib_put32_be(buf + pos, (uint32_t)(i + 1));   /* key_index 1..N */
              pos += 4;
              snprintf(val_str, sizeof(val_str), "%.8f", g_camera_calib_keys[i].value);
              pos += calib_data_atom(buf + pos, val_str);     /* data box */
              calib_put32_be(buf + item_start, (uint32_t)(pos - item_start)); /* item_size = 8 + data_len */
      }
      calib_put32_be(buf + ilst_start, (uint32_t)(pos - ilst_start));

      /* patch meta size */
      calib_put32_be(buf + meta_start, (uint32_t)(pos - meta_start));
      return pos;
}


