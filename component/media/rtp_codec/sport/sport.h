#ifndef _SPORT_H
#define _SPORT_H

#include "dlist.h"
#include "basic_types.h"
#include "osdep_service.h"

#define SPORT_DEBUG 1

#if SPORT_DEBUG
#define SPORT_PRINTF(fmt, args...)    printf("\n\r%s: " fmt, __FUNCTION__, ## args)
#define SPORT_ERROR(fmt, args...)     printf("\n\r%s: " fmt, __FUNCTION__, ## args)
#else
#define SPORT_PRINTF(fmt, args...)
#define SPORT_ERROR(fmt, args...)
#endif

// RTP SPORT object (similar to AAC object)
struct rtp_sport_obj {
    u16 sample_size;   // bytes per sample (3 for 24-bit)
    u16 channels;      // number of channels (e.g. 4)
    u32 samplerate;    // sample rate (e.g. 48000 Hz)
};

#endif /* _SPORT_H */
