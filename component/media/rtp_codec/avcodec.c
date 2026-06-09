/* general codec content for av profile*/
#include <platform_stdlib.h>
#include "avcodec.h"

void get_codec_by_id(struct codec_info *c, int id)
{
	int codec_id = id;
	if (codec_id >= AVCODEC_SIZE || codec_id < 0) {
		printf("\n\runknown id! cannot get codec info.");
		return;
	}
	memcpy(c, &av_codec_tables[id], sizeof(struct codec_info));
}