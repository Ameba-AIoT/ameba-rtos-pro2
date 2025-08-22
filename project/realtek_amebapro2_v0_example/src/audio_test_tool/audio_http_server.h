#ifndef AUDIO_HTTP_SERVER_H
#define AUDIO_HTTP_SERVER_H

// register what the disk the audio files are located
void httpd_register_audio_disk_tag(const char *tag_name, int tag_len);

// register the audio page in http server (this is register cb if call audio_httpd_create)
void httpd_register_audio_dump_page(void);

// create httpd server for audio dump, if user has construct their http server, just call httpd_register_audio_dump_page before httpd_start
void audio_httpd_create(void);

#endif /* AUDIO_HTTP_SERVER_H */
