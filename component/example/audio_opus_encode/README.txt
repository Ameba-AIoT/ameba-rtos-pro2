This opus encode example is used to encode PCM data to opus data with ogg container. In order to run the example the following steps must be followed.

	1. Set the parameter CONFIG_EXAMPLE_AUDIO_OPUS_ENCODE to 1 in platform_opts.h file
	
	2. In function opus_audio_opus_encode_thread(...), you can change source_file and dest_file to play your audio files.
	
	3. Build and flash the binary to test

[Supported List]
	Supported :
	    Ameba-D, AmebaPro, AmebaPro2
	Source code not in project:
