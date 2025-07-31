This Helix AAC example is used to play AAC LC files from an binary array. In order to run the example the following steps must be followed.
Helix AAC example

Description:
Helix AAC decoder is an AAC decoder which support AAC LC and AAC HE. (no AAC HEv2)
It has low CPU usage with low memory.

By default this Helix AAC decoder only support AAC LC.
This example show how to use this decoder.

Configuration:
1. [platform_opts.h]
	#define CONFIG_EXAMPLE_AUDIO_HELIX_AAC    1

2. example_audio_helix_aac.c
	#define AUDIO_SOURCE_BINARY_ARRAY (1)
	#define ADUIO_SOURCE_HTTP_FILE    (0)

	This configuration select audio source. The audio source is aac raw data without TS wrapper.

	To test http file as audio source, you need provide a http file location.
	You can use some http file server tool ( Ex. HFS: http://www.rejetto.com/hfs/ )
	Fill address, port, and file name, then you can run this example.

3. The audio source is 8k mono audio. The content is audio_8k_16bit.c and audio.aac.

Note: It only support the mono audio or you need to downsample. 

[Supported List]
	Supported :
	    Ameba-Pro