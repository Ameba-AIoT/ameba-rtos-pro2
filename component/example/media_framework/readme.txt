Please follow the instructions for various options

For AmebaPro:
    1. Set the parameter CONFIG_EXAMPLE_MEDIA_FRAMEWORK to 1 in platform_opts.h file

    2. Uncomment the example you want to run in example_mmf2_signal_stream_main()
 
    3. If you are running H264 and 2way audio examples, please copy the header file: inc/lwipopts.h to $SDK/project/realtek_amebapro_v0_example/inc

MMF usage for AmebaPro2:
    Note: the media_framwork example is not included the video related exmaples and NN examples now.
    If users want to use those examples, please refer to \project\realtek_amebapro2_v0_example\src\mmfv2_video_example.

    1. In project\realtek_amebapro2_v0_example\inc\platform_opts.h, users need to select the usage sensor.

    2. In component\example\media_framework\example_media_framework.c, users could select the example by uncommenting the example in function example_mmf2_audio_only.
    Note: please uncomment one example in the same time

    3. cd project/realtek_amebapro2_v0_example/GCC-RELEASE

	4. mkdir build

	5. cd build

    6. Use “cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DEXAMPLE=media_framework” to build up the project.

    7. cmake --build . --target flash

    About the application detail, users could refer to chapter Multimedia Framework - Architecture Using the MMF example, which is in the file AN0700 Realtek AmebaPro2 application note.en.pdf

MMF usage for AmebaD:

    1. Set the parameter CONFIG_EXAMPLE_MEDIA_FRAMEWORK to 1 in platform_opts.h file.

    2. Uncomment the example you want to run in example_mmf2_audio_only(). Since AmebaD support only
	
        audio examples, you cannot run video examples in example_mmf2_video_surport().
	
        (mmf2_example_i2s_audio_init() is not test yet)

    3. AAC and AAD modules need psram heap.If you run examples requiring AAC and AAD modules, you
	
        should enable psram in

        $SDK/component/soc/realtek/amebad/fwlib/usrcfg/rtl8721dhp_intfcfg.c

        Furthermore, if you run examples requiring AAC modules, you should also set a proper space for

        psram bss section and psram heap size. Because AAC library requires at least 0x10008 bytes bss section.
	
        For example :

        PSRAMCFG_TypeDef psram_dev_config = {
        .psram_dev_enable = TRUE,			//enable psram
        .psram_dev_cal_enable = FALSE,			//enable psram calibration function
        .psram_dev_retention = FALSE,			//enable psram retention
        .psram_heap_start_address = 0x02020000,		// MMF example using AAC module require more than 0x10008 bytes bss section. 
        .psram_heap_size = 0x400000-0x20000,		// Therefore, heap size for MMF example should be 0x400000-0x20000
        };

    4. Uncomment needed makefile sections:

        @make -C media all

        in $SDK/project/realtek_amebaD_cm4_gcc_verification/asdk/make/Makefile.

        # MMF AUDIO EXAMPLE
        CSRC += $(DIR)/media_framework/example_media_framework.c
        CSRC += $(DIR)/media_framework/mmf2_example_audioloop_init.c
        CSRC += $(DIR)/media_framework/mmf2_example_pcmu_array_rtsp_init.c
        CSRC += $(DIR)/media_framework/mmf2_example_aac_array_rtsp_init.c
        CSRC += $(DIR)/media_framework/mmf2_example_a_init.c
        CSRC += $(DIR)/media_framework/mmf2_example_rtp_aad_init.c
        CSRC += $(DIR)/media_framework/mmf2_example_2way_audio_init.c
        CSRC += $(DIR)/media_framework/mmf2_example_aacloop_init.c
        CSRC += $(DIR)/media_framework/mmf2_example_g711loop_init.c

        in $SDK/project/realtek_amebaD_cm4_gcc_verification/asdk/make/utilities_example/Makefile.

        # MMF LIBRARY
        LINK_APP_LIB += $(ROOTDIR)/lib/application/lib_rtsp.a
        LINK_APP_LIB += $(ROOTDIR)/lib/application/lib_g711.a
        LINK_APP_LIB += $(ROOTDIR)/lib/application/lib_speex.a
        LINK_APP_LIB += $(ROOTDIR)/lib/application/lib_haac.a
        LINK_APP_LIB += $(ROOTDIR)/lib/application/lib_faac.a
        LINK_APP_LIB += $(ROOTDIR)/lib/application/lib_cmsis_dsp.a

        #	MMF LIBRARY
        @make -C make/audio/g711 all
        @make -C make/audio/faac all
        @make -C make/audio/haac all
        @make -C make/audio/speex all
        @make -C make/network/rtsp all
        @make -C make/cmsis-dsp all

        in $SDK/project/realtek_amebaD_cm4_gcc_verification/asdk/Makefile

    5. Use "make xip" in CM4 for the first build to build libraries (g711, haac, faac, speex, rtsp and cmsis_dsp).