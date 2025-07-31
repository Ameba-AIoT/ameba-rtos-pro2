Please follow the instructions for various options
MMF usage for AmebaPro2:
    1. In project\realtek_amebapro2_v0_example\inc\platform_opts.h, users need to select the usage sensor.

    2. In project\realtek_amebapro2_v0_example\src\mmfv2_video_example\video_example_media_framework.c, users could select the example by uncommenting the example in function example_mmf2_video_surport.
    Note: please uncomment one example in the same time
    Note: If user want to use the examples, please check the defuault example "mmf2_video_example_v1_init" is comment

    3. cd project/realtek_amebapro2_v0_example/GCC-RELEASE

    4. mkdir build

    5. cd build

    6. cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DVIDEO_EXAMPLE=on to build up the project.

    7. cmake --build . --target flash

    About the application detail, users could refer to chapter Multimedia Framework - Architecture Using the MMF example, which is in the file AN0700 Realtek AmebaPro2 application note.en.pdf
    Audio + NN examples:

    1.  mmf2_video_example_audio_vipnn_init:

    The sound received by AmebaPro2 can be transmitted to NN engine to do sound classification.

    Please see NN chapter for more details

    Video + Audio +FCS:

    1.  mmf2_video_example_joint_test_rtsp_mp4_init_fcs:

    Please see FCS chapter for more details
