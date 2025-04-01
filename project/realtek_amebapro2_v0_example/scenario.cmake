cmake_minimum_required(VERSION 3.6)

enable_language(C CXX ASM)

#MMF_MODULE
list(
    APPEND app_sources
    ${sdk_root}/component/media/mmfv2/module_video.c
    ${sdk_root}/component/media/mmfv2/module_rtsp2.c
    ${sdk_root}/component/media/mmfv2/module_array.c
    ${sdk_root}/component/media/mmfv2/module_audio.c
    ${sdk_root}/component/media/mmfv2/module_aac.c
    ${sdk_root}/component/media/mmfv2/module_aad.c
    ${sdk_root}/component/media/mmfv2/module_g711.c
    ${sdk_root}/component/media/mmfv2/module_httpfs.c
    ${sdk_root}/component/media/mmfv2/module_i2s.c
    ${sdk_root}/component/media/mmfv2/module_mp4.c
    ${sdk_root}/component/media/mmfv2/module_rtp.c
    ${sdk_root}/component/media/mmfv2/module_opusc.c
    ${sdk_root}/component/media/mmfv2/module_opusd.c
    ${sdk_root}/component/media/mmfv2/module_uvcd.c
    ${sdk_root}/component/media/mmfv2/module_demuxer.c
    ${sdk_root}/component/media/mmfv2/module_eip.c
    ${sdk_root}/component/media/mmfv2/module_md.c
    ${sdk_root}/component/media/mmfv2/module_fmp4.c
    ${sdk_root}/component/media/mmfv2/module_fileloader.c
    ${sdk_root}/component/media/mmfv2/module_filesaver.c
    ${sdk_root}/component/media/mmfv2/module_queue.c
)

#NN module
list(
    APPEND app_sources

    ${sdk_root}/component/media/mmfv2/module_vipnn.c
    ${sdk_root}/component/media/mmfv2/module_facerecog.c
)

#SVM
list(
	APPEND scn_sources
	${prj_root}/src/test_model/svm/svm.cpp
	#sim_io
	${prj_root}/src/test_model/svm/sim_io/sim_io.c
	#FastLZ
	${prj_root}/src/test_model/svm/fastlz/fastlz.c
)

#NN MODEL
list(
	APPEND scn_sources
	${prj_root}/src/test_model/model_yolo.c
	${prj_root}/src/test_model/model_yolov9.c
	${prj_root}/src/test_model/model_yamnet.c
	${prj_root}/src/test_model/model_yamnet_s.c
	${prj_root}/src/test_model/model_landmark_sim.c
	${prj_root}/src/test_model/model_mobilefacenet.c
	${prj_root}/src/test_model/model_scrfd.c
	${prj_root}/src/test_model/model_nanodet.c
	${prj_root}/src/test_model/mel_spectrogram.c
	${prj_root}/src/test_model/model_palm_detection.c
	${prj_root}/src/test_model/model_hand_landmark.c
	${prj_root}/src/test_model/model_mobilenetv2.c
)

#NN utils
list(
	APPEND scn_sources
	${prj_root}/src/test_model/nn_utils/sigmoid.c
	${prj_root}/src/test_model/nn_utils/quantize.c
	${prj_root}/src/test_model/nn_utils/iou.c
	${prj_root}/src/test_model/nn_utils/nms.c
	${prj_root}/src/test_model/nn_utils/tensor.c
	${prj_root}/src/test_model/nn_utils/class_name.c

	${prj_root}/src/test_model/roi_delta_qp/roi_delta_qp.c
)

#USER
list(
	APPEND scn_sources
	${prj_root}/src/main.c
)

if(DEFINED EXAMPLE AND EXAMPLE)
	message(STATUS "EXAMPLE = ${EXAMPLE}")
	if(EXISTS ${sdk_root}/component/example/${EXAMPLE})
		if(EXISTS ${sdk_root}/component/example/${EXAMPLE}/${EXAMPLE}.cmake)
			message(STATUS "Found ${EXAMPLE} include project")
			include(${sdk_root}/component/example/${EXAMPLE}/${EXAMPLE}.cmake)
		else()
			message(WARNING "Found ${EXAMPLE} include project but ${EXAMPLE}.cmake not exist")
		endif()
	else()
		message(WARNING "${EXAMPLE} Not Found")
	endif()
	if(NOT DEBUG)
		set(EXAMPLE OFF CACHE STRING INTERNAL FORCE)
	endif()
elseif(DEFINED FAST_INF_EXAMPLE AND FAST_INF_EXAMPLE)
	message(STATUS "Build FAST_INF_EXAMPLE project")
	include(${prj_root}/src/fast_inf_example/fast_inf_example.cmake)
	if(NOT DEBUG)
		set(FAST_INF_EXAMPLE OFF CACHE STRING INTERNAL FORCE)
	endif()
elseif(DEFINED VIDEO_EXAMPLE AND VIDEO_EXAMPLE)
	message(STATUS "Build VIDEO_EXAMPLE project")
	include(${prj_root}/src/mmfv2_video_example/video_example_media_framework.cmake)
	if(NOT DEBUG)
		 set(VIDEO_EXAMPLE OFF CACHE STRING INTERNAL FORCE)
	endif()
elseif(DEFINED SELF_TEST AND SELF_TEST)
	message(STATUS "SELF_TEST = ${SELF_TEST}")
	include(${prj_root}/src/self_test/${SELF_TEST}/${SELF_TEST}.cmake)
	if(NOT DEBUG)
		set(SELF_TEST OFF CACHE STRING INTERNAL FORCE)
	endif()
elseif(DEFINED AUDIO_TEST_TOOL AND AUDIO_TEST_TOOL)
	message(STATUS "Build AUDIO_TEST_TOOL project")
	include(${prj_root}/src/internal/audio_test_tool/audio_test_tool.cmake OPTIONAL RESULT_VARIABLE audio_test_internal)
	if (audio_test_internal)
		message(STATUS "Internal Test Version")
	else()
		include(${prj_root}/src/audio_test_tool/audio_test_tool.cmake OPTIONAL RESULT_VARIABLE audio_test_release)
		if (NOT audio_test_release)
			message(STATUS "Audio Test Tool Not Release")
		endif()
	endif()
	if(NOT DEBUG)
		set(AUDIO_TEST_TOOL OFF CACHE STRING INTERNAL FORCE)
	endif()
elseif(DEFINED NN_TESTER_EXAMPLE AND NN_TESTER_EXAMPLE)
	message(STATUS "Build NN_TESTER_EXAMPLE project")
	include(${prj_root}/src/internal/nn_tester/example_file_vipnn_tester.cmake)
	if(NOT DEBUG)
		set(NN_TESTER_EXAMPLE OFF CACHE STRING INTERNAL FORCE)
	endif()
else()
endif()

if(UNITEST)
	include(${prj_root}/src/internal/unitest/unitest.cmake OPTIONAL)
endif()

list(
	APPEND scn_inc_path
	${app_example_inc_path}
	${prj_root}/src/test_model/svm
	${prj_root}/src/test_model
	${prj_root}/src
	${prj_root}/src/${viplite}/sdk/inc
	${prj_root}/src/${viplite}/driver/inc
	${prj_root}/src/${viplite}/hal/inc
	${prj_root}/src/${viplite}/hal/user
	${prj_root}/src/${viplite}/hal/user/freeRTOS
	${prj_root}/src/${viplite}/include
	${prj_root}/src/${viplite}/include/nbg_linker
)

list(
	APPEND scn_flags
	${app_example_flags}
)

list(
	APPEND scn_libs
	${app_example_lib}
)

list(
	APPEND scn_sources
	${app_example_sources}
)