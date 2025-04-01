cmake_minimum_required(VERSION 3.2)

project(nn_postprocess)

set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_C_COMPILER "gcc" )
set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu99 -O3")

set(viplite "VIPLiteDrv_2.0.0")
set (prj_root "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set (sdk_root "${CMAKE_CURRENT_SOURCE_DIR}/../../../..")

list(
	APPEND src
	src/main.c
	src/model_yolo_sim.c
	src/model_yolov9_sim.c
	src/model_nanodet_sim.c
	src/model_scrfd_sim.c
	src/img_render.c
	src/nbg_reader.c

	${prj_root}/src/test_model/nn_utils/class_name.c
	${prj_root}/src/test_model/nn_utils/iou.c
	${prj_root}/src/test_model/nn_utils/nms.c
	${prj_root}/src/test_model/nn_utils/quantize.c
	${prj_root}/src/test_model/nn_utils/sigmoid.c
	${prj_root}/src/test_model/nn_utils/tensor.c
)

list(
	APPEND inc_path
	${prj_root}/src/test_model
	${prj_root}/src/test_model/nn_utils

	${sdk_root}/component/media/mmfv2

	${prj_root}/src/${viplite}/include/nbg_linker
	${prj_root}/src/${viplite}/include
	${prj_root}/src/${viplite}/driver/inc
	${prj_root}/src/${viplite}/hal/inc
	${prj_root}/src/${viplite}/hal/user
	${prj_root}/src/${viplite}/hal/user/freeRTOS

	${sdk_root}/component/image/3rdparty/stb

	${prj_root}/src/nn_post_process_pc_dev_tool/lib
	${prj_root}/src/nn_post_process_pc_dev_tool/lib/NBGParser
)

list(
	APPEND flags
	PC_SIMULATION=1
	__fp16=float #TBD
)

ADD_LIBRARY (libNBGParser STATIC IMPORTED )
SET_PROPERTY ( TARGET libNBGParser PROPERTY IMPORTED_LOCATION ${prj_root}/src/nn_post_process_pc_dev_tool/lib/NBGParser/libNBGParser.a )

add_executable(nn_postprocess ${src})
target_compile_definitions(nn_postprocess PRIVATE ${flags})
target_include_directories(nn_postprocess PUBLIC ${inc_path})
target_link_libraries(nn_postprocess PUBLIC libNBGParser m c gcc)
