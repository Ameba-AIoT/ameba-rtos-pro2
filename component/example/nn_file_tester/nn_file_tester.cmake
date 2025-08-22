### add lib ###
list(
	APPEND app_example_lib
)

### add flags ###
list(
	APPEND app_example_flags
)

### add header files ###
list (
	APPEND app_example_inc_path
)

### add source file ###
list(
	APPEND app_example_sources
	${sdk_root}/component/example/nn_file_tester/app_example.c
	${sdk_root}/component/example/nn_file_tester/example_nn_file_tester.c

	${sdk_root}/component/example/nn_file_tester/mmf2_example_vipnn_objectdet_test_init.c
	${sdk_root}/component/example/nn_file_tester/mmf2_example_vipnn_facedet_test_init.c
	${sdk_root}/component/example/nn_file_tester/mmf2_example_vipnn_facerecog_test_init.c
)

list(
	APPEND out_sources

	#ftp
	${sdk_root}/component/network/ftp/FtpClient.c
)