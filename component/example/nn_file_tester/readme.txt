Description:

	Read up to images from SD card and detect with nn model, and save nn result in SD card.


Bulid Steps:

	1. Mark out one of the NN file tester in example_nn_file_tester.c:

	// object detection nn tester
	mmf2_example_vipnn_objectdet_test_init();
	// face detection nn tester
	//mmf2_example_vipnn_facedet_test_init();
	// face recognition nn tester
	//mmf2_example_vipnn_facerecog_test_init();

	2. Generate the make files.

	cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DEXAMPLE=nn_file_tester

	3. Build flash binary.

	make flash_nn -j4


User Guide:

	1. Create a dataset list of your images and define the file name in mmf2_example_vipnn_objectdet_test_init.c

	Ex: #define FILELIST_NAME "coco_val2017_list.txt"

	2. Place your dataset image(*.jpg) and dataset list(.txt) to SD card

	Ex: copy sample images in file_sample/ and sample dataset lists in file_list/ to SD card

	3. Rebuild the FW, download it to device and reboot. The NN inference results will be saved to SD card with same path


Note:

	1. User need to check the pre-process and post-process of their model

	2. User can modify the filesaver handler(ex: nn_save_handler_for_evaluate) to save the results with their customized format


Supported List:

	AmebaPro2