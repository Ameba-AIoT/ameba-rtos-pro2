Description:

    PC tool for developing nn post-process.
    The post-process api interface is same as Pro2 version.

Bulid Steps:

    1. Develop post-process in model_yolo_sim.c
    
    2. Setup tensor params from NBinfo
    
    configure in main.c --> yolo_simulation() --> configure_tensor_param()
    
    3. Get output tensor from Acuity inference
    
    configure output tensor file name in main.c --> yolo_simulation() --> acuity_tensor_name[]

    4. Build project

    mkdir build && cd build
    cmake .. -G"Unix Makefiles"
    make -j4

    5. Execute the progarm

    ./nn_postprocess

    6. The result image with bbox will ba saved in data/yolo_data/prediction.jpg

