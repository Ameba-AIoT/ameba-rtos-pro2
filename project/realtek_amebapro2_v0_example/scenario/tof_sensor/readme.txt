Example Description

tof_dist_array_init.c

This example demonstrates how to use a ToF (Time-of-Flight) sensor to display distance measurements in an 4x4/8x8 array format.

1. Connect ToF sensor SDA(PE_4) and SCL(PE_3) to AmebaPro2. 
  
2. In main.c, uncomment tof_dist_array_init().

3. Get the distance measurements in an 4x4/8x8 array format.


tof_isp_osd_dist_init.c

This example demonstrates how to use a ToF (Time-of-Flight) sensor to display distance measurements in an 8x8 array format overlay onto video stream.

1. Connect ToF sensor SDA(PE_4) and SCL(PE_3) to AmebaPro2. 
  
2. In main.c, uncomment tof_isp_osd_dist_init().

3. Open VLC to view the video stream. The 8×8 distance array will be shown as an overlay once data becomes available.


tof_isp_osd_region_init.c

1. Connect ToF sensor SDA(PE_4) and SCL(PE_3) to AmebaPro2. 
  
2. In main.c, uncomment tof_isp_osd_region_init().

3. Open VLC to view the video stream. A box indicate the nearest valid distance, and a bounding box is drawn based on the ToF and camera FOV to indicate the detection region.

