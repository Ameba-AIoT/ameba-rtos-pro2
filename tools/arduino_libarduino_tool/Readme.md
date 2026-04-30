# Build libarduio.a
- libarduino.a

## Process
In MSYS, navigate to:
>  ~/project/realtek_amebapro2_v0_example/GCC-RELEASE/build

Run:
> ./../../../../tools/arduino_libarduino_tool/libarduino_tool.exe

The output will be generated at:
> ~/arduino_libarduino_tool/output

Copy `libarduino.a` into the Arduino SDK:
> ~/Arduino15/packages/realtek/hardware/AmebaPro2/<versions>/variants/common_libs

## Other lib files
Rebuilding the application source files for `libarduino.a` will also generate two additional libraries.
It is not recommended to update these:
- libiperf3.a
- liboutsrc.a

## C/C++ files built into libarduino.a
Please review carefully before making any modifications.

```text
ainr.c.obj
AL3042.c.obj
ambient_light_sensor.c.obj
app_example.c.obj
atcmd_bt.c.obj
atcmd_ethernet.c.obj
atcmd_ftl.c.obj
atcmd_isp.c.obj
atcmd_mp.c.obj
atcmd_mp_ext2.c.obj
atcmd_mqtt.c.obj
atcmd_sys.c.obj
atcmd_wifi.c.obj
cJSON.c.obj
class_name.c.obj
cmd_shell.c.obj
console_auth.c.obj
dfu_8735b.c.obj
dhcps.c.obj
eip_auto_wdr.c.obj
ethernet_mii.c.obj
ethernet_usb.c.obj
fastlz.c.obj
ftl.c.obj
ftl_common_api.c.obj
ftl_nand_api.c.obj
ftl_nor_api.c.obj
fwfs.c.obj
httpc_tls.c.obj
httpd_tls.c.obj
http_client.c.obj
iou.c.obj
iperf.c.obj
ir_ctrl.c.obj
ir_cut.c.obj
isp_osd_example.c.obj
isp_osd_lite.c.obj
libc_wrap.c.obj
libwsclient.c.obj
log_service.c.obj
low_level_io.c.obj
mel_spectrogram.c.obj
mmf2_video_example_2way_audio_pcmu_doorbell_init.c.obj
mmf2_video_example_2way_audio_pcmu_init.c.obj
mmf2_video_example_array_rtsp_init.c.obj
mmf2_video_example_audio_vipnn_init.c.obj
mmf2_video_example_av21_init.c.obj
mmf2_video_example_av2_init.c.obj
mmf2_video_example_av_init.c.obj
mmf2_video_example_av_mp4_httpfs_init.c.obj
mmf2_video_example_av_mp4_init.c.obj
mmf2_video_example_av_rtsp_mp4_init.c.obj
mmf2_video_example_bayercap_rtsp_init.c.obj
mmf2_video_example_demuxer_rtsp_init.c.obj
mmf2_video_example_dynamic_roi_rtsp_init.c.obj
mmf2_video_example_face_rtsp_init.c.obj
mmf2_video_example_fd_lm_mfn_sim_rtsp_init.c.obj
mmf2_video_example_h264_array_mp4_init.c.obj
mmf2_video_example_h264_pcmu_array_mp4_init.c.obj
mmf2_video_example_joint_test_all_nn_rtsp_init.c.obj
mmf2_video_example_joint_test_init.c.obj
mmf2_video_example_joint_test_rtsp_mp4_init.c.obj
mmf2_video_example_joint_test_rtsp_mp4_init_fcs.c.obj
mmf2_video_example_joint_test_vipnn_rtsp_mp4_init.c.obj
mmf2_video_example_jpeg_external_init.c.obj
mmf2_video_example_md_mp4_init.c.obj
mmf2_video_example_md_nn_rtsp_init.c.obj
mmf2_video_example_md_rtsp_init.c.obj
mmf2_video_example_simo_init.c.obj
mmf2_video_example_timelapse_mp4_init.c.obj
mmf2_video_example_v1_day_night_change_init.c.obj
mmf2_video_example_v1_init.c.obj
mmf2_video_example_v1_mask_init.c.obj
mmf2_video_example_v1_param_change_init.c.obj
mmf2_video_example_v1_rate_control_init.c.obj
mmf2_video_example_v1_snapshot_hr_init.c.obj
mmf2_video_example_v1_snapshot_httpfs_init.c.obj
mmf2_video_example_v1_snapshot_init.c.obj
mmf2_video_example_v2_init.c.obj
mmf2_video_example_v3_init.c.obj
mmf2_video_example_vipnn_classify_rtsp_init.c.obj
mmf2_video_example_vipnn_facedet_init.c.obj
mmf2_video_example_vipnn_facedet_sync_init.c.obj
mmf2_video_example_vipnn_facedet_sync_snapshot_init.c.obj
mmf2_video_example_vipnn_handgesture_init.c.obj
mmf2_video_example_vipnn_rtsp_init.c.obj
model_hand_landmark.c.obj
model_landmark_sim.c.obj
model_mobilefacenet.c.obj
model_mobilenetv2.c.obj
model_nanodet.c.obj
model_palm_detection.c.obj
model_scrfd.c.obj
model_yamnet.c.obj
model_yamnet_s.c.obj
model_yolo.c.obj
model_yolov9.c.obj
module_aac.c.obj
module_aad.c.obj
module_array.c.obj
module_audio.c.obj
module_demuxer.c.obj
module_eip.c.obj
module_facerecog.c.obj
module_fileloader.c.obj
module_filesaver.c.obj
module_fmp4.c.obj
module_g711.c.obj
module_httpfs.c.obj
module_i2s.c.obj
module_md.c.obj
module_mp4.c.obj
module_opusc.c.obj
module_opusd.c.obj
module_queue.c.obj
module_rtp.c.obj
module_rtsp2.c.obj
module_uvcd.c.obj
module_vipnn.c.obj
module_web_viewer.c.obj
mpu_protect.c.obj
MQTTClient.c.obj
MQTTConnectClient.c.obj
MQTTConnectServer.c.obj
MQTTDeserializePublish.c.obj
MQTTFormat.c.obj
MQTTFreertos.c.obj
MQTTPacket.c.obj
MQTTSerializePublish.c.obj
MQTTSubscribeClient.c.obj
MQTTSubscribeServer.c.obj
MQTTUnsubscribeClient.c.obj
MQTTUnsubscribeServer.c.obj
nms.c.obj
nnlite_api.c.obj
nn_file_op.c.obj
osd_render.c.obj
ota_8735b.c.obj
ota_8735b_fwfs.c.obj
ping_test.c.obj
quantize.c.obj
roi_delta_qp.c.obj
rtl_console.c.obj
rtp_api.c.obj
rtsp_api.c.obj
rtw_opt_crypto_ssl.c.obj
rtw_opt_skbuf_rtl8735b.c.obj
sdp.c.obj
sensor_service.c.obj
sigmoid.c.obj
sim_io.c.obj
sntp.c.obj
sn_coap_ameba_port.c.obj
sn_coap_builder.c.obj
sn_coap_header_check.c.obj
sn_coap_parser.c.obj
sn_coap_protocol.c.obj
ssl_client.c.obj
ssl_client_ext.c.obj
svm.cpp.obj
system_data_api.c.obj
tensor.c.obj
tftp_client.c.obj
tftp_server.c.obj
tls_polarssl.c.obj
ulaw_decode_lookup.S.obj
ulaw_encode_lookup.S.obj
ulaw_mixup_decode_lookup.S.obj
vfs.c.obj
vfs_fatfs.c.obj
vfs_littlefs.c.obj
vfs_wrap.c.obj
video_example_media_framework.c.obj
video_snapshot.c.obj
wifi_conf.c.obj
wifi_conf_ext.c.obj
wifi_conf_inter.c.obj
wifi_conf_promisc.c.obj
wifi_conf_wowlan.c.obj
wifi_eap_config.c.obj
wifi_fast_connect.c.obj
wifi_ind.c.obj
wifi_promisc.c.obj
wifi_wps_config.c.obj
wlan_network.c.obj
wsclient_api.c.obj
wsclient_tls.c.obj
