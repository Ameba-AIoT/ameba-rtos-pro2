#include "osdep_service.h"
#include "uvc_os_wrap_via_osdep_api.h"
#include "basic_types.h"
#include "video.h"
#include "uvc/inc/usbd_uvc_desc.h"
#include "example_media_uvcd.h"
#define JPEG_HEIGHT 1080
#define JPEG_WIDTH  1920
struct UVC_INPUT_HEADER_DESCRIPTOR(1, 4) uvc_input_header = {
	.bLength = UVC_DT_INPUT_HEADER_SIZE(1, 1),
	.bDescriptorType = USB_DT_CS_INTERFACE,
	.bDescriptorSubType = UVC_VS_INPUT_HEADER,
	.bNumFormats  = 1,
	.wTotalLength  = 0,
	.bEndpointAddress  = 0,
	.bmInfo  = 0,
	.bTerminalLink  = 4,
	.bStillCaptureMethod  = 0,
	.bTriggerSupport  = 0,
	.bTriggerUsage  = 0,
	.bControlSize  = 1,
	.bmaControls[0][0]  = 0,
	.bmaControls[1][0]  = 0,
	.bmaControls[2][0]  = 0,
	.bmaControls[3][0]  = 0,
};

struct uvc_format_mjpeg uvc_format_mjpg = {
	.bLength		= UVC_DT_FORMAT_MJPEG_SIZE,
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FORMAT_MJPEG,
	.bFormatIndex		= 1,
	.bNumFrameDescriptors	= 1,
	.bmFlags		= 1,
	.bDefaultFrameIndex	= 1,
	.bAspectRatioX		= 0,
	.bAspectRatioY		= 0,
	.bmInterfaceFlags	= 0,
	.bCopyProtect		= 0,
};

struct UVC_FRAME_MJPEG(3) uvc_frame_mjpg_1080p = {
	.bLength		= UVC_DT_FRAME_MJPEG_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_MJPEG,
	.bFrameIndex		= 1,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(JPEG_WIDTH),
	.wHeight		= cpu_to_le16(JPEG_HEIGHT),
	.dwMinBitRate		= cpu_to_le32(JPEG_WIDTH *JPEG_HEIGHT * 9),
	.dwMaxBitRate		= cpu_to_le32(JPEG_WIDTH *JPEG_HEIGHT * 27),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(460800),
	.dwDefaultFrameInterval	= cpu_to_le32(666666),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(666666),
	.dwFrameInterval[1]	= cpu_to_le32(1000000),
	.dwFrameInterval[2]	= cpu_to_le32(5000000),
};

struct uvc_descriptor_header *uvc_fs_streaming_cls[] = {
	(struct uvc_descriptor_header *) &uvc_input_header,
	(struct uvc_descriptor_header *)  &uvc_format_mjpg,
	(struct uvc_descriptor_header *)  &uvc_frame_mjpg_1080p,
	(struct uvc_descriptor_header *) &uvc_color_matching,
	NULL,
};
struct uvc_descriptor_header *uvc_hs_streaming_cls[] = {
	(struct uvc_descriptor_header *) &uvc_input_header,
	(struct uvc_descriptor_header *)  &uvc_format_mjpg,
	(struct uvc_descriptor_header *)  &uvc_frame_mjpg_1080p,
	(struct uvc_descriptor_header *) &uvc_color_matching,
	NULL,
};
struct usb_descriptor_header *usbd_uvc_descriptors_FS[] = {
	(struct usb_descriptor_header *) &uvc_iad,
	(struct usb_descriptor_header *) &uvc_control_intf,
	(struct usb_descriptor_header *) &uvc_control_header,
	(struct usb_descriptor_header *) &uvc_camera_terminal,
	(struct usb_descriptor_header *) &uvc_processing,
	(struct usb_descriptor_header *) &uvc_extension_unit,
	(struct usb_descriptor_header *) &uvc_output_terminal,
	(struct usb_descriptor_header *) &uvc_control_ep,
	(struct usb_descriptor_header *) &uvc_control_cs_ep,
	(struct usb_descriptor_header *) &uvc_streaming_intf_alt0,
	(struct usb_descriptor_header *) &uvc_input_header,
	(struct usb_descriptor_header *)  &uvc_format_mjpg,
	(struct usb_descriptor_header *)  &uvc_frame_mjpg_1080p,
	(struct usb_descriptor_header *) &uvc_color_matching,
	(struct usb_descriptor_header *) &uvc_streaming_intf_alt1,
	(struct usb_descriptor_header *) &uvc_fs_streaming_ep,
	NULL,
};
struct usb_descriptor_header *usbd_uvc_descriptors_HS[] = {
	(struct usb_descriptor_header *) &uvc_iad,
	(struct usb_descriptor_header *) &uvc_control_intf,
	(struct usb_descriptor_header *) &uvc_control_header,
	(struct usb_descriptor_header *) &uvc_camera_terminal,
	(struct usb_descriptor_header *) &uvc_processing,
	(struct usb_descriptor_header *) &uvc_extension_unit,
	(struct usb_descriptor_header *) &uvc_output_terminal,
	(struct usb_descriptor_header *) &uvc_control_ep,
	(struct usb_descriptor_header *) &uvc_control_cs_ep,
	(struct usb_descriptor_header *) &uvc_streaming_intf_alt0,
	(struct usb_descriptor_header *) &uvc_input_header,
	(struct usb_descriptor_header *)  &uvc_format_mjpg,
	(struct usb_descriptor_header *)  &uvc_frame_mjpg_1080p,
	(struct usb_descriptor_header *) &uvc_color_matching,
	(struct usb_descriptor_header *) &uvc_streaming_intf_alt1,
	(struct usb_descriptor_header *) &uvc_hs_streaming_ep,
	NULL,
};

struct uvc_frame_info uvc_frames_mjpg[] = {
	{ JPEG_WIDTH, JPEG_HEIGHT, { VALUE_FPS(30), VALUE_FPS(15), VALUE_FPS(10), 0 },},
	{ 0, 0, { 0, }, },
};
struct uvc_format_info uvc_formats[] = {
	{ FORMAT_TYPE_MJPEG, uvc_frames_mjpg },
};
