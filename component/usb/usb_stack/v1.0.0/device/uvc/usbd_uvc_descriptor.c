/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Includes ------------------------------------------------------------------*/

#include "usbd.h"
#include "usbd_uvc.h"
#include "usbd_video.h"

/* Private defines -----------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* #define UVC_ORIGINAL_TYPE */
#ifdef UVC_SINGLE_STREAM

/* UVC Device Descriptor Set */
u8 usbd_uvc_descriptors[] = {

	/* =========================
	 * Configuration Descriptor
	 * ========================= */
	0x09, 0x02,             // bLength, bDescriptorType = CONFIGURATION
	0xE4, 0x00,             // wTotalLength = 0x00E4 (228 bytes)
	0x02,                   // bNumInterfaces = 2
	0x01,                   // bConfigurationValue
	0x01,                   // iConfiguration
	0x80,                   // bmAttributes = Bus Powered
	0x32,                   // bMaxPower = 100 mA

	/* =========================
	 * Interface Association Descriptor (IAD)
	 * ========================= */
	0x08, 0x0B,             // bLength, bDescriptorType = IAD
	0x00,                   // bFirstInterface
	0x02,                   // bInterfaceCount (VC + VS)
	0x0E,                   // bFunctionClass = CC_VIDEO
	0x03,                   // bFunctionSubClass = SC_VIDEO_INTERFACE_COLLECTION
	0x00,                   // bFunctionProtocol
	0x00,                   // iFunction

	/* =========================
	 * Interface 0: Video Control (VC)
	 * ========================= */
	0x09, 0x04,             // bLength, bDescriptorType = INTERFACE
	0x00,                   // bInterfaceNumber
	0x00,                   // bAlternateSetting
	0x01,                   // bNumEndpoints
	0x0E,                   // bInterfaceClass = CC_VIDEO
	0x01,                   // bInterfaceSubClass = SC_VIDEOCONTROL
	0x00,                   // bInterfaceProtocol
	0x00,                   // iInterface

	/* =========================
	 * Class-specific VC Header
	 * ========================= */
	0x0D, 0x24, 0x01,       // bLength, CS_INTERFACE, VC_HEADER
	0x00, 0x01,             // bcdUVC = 1.00
	0x4F, 0x00,             // wTotalLength (VC descriptors)
	0x00, 0x6C,             // dwClockFrequency = 6.912 MHz
	0xDC, 0x02,
	0x01,                   // bInCollection
	0x01,                   // baInterfaceNr(1) = VS interface 1

	/* =========================
	 * Input Terminal Descriptor (Camera)
	 * ========================= */
	0x12, 0x24, 0x02,       // INPUT_TERMINAL
	0x01,                   // bTerminalID
	0x01, 0x02,             // wTerminalType = ITT_CAMERA
	0x00,                   // bAssocTerminal
	0x00,                   // iTerminal
	0x00, 0x00,             // wObjectiveFocalLengthMin
	0x00, 0x00,             // wObjectiveFocalLengthMax
	0x00, 0x00,             // wOcularFocalLength
	0x03,                   // bControlSize
	0x00, 0x00, 0x00,       // bmControls

	/* =========================
	 * Processing Unit Descriptor
	 * ========================= */
	0x0B, 0x24, 0x05,       // PROCESSING_UNIT
	0x02,                   // bUnitID
	0x01,                   // bSourceID (Input Terminal 1)
	0x00, 0x40,             // wMaxMultiplier
	0x02,                   // bControlSize
	0xFF, 0xFF,             // bmControls (many processing controls)
	0x00,                   // iProcessing

	/* =========================
	 * Extension Unit Descriptor (Vendor-specific)
	 * ========================= */
	0x1C, 0x24, 0x06,       // EXTENSION_UNIT
	0x03,                   // bUnitID
	0xCE, 0xB9, 0x55, 0x2B, // GUID
	0xB0, 0x14, 0x49, 0xA8,
	0xB2, 0x2E, 0xF6, 0xB0,
	0xA0, 0x12, 0x82, 0x74,
	0x18,                   // bNumControls
	0x01,                   // bNrInPins
	0x02,                   // baSourceID(1) = PU
	0x03,                   // bControlSize
	0x00, 0x1e, 0x00,       // bmControls
	0x00,                   // iExtension

	/* =========================
	 * Output Terminal Descriptor (USB Streaming)
	 * ========================= */
	0x09, 0x24, 0x03,       // OUTPUT_TERMINAL
	0x04,                   // bTerminalID
	0x01, 0x01,             // wTerminalType = TT_STREAMING
	0x00,                   // bAssocTerminal
	0x03,                   // bSourceID (Extension Unit)
	0x00,                   // iTerminal

	/* =========================
	 * VC Interrupt Endpoint
	 * ========================= */
	0x07, 0x05,             // ENDPOINT
	0x81,                   // IN endpoint 1
	0x03,                   // Interrupt
	0x40, 0x00,             // wMaxPacketSize = 64
	0x08,                   // bInterval

	/* =========================
	 * Class-specific Interrupt Endpoint
	 * ========================= */
	0x05, 0x25, 0x03,       // bLength=5, bDescriptorType=CS_ENDPOINT, bDescriptorSubtype=EP_INTERRUPT
	0x40, 0x00,             // wMaxTransferSize = 64

	/* =========================
	 * Interface 1: Video Streaming (Alt 0)
	 * ========================= */
	0x09, 0x04,             // bLength=9, bDescriptorType=INTERFACE
	0x01,                   // bInterfaceNumber = 1
	0x00,                   // bAlternateSetting = 0 (zero-bandwidth)
	0x00,                   // bNumEndpoints = 0
	0x0E,                   // bInterfaceClass = CC_VIDEO
	0x02,                   // bInterfaceSubClass = SC_VIDEOSTREAMING
	0x00,                   // bInterfaceProtocol = PC_PROTOCOL_UNDEFINED
	0x00,                   // iInterface

	/* =========================
	 * VS Header Descriptor
	 * ========================= */
	0x0E, 0x24, 0x01,       // bLength=14, CS_INTERFACE, VS_INPUT_HEADER
	0x01,                   // bNumFormats = 1
	0x56, 0x00,             // wTotalLength = 86 (VS class descriptors)
	USBD_UVC_ISO_IN_EP,     // bEndpointAddress
	0x00,                   // bmInfo
	0x04, 0x00,             // bTerminalLink=4 (OT), bStillCaptureMethod=0
	0x00, 0x00,             // bTriggerSupport=0, bTriggerUsage=0
	0x01, 0x00,             // bControlSize=1, bmaControls[0]=0

	/* =========================
	 * VS Format Descriptor (H.264)
	 * ========================= */
	0x1C, 0x24, 0x10,               // bLength=28, CS_INTERFACE, VS_FORMAT_FRAME_BASED (H.264)
	0x01,                           // bFormatIndex = 1
	0x01,                           // bNumFrameDescriptors = 1
	'H', '2', '6', '4',             // guidFormat[0..3]
	0x00, 0x00,                     // guidFormat[4..5]
	0x10, 0x00,                     // guidFormat[6..7]
	0x80, 0x00,                     // guidFormat[8..9]
	0x00, 0xAA,                     // guidFormat[10..11]
	0x00,                           // guidFormat[12]
	0x38, 0x9B, 0x71, 0x0C,         // guidFormat[13..15], bBitsPerPixel = 12
	0x01,                           // bDefaultFrameIndex = 1
	0x01,                           // bAspectRatioX = 1
	0x00, 0x00, 0x00, 0x01,         // bAspectRatioY, bmInterlaceFlags, bCopyProtect, bVariableSize

	/* =========================
	 * VS Frame Descriptor
	 * ========================= */
	0x26, 0x24, 0x11,               // bLength=38, CS_INTERFACE, VS_FRAME_FRAME_BASED (H.264)
	0x01,                           // bFrameIndex = 1
	0x00,                           // bmCapabilities = 0
	//0x80, 0x02,             // Width = 640
	//0xE0, 0x01,             // Height = 480
	USBD_UVC_WBVAL(USBD_UVC_FRAME_WIDTH),     // wWidth
	USBD_UVC_WBVAL(USBD_UVC_FRAME_HEIGHT),    // wHeight
	0x00, 0x40, 0x19, 0x01,         // dwMinBitRate = 18,432,000 bps
	0x00, 0xC0, 0x4B, 0x03,         // dwMaxBitRate = 55,296,000 bps
	0x15, 0x16, 0x05, 0x00,         // dwDefaultFrameInterval = 333333 x 100ns (30fps)
	0x03,                           // bFrameIntervalType = 3 (discrete)
	0x00, 0x00,                     // dwBytesPerLine (LS word) = 0 (compressed)
	0x00, 0x00,                     // dwBytesPerLine (MS word) = 0
	0x15, 0x16, 0x05, 0x00,         // dwFrameInterval[0] = 333333 x 100ns (30fps)
	0x2A, 0x2C, 0x0A, 0x00,         // dwFrameInterval[1] = 666666 x 100ns (15fps)
	0x40, 0x42, 0x0F, 0x00,         // dwFrameInterval[2] = 1000000 x 100ns (10fps)

	/* =========================
	 * VS Color Matching Descriptor
	 * ========================= */
	0x06, 0x24, 0x0D,       // bLength=6, CS_INTERFACE, VS_COLORFORMAT
	0x01,                   // bColorPrimaries = BT.709/sRGB
	0x01,                   // bTransferCharacteristics = sRGB
	0x04,                   // bMatrixCoefficients = SMPTE 170M (BT.601)

	/* =========================
	 * Interface 1: Video Streaming (Alt 1)
	 * ========================= */
	0x09, 0x04,             // bLength=9, bDescriptorType=INTERFACE
	0x01,                   // bInterfaceNumber = 1
	0x01,                   // bAlternateSetting = 1 (active streaming)
	0x01,                   // bNumEndpoints = 1 (ISOC IN)
	0x0E,                   // bInterfaceClass = CC_VIDEO
	0x02,                   // bInterfaceSubClass = SC_VIDEOSTREAMING
	0x00,                   // bInterfaceProtocol = PC_PROTOCOL_UNDEFINED
	0x00,                   // iInterface

	/* =========================
	 * Isochronous Video Endpoint
	 * ========================= */
	0x07, 0x05,             // bLength=7, bDescriptorType=ENDPOINT
	USBD_UVC_ISO_IN_EP,     // bEndpointAddress
	0x05,                   // bmAttributes = Isochronous, asynchronous
	USBD_UVC_WBVAL(USBD_UVC_ISOC_EP_MPKT), // wMaxPacketSize: bits[12:11]=extra_trans, bits[10:0]=MPS
	0x01,                   // bInterval = 1 microframe
};

int usbd_uvc_descriptors_size = sizeof(usbd_uvc_descriptors);

usbd_uvc_streaming_control_t usbd_uvc_probe = {
	.bmHint = 0,
	.bFormatIndex = 1,
	.bFrameIndex = 1,
	.dwFrameInterval = 666666,
	.wKeyFrameRate = 0,
	.wPFrameRate = 0,
	.wCompQuality = 0,
	.wCompWindowSize = 0,
	.wDelay = 0,
	.dwMaxVideoFrameSize = USBD_TUNING_W * USBD_TUNING_H,
	.dwMaxPayloadTransferSize = 0,
	.dwClockFrequency = USBD_UVC_CLOCK_FREQUENCY,
	.bmFramingInfo = 0,
	.bPreferedVersion = 0,
	.bMinVersion = 0,
	.bMaxVersion = 0,
};

usbd_uvc_streaming_control_t usbd_uvc_commit = {
	.bmHint = 0,
	.bFormatIndex = 1,
	.bFrameIndex = 1,
	.dwFrameInterval = 666666,
	.wKeyFrameRate = 0,
	.wPFrameRate = 0,
	.wCompQuality = 0,
	.wCompWindowSize = 0,
	.wDelay = 0,
	.dwMaxVideoFrameSize = USBD_TUNING_W * USBD_TUNING_H,
	.dwMaxPayloadTransferSize = 0,
	.dwClockFrequency = USBD_UVC_CLOCK_FREQUENCY,
	.bmFramingInfo = 0,
	.bPreferedVersion = 0,
	.bMinVersion = 0,
	.bMaxVersion = 0,
};


usbd_uvc_frame_info_t uvc_frames_h264[] = {
	{ USBD_UVC_FRAME_WIDTH, USBD_UVC_FRAME_HEIGHT, { USBD_UVC_FPS(30), USBD_UVC_FPS(15), USBD_UVC_FPS(10), 0 },},
	{ 0, 0, { 0, }, },
};
usbd_uvc_format_info_t uvcd_formats[] = {
	{ USBD_UVC_FORMAT_TYPE_H264, uvc_frames_h264 },
};
const u32 uvcd_nformats = sizeof(uvcd_formats) / sizeof(uvcd_formats[0]);
#else

u8 usbd_uvc_descriptors[] = {
	/* ===== USB Configuration Descriptor (9 bytes) [USB2.0 9.6.3] ===== */
	0x09,                                           /* bLength */
	USB_DESC_TYPE_CONFIGURATION,                    /* bDescriptorType: CONFIGURATION */
	USB_LOW_BYTE(450 + USBD_UVC_XU_DESC_LEN),       /* wTotalLength (LSB): 450 (+28 if XU enabled) */
	USB_HIGH_BYTE(450 + USBD_UVC_XU_DESC_LEN),      /* wTotalLength (MSB) */
	0x02,                                           /* bNumInterfaces: VideoControl + VideoStreaming */
	0x01,                                           /* bConfigurationValue */
	0x01,                                           /* iConfiguration */
	0x80,                                           /* bmAttributes: bus powered */
	0x32,                                           /* bMaxPower: 100 mA */

	/* ===== Interface Association Descriptor (8 bytes) [UVC1.5 3.6] ===== */
	0x08,                                           /* bLength */
	0x0B,                                           /* bDescriptorType: IAD */
	0x00,                                           /* bFirstInterface: VC (0) */
	0x02,                                           /* bInterfaceCount: VC + VS */
	0x0E,                                           /* bFunctionClass: CC_VIDEO */
	0x03,                                           /* bFunctionSubClass: SC_VIDEO_INTERFACE_COLLECTION */
	0x00,                                           /* bFunctionProtocol */
	0x00,                                           /* iFunction */

	/* ===== VideoControl Interface, Alt 0 (9 bytes) ===== */
	0x09,                                           /* bLength */
	USB_DESC_TYPE_INTERFACE,                        /* bDescriptorType: INTERFACE */
	0x00,                                           /* bInterfaceNumber: 0 */
	0x00,                                           /* bAlternateSetting */
	0x01,                                           /* bNumEndpoints: 1 (status interrupt) */
	0x0E,                                           /* bInterfaceClass: CC_VIDEO */
	0x01,                                           /* bInterfaceSubClass: SC_VIDEOCONTROL */
	0x00,                                           /* bInterfaceProtocol */
	0x00,                                           /* iInterface */

	/* ===== Class-specific VC Interface Header (13 bytes) [UVC1.5 Tbl 3-3] ===== */
	0x0D,                                           /* bLength */
	0x24,                                           /* bDescriptorType: CS_INTERFACE */
	0x01,                                           /* bDescriptorSubtype: VC_HEADER */
	0x10, 0x01,                                     /* bcdUVC: 1.10 */
	USB_LOW_BYTE(52 + USBD_UVC_XU_DESC_LEN),        /* wTotalLength (LSB): VC class descriptors, 52 (+28 if XU) */
	USB_HIGH_BYTE(52 + USBD_UVC_XU_DESC_LEN),       /* wTotalLength (MSB) */
	0x00, 0x6C, 0xDC, 0x02,                         /* dwClockFrequency: 0x02DC6C00 = 48 MHz */
	0x01,                                           /* bInCollection: 1 streaming interface */
	0x01,                                           /* baInterfaceNr(1): VS = interface 1 */

	/* ===== Camera Input Terminal (18 bytes) [UVC1.5 Tbl 3-4] ===== */
	0x12,                                           /* bLength */
	0x24,                                           /* bDescriptorType: CS_INTERFACE */
	0x02,                                           /* bDescriptorSubtype: VC_INPUT_TERMINAL */
	0x01,                                           /* bTerminalID: 1 */
	0x01, 0x02,                                     /* wTerminalType: 0x0201 ITT_CAMERA */
	0x00,                                           /* bAssocTerminal */
	0x00,                                           /* iTerminal */
	0x00, 0x00,                                     /* wObjectiveFocalLengthMin */
	0x00, 0x00,                                     /* wObjectiveFocalLengthMax */
	0x00, 0x00,                                     /* wOcularFocalLength */
	0x03,                                           /* bControlSize */
	0x00, 0x00, 0x00,                               /* bmControls: none implemented by firmware */

	/* ===== Processing Unit (12 bytes, UVC1.1) [UVC1.5 Tbl 3-8] ===== */
	0x0C,                                           /* bLength (10 + bControlSize) */
	0x24,                                           /* bDescriptorType: CS_INTERFACE */
	0x05,                                           /* bDescriptorSubtype: VC_PROCESSING_UNIT */
	0x02,                                           /* bUnitID: 2 */
	0x01,                                           /* bSourceID: 1 (Camera Terminal) */
	0x00, 0x40,                                     /* wMaxMultiplier: 0x4000 */
	0x02,                                           /* bControlSize */
	0x00, 0x00,                                     /* bmControls: none implemented by firmware */
	0x00,                                           /* iProcessing */
	0x00,                                           /* bmVideoStandards (UVC1.1): none */

#if USBD_UVC_USE_EXTENSION_UNIT
	/* ===== Extension Unit (28 bytes) [UVC1.5 Tbl 3-10] =====
	 * WARNING: disabled by default. Before enabling, make bNumControls / bmControls
	 * consistent and back every declared control with a real GET/SET handler in
	 * usbd_uvc_event.c; otherwise a strict host (Windows) fails enumeration.
	 * (The historical value bNumControls=24 with only 4 bmControls bits set is
	 * inconsistent and was rejected by CATC.) */
	0x1C,                                           /* bLength (24 + p + n) */
	0x24,                                           /* bDescriptorType: CS_INTERFACE */
	0x06,                                           /* bDescriptorSubtype: VC_EXTENSION_UNIT */
	0x03,                                           /* bUnitID: 3 */
	0xCE, 0xB9, 0x55, 0x2B, 0xB0, 0x14, 0x49, 0xA8, /* guidExtensionCode (16) */
	0xB2, 0x2E, 0xF6, 0xB0, 0xA0, 0x12, 0x82, 0x74,
	0x18,                                           /* bNumControls */
	0x01,                                           /* bNrInPins: 1 */
	0x02,                                           /* baSourceID(1): 2 (Processing Unit) */
	0x03,                                           /* bControlSize */
	0x00, 0x1E, 0x00,                               /* bmControls */
	0x00,                                           /* iExtension */
#endif

	/* ===== Output Terminal (9 bytes) [UVC1.5 Tbl 3-5] ===== */
	0x09,                                           /* bLength */
	0x24,                                           /* bDescriptorType: CS_INTERFACE */
	0x03,                                           /* bDescriptorSubtype: VC_OUTPUT_TERMINAL */
	0x04,                                           /* bTerminalID: 4 */
	0x01, 0x01,                                     /* wTerminalType: 0x0101 TT_STREAMING */
	0x00,                                           /* bAssocTerminal */
	USBD_UVC_OT_SOURCE_ID,                          /* bSourceID: PU(2) or XU(3) per USBD_UVC_USE_EXTENSION_UNIT */
	0x00,                                           /* iTerminal */

	/* ===== VC Status Interrupt Endpoint (7 bytes) ===== */
	0x07,                                           /* bLength */
	USB_DESC_TYPE_ENDPOINT,                         /* bDescriptorType: ENDPOINT */
	0x81,                                           /* bEndpointAddress: EP1 IN */
	0x03,                                           /* bmAttributes: Interrupt */
	0x40, 0x00,                                     /* wMaxPacketSize: 64 */
	0x08,                                           /* bInterval */

	/* ===== Class-specific VC Interrupt Endpoint (5 bytes) ===== */
	0x05,                                           /* bLength */
	0x25,                                           /* bDescriptorType: CS_ENDPOINT */
	0x03,                                           /* bDescriptorSubtype: EP_INTERRUPT */
	0x40, 0x00,                                     /* wMaxTransferSize: 64 */

	/* ===== VideoStreaming Interface, Alt 0 (zero-bandwidth, 9 bytes) ===== */
	0x09,                                           /* bLength */
	USB_DESC_TYPE_INTERFACE,                        /* bDescriptorType: INTERFACE */
	0x01,                                           /* bInterfaceNumber: 1 */
	0x00,                                           /* bAlternateSetting: 0 */
	0x00,                                           /* bNumEndpoints: 0 */
	0x0E,                                           /* bInterfaceClass: CC_VIDEO */
	0x02,                                           /* bInterfaceSubClass: SC_VIDEOSTREAMING */
	0x00,                                           /* bInterfaceProtocol */
	0x00,                                           /* iInterface */

	/* ===== Class-specific VS Input Header (18 bytes) [UVC1.5 Tbl 3-14] ===== */
	0x12,                                           /* bLength: 13 + bControlSize * bNumFormats = 13 + 1*5 */
	0x24,                                           /* bDescriptorType: CS_INTERFACE */
	0x01,                                           /* bDescriptorSubtype: VS_INPUT_HEADER */
	0x05,                                           /* bNumFormats: 5 (NV16/NV12/MJPEG/H264/HEVC) */
	0x4F, 0x01,                                     /* wTotalLength: 0x014F = 335 (VS class descriptors) */
	USBD_UVC_ISO_IN_EP,                             /* bEndpointAddress */
	0x00,                                           /* bmInfo */
	0x04,                                           /* bTerminalLink: Output Terminal (id 4) */
	0x00,                                           /* bStillCaptureMethod: none */
	0x00,                                           /* bTriggerSupport */
	0x00,                                           /* bTriggerUsage */
	0x01,                                           /* bControlSize */
	0x00,                                           /* bmaControls(1): NV16 */
	0x00,                                           /* bmaControls(2): NV12 */
	0x00,                                           /* bmaControls(3): MJPEG */
	0x00,                                           /* bmaControls(4): H.264 */
	0x00,                                           /* bmaControls(5): HEVC */

	/* Format 1: NV16 Uncompressed (27 bytes) - VS_FORMAT_UNCOMPRESSED */
	0x1B,                                           /* bLength = 27 */
	0x24,                                           /* bDescriptorType = CS_INTERFACE */
	0x04,                                           /* bDescriptorSubtype = VS_FORMAT_UNCOMPRESSED */
	0x01,                                           /* bFormatIndex = 1 */
	0x01,                                           /* bNumFrameDescriptors = 1 */
	0x4E, 0x56, 0x31, 0x36,                         /* guidFormat: NV16 FOURCC bytes [0..3] */
	0x00, 0x00, 0x10, 0x00,                         /* guidFormat: standard video GUID [4..7] */
	0x80, 0x00, 0x00, 0xAA,                         /* guidFormat: [8..11] */
	0x00, 0x38, 0x9B, 0x71,                         /* guidFormat: [12..15] */
	0x10,                                           /* bBitsPerPixel = 16 */
	0x01,                                           /* bDefaultFrameIndex = 1 */
	0x00,                                           /* bAspectRatioX = 0 */
	0x00,                                           /* bAspectRatioY = 0 */
	0x00,                                           /* bmInterlaceFlags = 0 */
	0x00,                                           /* bCopyProtect = 0 */

	/* Frame Descriptor - NV16 (38 bytes) - VS_FRAME_UNCOMPRESSED, 3 discrete intervals */
	0x26,                                           /* bLength = 38 */
	0x24,                                           /* bDescriptorType = CS_INTERFACE */
	0x05,                                           /* bDescriptorSubtype = VS_FRAME_UNCOMPRESSED */
	0x01,                                           /* bFrameIndex = 1 */
	0x00,                                           /* bmCapabilities = 0 (no still image) */
	USBD_UVC_WBVAL(USBD_TUNING_W),                 /* wWidth */
	USBD_UVC_WBVAL(USBD_TUNING_H),                 /* wHeight */
	0x00, 0x80, 0xF4, 0x03,                         /* dwMinBitRate = 66,355,200 bps */
	0x00, 0xC0, 0xA9, 0x1D,                         /* dwMaxBitRate = 498,073,600 bps */
	0x00, 0x48, 0x3F, 0x00,                         /* dwMaxVideoFrameBufferSize = 4,147,200 B */
	0x9A, 0x5B, 0x06, 0x00,                         /* dwDefaultFrameInterval = 416666 x 100ns (24fps) */
	0x03,                                           /* bFrameIntervalType = 3 (discrete) */
	0x9A, 0x5B, 0x06, 0x00,                         /* dwFrameInterval[0] = 416666 x 100ns (24fps) */
	0x2A, 0x2C, 0x0A, 0x00,                         /* dwFrameInterval[1] = 666666 x 100ns (15fps) */
	0x40, 0x42, 0x0F, 0x00,                         /* dwFrameInterval[2] = 1000000 x 100ns (10fps) */

	/* Format 2: NV12 Uncompressed (27 bytes) - VS_FORMAT_UNCOMPRESSED */
	0x1B,                                           /* bLength = 27 */
	0x24,                                           /* bDescriptorType = CS_INTERFACE */
	0x04,                                           /* bDescriptorSubtype = VS_FORMAT_UNCOMPRESSED */
	0x02,                                           /* bFormatIndex = 2 */
	0x01,                                           /* bNumFrameDescriptors = 1 */
	0x4E, 0x56, 0x31, 0x32,                         /* guidFormat: NV12 FOURCC bytes [0..3] */
	0x00, 0x00, 0x10, 0x00,                         /* guidFormat: standard video GUID [4..7] */
	0x80, 0x00, 0x00, 0xAA,                         /* guidFormat: [8..11] */
	0x00, 0x38, 0x9B, 0x71,                         /* guidFormat: [12..15] */
	0x0C,                                           /* bBitsPerPixel = 12 */
	0x01,                                           /* bDefaultFrameIndex = 1 */
	0x00,                                           /* bAspectRatioX = 0 */
	0x00,                                           /* bAspectRatioY = 0 */
	0x00,                                           /* bmInterlaceFlags = 0 */
	0x00,                                           /* bCopyProtect = 0 */

	/* Frame Descriptor - NV12 (38 bytes) - VS_FRAME_UNCOMPRESSED, 3 discrete intervals */
	0x26,                                           /* bLength = 38 */
	0x24,                                           /* bDescriptorType = CS_INTERFACE */
	0x05,                                           /* bDescriptorSubtype = VS_FRAME_UNCOMPRESSED */
	0x01,                                           /* bFrameIndex = 1 */
	0x00,                                           /* bmCapabilities = 0 (no still image) */
	USBD_UVC_WBVAL(USBD_UVC_FRAME_WIDTH),          /* wWidth */
	USBD_UVC_WBVAL(USBD_UVC_FRAME_HEIGHT),         /* wHeight */
	0x00, 0xEC, 0x5E, 0x00,                         /* dwMinBitRate = 6,217,728 bps */
	0x00, 0xEA, 0xC7, 0x02,                         /* dwMaxBitRate = 46,966,272 bps */
	0x00, 0x76, 0x2F, 0x00,                         /* dwMaxVideoFrameBufferSize = 3,110,400 B */
	0x9A, 0x5B, 0x06, 0x00,                         /* dwDefaultFrameInterval = 416666 x 100ns (24fps) */
	0x03,                                           /* bFrameIntervalType = 3 (discrete) */
	0x9A, 0x5B, 0x06, 0x00,                         /* dwFrameInterval[0] = 416666 x 100ns (24fps) */
	0x2A, 0x2C, 0x0A, 0x00,                         /* dwFrameInterval[1] = 666666 x 100ns (15fps) */
	0x40, 0x42, 0x0F, 0x00,                         /* dwFrameInterval[2] = 1000000 x 100ns (10fps) */

	/* Format 3: MJPEG (11 bytes) - VS_FORMAT_MJPEG */
	0x0B,                                           /* bLength = 11 */
	0x24,                                           /* bDescriptorType = CS_INTERFACE */
	0x06,                                           /* bDescriptorSubtype = VS_FORMAT_MJPEG */
	0x03,                                           /* bFormatIndex = 3 */
	0x01,                                           /* bNumFrameDescriptors = 1 */
	0x01,                                           /* bmFlags = 1 (FixedSizeSamples) */
	0x01,                                           /* bDefaultFrameIndex = 1 */
	0x00,                                           /* bAspectRatioX = 0 */
	0x00,                                           /* bAspectRatioY = 0 */
	0x00,                                           /* bmInterlaceFlags = 0 */
	0x00,                                           /* bCopyProtect = 0 */

	/* Frame Descriptor - MJPEG (38 bytes) - VS_FRAME_MJPEG, 3 discrete intervals */
	0x26,                                           /* bLength = 38 */
	0x24,                                           /* bDescriptorType = CS_INTERFACE */
	0x07,                                           /* bDescriptorSubtype = VS_FRAME_MJPEG */
	0x01,                                           /* bFrameIndex = 1 */
	0x00,                                           /* bmCapabilities = 0 (no still image) */
	USBD_UVC_WBVAL(USBD_UVC_JPEG_FRAME_WIDTH),     /* wWidth */
	USBD_UVC_WBVAL(USBD_UVC_JPEG_FRAME_HEIGHT),    /* wHeight */
	0x00, 0xC4, 0x1C, 0x01,                         /* dwMinBitRate = 18,603,008 bps */
	0x00, 0x4C, 0x56, 0x03,                         /* dwMaxBitRate = 56,278,016 bps */
	0x00, 0x08, 0x07, 0x00,                         /* dwMaxVideoFrameBufferSize = 460,800 B */
	0x9A, 0x5B, 0x06, 0x00,                         /* dwDefaultFrameInterval = 416666 x 100ns (24fps) */
	0x03,                                           /* bFrameIntervalType = 3 (discrete) */
	0x9A, 0x5B, 0x06, 0x00,                         /* dwFrameInterval[0] = 416666 x 100ns (24fps) */
	0x2A, 0x2C, 0x0A, 0x00,                         /* dwFrameInterval[1] = 666666 x 100ns (15fps) */
	0x40, 0x42, 0x0F, 0x00,                         /* dwFrameInterval[2] = 1000000 x 100ns (10fps) */

	/* Format 4: H.264 (28 bytes) - VS_FORMAT_FRAME_BASED */
	0x1C,                                           /* bLength = 28 */
	0x24,                                           /* bDescriptorType = CS_INTERFACE */
	0x10,                                           /* bDescriptorSubtype = VS_FORMAT_FRAME_BASED */
	0x04,                                           /* bFormatIndex = 4 */
	0x01,                                           /* bNumFrameDescriptors = 1 */
	0x48, 0x32, 0x36, 0x34,                         /* guidFormat: H264 FOURCC bytes [0..3] */
	0x00, 0x00, 0x10, 0x00,                         /* guidFormat: standard video GUID [4..7] */
	0x80, 0x00, 0x00, 0xAA,                         /* guidFormat: [8..11] */
	0x00, 0x38, 0x9B, 0x71,                         /* guidFormat: [12..15] */
	0x0C,                                           /* bBitsPerPixel = 12 */
	0x01,                                           /* bDefaultFrameIndex = 1 */
	0x00,                                           /* bAspectRatioX = 0 */
	0x00,                                           /* bAspectRatioY = 0 */
	0x00,                                           /* bmInterlaceFlags = 0 */
	0x00,                                           /* bCopyProtect = 0 */
	0x01,                                           /* bVariableSize = 1 (variable-size frames) */

	/* Frame Descriptor - H.264 (38 bytes) - VS_FRAME_FRAME_BASED, 3 discrete intervals */
	0x26,                                           /* bLength = 38 */
	0x24,                                           /* bDescriptorType = CS_INTERFACE */
	0x11,                                           /* bDescriptorSubtype = VS_FRAME_FRAME_BASED */
	0x01,                                           /* bFrameIndex = 1 */
	0x00,                                           /* bmCapabilities = 0 (no still image) */
	USBD_UVC_WBVAL(USBD_UVC_FRAME_WIDTH),          /* wWidth */
	USBD_UVC_WBVAL(USBD_UVC_FRAME_HEIGHT),         /* wHeight */
	0x00, 0xC4, 0x1C, 0x01,                         /* dwMinBitRate = 18,603,008 bps */
	0x00, 0x4C, 0x56, 0x03,                         /* dwMaxBitRate = 56,278,016 bps */
	0x15, 0x16, 0x05, 0x00,                         /* dwDefaultFrameInterval = 333333 x 100ns (30fps) */
	0x03,                                           /* bFrameIntervalType = 3 (discrete) */
	0x00, 0x00, 0x00, 0x00,                         /* dwBytesPerLine = 0 (variable compressed size) */
	0x15, 0x16, 0x05, 0x00,                         /* dwFrameInterval[0] = 333333 x 100ns (30fps) */
	0x2A, 0x2C, 0x0A, 0x00,                         /* dwFrameInterval[1] = 666666 x 100ns (15fps) */
	0x40, 0x42, 0x0F, 0x00,                         /* dwFrameInterval[2] = 1000000 x 100ns (10fps) */

	/* Format 5: HEVC (28 bytes) - VS_FORMAT_FRAME_BASED */
	0x1C,                                           /* bLength = 28 */
	0x24,                                           /* bDescriptorType = CS_INTERFACE */
	0x10,                                           /* bDescriptorSubtype = VS_FORMAT_FRAME_BASED */
	0x05,                                           /* bFormatIndex = 5 */
	0x01,                                           /* bNumFrameDescriptors = 1 */
	0x48, 0x45, 0x56, 0x43,                         /* guidFormat: HEVC FOURCC bytes [0..3] */
	0x00, 0x00, 0x10, 0x00,                         /* guidFormat: standard video GUID [4..7] */
	0x80, 0x00, 0x00, 0xAA,                         /* guidFormat: [8..11] */
	0x00, 0x38, 0x9B, 0x71,                         /* guidFormat: [12..15] */
	0x0C,                                           /* bBitsPerPixel = 12 */
	0x01,                                           /* bDefaultFrameIndex = 1 */
	0x00,                                           /* bAspectRatioX = 0 */
	0x00,                                           /* bAspectRatioY = 0 */
	0x00,                                           /* bmInterlaceFlags = 0 */
	0x00,                                           /* bCopyProtect = 0 */
	0x01,                                           /* bVariableSize = 1 (variable-size frames) */

	/* Frame Descriptor - HEVC (38 bytes) - VS_FRAME_FRAME_BASED, 3 discrete intervals */
	0x26,                                           /* bLength = 38 */
	0x24,                                           /* bDescriptorType = CS_INTERFACE */
	0x11,                                           /* bDescriptorSubtype = VS_FRAME_FRAME_BASED */
	0x01,                                           /* bFrameIndex = 1 */
	0x00,                                           /* bmCapabilities = 0 (no still image) */
	USBD_UVC_WBVAL(USBD_UVC_FRAME_WIDTH),          /* wWidth */
	USBD_UVC_WBVAL(USBD_UVC_FRAME_HEIGHT),         /* wHeight */
	0x00, 0xC4, 0x1C, 0x01,                         /* dwMinBitRate = 18,603,008 bps */
	0x00, 0x4C, 0x56, 0x03,                         /* dwMaxBitRate = 56,278,016 bps */
	0x9A, 0x5B, 0x06, 0x00,                         /* dwDefaultFrameInterval = 416666 x 100ns (24fps) */
	0x03,                                           /* bFrameIntervalType = 3 (discrete) */
	0x00, 0x00, 0x00, 0x00,                         /* dwBytesPerLine = 0 (variable compressed size) */
	0x9A, 0x5B, 0x06, 0x00,                         /* dwFrameInterval[0] = 416666 x 100ns (24fps) */
	0x2A, 0x2C, 0x0A, 0x00,                         /* dwFrameInterval[1] = 666666 x 100ns (15fps) */
	0x40, 0x42, 0x0F, 0x00,                         /* dwFrameInterval[2] = 1000000 x 100ns (10fps) */

	/* Color Matching Descriptor (6 bytes) - VS_COLORFORMAT */
	0x06,                                           /* bLength = 6 */
	0x24,                                           /* bDescriptorType = CS_INTERFACE */
	0x0D,                                           /* bDescriptorSubtype = VS_COLORFORMAT */
	0x01,                                           /* bColorPrimaries = 1 (BT.709) */
	0x01,                                           /* bTransferCharacteristics = 1 (BT.709) */
	0x04,                                           /* bMatrixCoefficients = 4 (SMPTE 170M) */

	/* Video Streaming Interface Alt 1 (9 bytes) - active streaming alternate setting */
	0x09,                                           /* bLength = 9 */
	0x04,                                           /* bDescriptorType = INTERFACE */
	0x01,                                           /* bInterfaceNumber = 1 (VS) */
	0x01,                                           /* bAlternateSetting = 1 (active) */
	0x01,                                           /* bNumEndpoints = 1 */
	0x0E,                                           /* bInterfaceClass = CC_VIDEO */
	0x02,                                           /* bInterfaceSubClass = SC_VIDEOSTREAMING */
	0x00,                                           /* bInterfaceProtocol = PC_PROTOCOL_UNDEFINED */
	0x00,                                           /* iInterface = 0 (no string) */

	/* Isochronous Endpoint (7 bytes) - ISOC IN, HS high-bandwidth 3x1024 */
	0x07,                                           /* bLength = 7 */
	0x05,                                           /* bDescriptorType = ENDPOINT */
	USBD_UVC_ISO_IN_EP,                             /* bEndpointAddress */
	0x05,                                           /* bmAttributes = ISOC | ASYNC */
	USBD_UVC_WBVAL(USBD_UVC_ISOC_EP_MPKT),         /* wMaxPacketSize: bits[12:11]=extra_trans, bits[10:0]=MPS */
	0x01                                            /* bInterval = 1 (every microframe) */
};

int usbd_uvc_descriptors_size = sizeof(usbd_uvc_descriptors);

usbd_uvc_streaming_control_t usbd_uvc_probe = {
	.bmHint = 0,
	.bFormatIndex = 4,
	.bFrameIndex = 1,
	.dwFrameInterval = 333333,
	.wKeyFrameRate = 0,
	.wPFrameRate = 0,
	.wCompQuality = 0,
	.wCompWindowSize = 0,
	.wDelay = 0,
	.dwMaxVideoFrameSize = USBD_TUNING_W * USBD_TUNING_H,
	.dwMaxPayloadTransferSize = 0,
	.dwClockFrequency = USBD_UVC_CLOCK_FREQUENCY,
	.bmFramingInfo = 0,
	.bPreferedVersion = 0,
	.bMinVersion = 0,
	.bMaxVersion = 0,
};

usbd_uvc_streaming_control_t usbd_uvc_commit = {
	.bmHint = 0,
	.bFormatIndex = 4,
	.bFrameIndex = 1,
	.dwFrameInterval = 333333,
	.wKeyFrameRate = 0,
	.wPFrameRate = 0,
	.wCompQuality = 0,
	.wCompWindowSize = 0,
	.wDelay = 0,
	.dwMaxVideoFrameSize = USBD_TUNING_W * USBD_TUNING_H,
	.dwMaxPayloadTransferSize = 0,
	.dwClockFrequency = USBD_UVC_CLOCK_FREQUENCY,
	.bmFramingInfo = 0,
	.bPreferedVersion = 0,
	.bMinVersion = 0,
	.bMaxVersion = 0,
};

usbd_uvc_frame_info_t uvc_frames_yuy2[] = {
	{ USBD_TUNING_W, USBD_TUNING_H, { USBD_UVC_FPS(30), USBD_UVC_FPS(15), USBD_UVC_FPS(10), 0 },},
	{ 0, 0, { 0, },  },
};

usbd_uvc_frame_info_t uvc_frames_nv12[] = {
	{ USBD_UVC_FRAME_WIDTH, USBD_UVC_FRAME_HEIGHT, { USBD_UVC_FPS(30), USBD_UVC_FPS(15), USBD_UVC_FPS(10), 0 },},
	{ 0, 0, { 0, }, },
};

usbd_uvc_frame_info_t uvc_frames_mjpeg[] = {
	{ USBD_UVC_JPEG_FRAME_WIDTH, USBD_UVC_JPEG_FRAME_HEIGHT, { USBD_UVC_FPS(30), USBD_UVC_FPS(15), USBD_UVC_FPS(10), 0 },},
	{ 0, 0, { 0, }, },
};

usbd_uvc_frame_info_t uvc_frames_h264[] = {
	{ USBD_UVC_FRAME_WIDTH, USBD_UVC_FRAME_HEIGHT, { USBD_UVC_FPS(30), USBD_UVC_FPS(15), USBD_UVC_FPS(10), 0 },},
	{ 0, 0, { 0, }, },
};

usbd_uvc_frame_info_t uvc_frames_h265[] = {
	{ USBD_UVC_FRAME_WIDTH, USBD_UVC_FRAME_HEIGHT, { USBD_UVC_FPS(30), USBD_UVC_FPS(15), USBD_UVC_FPS(10), 0 },},
	{ 0, 0, { 0, }, },
};

usbd_uvc_format_info_t uvcd_formats[] = {
	{ USBD_UVC_FORMAT_TYPE_YUY2, uvc_frames_yuy2 },
	{ USBD_UVC_FORMAT_TYPE_NV12, uvc_frames_nv12 },
	{ USBD_UVC_FORMAT_TYPE_MJPEG, uvc_frames_mjpeg},
	{ USBD_UVC_FORMAT_TYPE_H264, uvc_frames_h264 },
	{ USBD_UVC_FORMAT_TYPE_H265, uvc_frames_h265 },
};
const u32 uvcd_nformats = sizeof(uvcd_formats) / sizeof(uvcd_formats[0]);
#endif

/* ---------------------------------------------------------------------------
 * Configuration descriptor generator for usbd_uvc_init_multi()
 *
 * One UVC function per instance, instance i on interfaces 2i (VC) and 2i+1 (VS):
 *   IAD, VC interface, VC header, Camera IT, PU, [XU], OT,
 *   VS interface alt 0, VS input header, format + frame descriptors, color matching,
 *   VS interface alt 1 with the instance's ISOC IN endpoint.
 * The VC status interrupt endpoint of the static descriptor is left out (optional in UVC 1.5,
 * never used by this driver), so every IN endpoint stays available for video.
 * ------------------------------------------------------------------------- */

#define USBD_UVC_DESC_IAD_LEN            8U
#define USBD_UVC_DESC_ITF_LEN            9U
#define USBD_UVC_DESC_VC_HDR_LEN         13U
#define USBD_UVC_DESC_IT_LEN             18U
#define USBD_UVC_DESC_PU_LEN             12U
#define USBD_UVC_DESC_OT_LEN             9U
#define USBD_UVC_DESC_EP_LEN             7U
#define USBD_UVC_DESC_COLOR_LEN          6U
#define USBD_UVC_DESC_VS_HDR_LEN         13U    /* + bControlSize * bNumFormats */
#define USBD_UVC_DESC_FMT_UNCOMP_LEN     27U
#define USBD_UVC_DESC_FMT_MJPEG_LEN      11U
#define USBD_UVC_DESC_FMT_FRAME_LEN      28U
#define USBD_UVC_DESC_FRAME_LEN          26U    /* + 4 * bFrameIntervalType */

#define USBD_UVC_DESC_CS_INTERFACE       0x24U
#define USBD_UVC_DESC_VC_PART_LEN        (USBD_UVC_DESC_VC_HDR_LEN + USBD_UVC_DESC_IT_LEN + USBD_UVC_DESC_PU_LEN + \
										  USBD_UVC_XU_DESC_LEN + USBD_UVC_DESC_OT_LEN)

/* VS subtypes [UVC1.5 Tbl A-6] */
#define USBD_UVC_VS_SUB_INPUT_HEADER     0x01U
#define USBD_UVC_VS_SUB_FMT_UNCOMP       0x04U
#define USBD_UVC_VS_SUB_FRM_UNCOMP       0x05U
#define USBD_UVC_VS_SUB_FMT_MJPEG        0x06U
#define USBD_UVC_VS_SUB_FRM_MJPEG        0x07U
#define USBD_UVC_VS_SUB_COLORFORMAT      0x0DU
#define USBD_UVC_VS_SUB_FMT_FRAME        0x10U
#define USBD_UVC_VS_SUB_FRM_FRAME        0x11U

static u16 usbd_uvc_put16(u8 *p, u32 v)
{
	p[0] = (u8)(v & 0xFFU);
	p[1] = (u8)((v >> 8) & 0xFFU);
	return 2U;
}

static u16 usbd_uvc_put32(u8 *p, u32 v)
{
	p[0] = (u8)(v & 0xFFU);
	p[1] = (u8)((v >> 8) & 0xFFU);
	p[2] = (u8)((v >> 16) & 0xFFU);
	p[3] = (u8)((v >> 24) & 0xFFU);
	return 4U;
}

/* Descriptor subtypes, FOURCC (NULL for MJPEG) and bBitsPerPixel of a format. 0: unknown format */
static u8 usbd_uvc_fmt_desc_info(u32 fcc, u8 *fmt_sub, u8 *frm_sub, const u8 **fourcc)
{
	/* USBD_UVC_FORMAT_TYPE_YUY2 carries the ISP's YUV422 semi-planar output, advertised as NV16
	   exactly like the static descriptor */
	static const u8 cc_nv16[4] = {'N', 'V', '1', '6'};
	static const u8 cc_nv12[4] = {'N', 'V', '1', '2'};
	static const u8 cc_h264[4] = {'H', '2', '6', '4'};
	static const u8 cc_hevc[4] = {'H', 'E', 'V', 'C'};

	switch (fcc) {
	case USBD_UVC_FORMAT_TYPE_YUY2:
		*fmt_sub = USBD_UVC_VS_SUB_FMT_UNCOMP;
		*frm_sub = USBD_UVC_VS_SUB_FRM_UNCOMP;
		*fourcc = cc_nv16;
		return 16U;
	case USBD_UVC_FORMAT_TYPE_NV12:
		*fmt_sub = USBD_UVC_VS_SUB_FMT_UNCOMP;
		*frm_sub = USBD_UVC_VS_SUB_FRM_UNCOMP;
		*fourcc = cc_nv12;
		return 12U;
	case USBD_UVC_FORMAT_TYPE_MJPEG:
		*fmt_sub = USBD_UVC_VS_SUB_FMT_MJPEG;
		*frm_sub = USBD_UVC_VS_SUB_FRM_MJPEG;
		*fourcc = NULL;
		return 16U;
	case USBD_UVC_FORMAT_TYPE_H264:
		*fmt_sub = USBD_UVC_VS_SUB_FMT_FRAME;
		*frm_sub = USBD_UVC_VS_SUB_FRM_FRAME;
		*fourcc = cc_h264;
		return 12U;
	case USBD_UVC_FORMAT_TYPE_H265:
		*fmt_sub = USBD_UVC_VS_SUB_FMT_FRAME;
		*frm_sub = USBD_UVC_VS_SUB_FRM_FRAME;
		*fourcc = cc_hevc;
		return 12U;
	default:
		return 0U;
	}
}

/* Largest frame size of a format, same rule as probe/commit (usbd_uvc_event.c) */
static u32 usbd_uvc_frame_bytes(u32 fcc, u32 w, u32 h)
{
	switch (fcc) {
	case USBD_UVC_FORMAT_TYPE_YUY2:
		return w * h * USBD_UVC_YUY2_BPP;
	case USBD_UVC_FORMAT_TYPE_NV12:
		return (w * h * USBD_UVC_NV12_SIZE_NUM) / USBD_UVC_NV12_SIZE_DEN;
	default:
		return w * h;
	}
}

/* Video Control part of one function: IAD, VC interface, header, IT, PU, [XU], OT */
static u16 usbd_uvc_build_vc(u8 *buf, u8 itf)
{
	u16 pos = 0U;

	/* IAD [UVC1.5 3.6] */
	buf[pos++] = USBD_UVC_DESC_IAD_LEN;
	buf[pos++] = USB_DESC_TYPE_IAD;
	buf[pos++] = itf;                               /* bFirstInterface */
	buf[pos++] = 0x02U;                             /* bInterfaceCount: VC + VS */
	buf[pos++] = 0x0EU;                             /* bFunctionClass: CC_VIDEO */
	buf[pos++] = 0x03U;                             /* bFunctionSubClass: SC_VIDEO_INTERFACE_COLLECTION */
	buf[pos++] = 0x00U;
	buf[pos++] = 0x00U;                             /* iFunction */

	/* VideoControl interface */
	buf[pos++] = USBD_UVC_DESC_ITF_LEN;
	buf[pos++] = USB_DESC_TYPE_INTERFACE;
	buf[pos++] = itf;
	buf[pos++] = 0x00U;                             /* bAlternateSetting */
	buf[pos++] = 0x00U;                             /* bNumEndpoints: no status interrupt EP */
	buf[pos++] = 0x0EU;                             /* CC_VIDEO */
	buf[pos++] = 0x01U;                             /* SC_VIDEOCONTROL */
	buf[pos++] = 0x00U;
	buf[pos++] = 0x00U;

	/* VC header [UVC1.5 Tbl 3-3] */
	buf[pos++] = USBD_UVC_DESC_VC_HDR_LEN;
	buf[pos++] = USBD_UVC_DESC_CS_INTERFACE;
	buf[pos++] = 0x01U;                             /* VC_HEADER */
	pos += usbd_uvc_put16(&buf[pos], 0x0110U);      /* bcdUVC 1.10 */
	pos += usbd_uvc_put16(&buf[pos], USBD_UVC_DESC_VC_PART_LEN);
	pos += usbd_uvc_put32(&buf[pos], USBD_UVC_CLOCK_FREQUENCY);
	buf[pos++] = 0x01U;                             /* bInCollection */
	buf[pos++] = (u8)(itf + 1U);                    /* baInterfaceNr(1): VS */

	/* Camera Input Terminal [UVC1.5 Tbl 3-4], id 1 */
	buf[pos++] = USBD_UVC_DESC_IT_LEN;
	buf[pos++] = USBD_UVC_DESC_CS_INTERFACE;
	buf[pos++] = 0x02U;                             /* VC_INPUT_TERMINAL */
	buf[pos++] = 0x01U;                             /* bTerminalID */
	pos += usbd_uvc_put16(&buf[pos], 0x0201U);      /* ITT_CAMERA */
	buf[pos++] = 0x00U;                             /* bAssocTerminal */
	buf[pos++] = 0x00U;                             /* iTerminal */
	pos += usbd_uvc_put16(&buf[pos], 0U);           /* wObjectiveFocalLengthMin */
	pos += usbd_uvc_put16(&buf[pos], 0U);           /* wObjectiveFocalLengthMax */
	pos += usbd_uvc_put16(&buf[pos], 0U);           /* wOcularFocalLength */
	buf[pos++] = 0x03U;                             /* bControlSize */
	buf[pos++] = 0x00U;                             /* bmControls: none */
	buf[pos++] = 0x00U;
	buf[pos++] = 0x00U;

	/* Processing Unit [UVC1.5 Tbl 3-8], id 2 */
	buf[pos++] = USBD_UVC_DESC_PU_LEN;
	buf[pos++] = USBD_UVC_DESC_CS_INTERFACE;
	buf[pos++] = 0x05U;                             /* VC_PROCESSING_UNIT */
	buf[pos++] = 0x02U;                             /* bUnitID */
	buf[pos++] = 0x01U;                             /* bSourceID: IT */
	pos += usbd_uvc_put16(&buf[pos], 0x4000U);      /* wMaxMultiplier */
	buf[pos++] = 0x02U;                             /* bControlSize */
	buf[pos++] = 0x00U;                             /* bmControls: none */
	buf[pos++] = 0x00U;
	buf[pos++] = 0x00U;                             /* iProcessing */
	buf[pos++] = 0x00U;                             /* bmVideoStandards */

#if USBD_UVC_USE_EXTENSION_UNIT
	{
		/* Extension Unit [UVC1.5 Tbl 3-10], id 3, same as the static descriptor */
		static const u8 xu[USBD_UVC_XU_DESC_LEN] = {
			USBD_UVC_XU_DESC_LEN, USBD_UVC_DESC_CS_INTERFACE, 0x06, 0x03,
			0xCE, 0xB9, 0x55, 0x2B, 0xB0, 0x14, 0x49, 0xA8,
			0xB2, 0x2E, 0xF6, 0xB0, 0xA0, 0x12, 0x82, 0x74,
			0x18, 0x01, 0x02, 0x03, 0x00, 0x1E, 0x00, 0x00,
		};
		usb_os_memcpy((void *)&buf[pos], (const void *)xu, USBD_UVC_XU_DESC_LEN);
		pos += USBD_UVC_XU_DESC_LEN;
	}
#endif

	/* Output Terminal [UVC1.5 Tbl 3-5], id 4 */
	buf[pos++] = USBD_UVC_DESC_OT_LEN;
	buf[pos++] = USBD_UVC_DESC_CS_INTERFACE;
	buf[pos++] = 0x03U;                             /* VC_OUTPUT_TERMINAL */
	buf[pos++] = 0x04U;                             /* bTerminalID */
	pos += usbd_uvc_put16(&buf[pos], 0x0101U);      /* TT_STREAMING */
	buf[pos++] = 0x00U;
	buf[pos++] = USBD_UVC_OT_SOURCE_ID;             /* PU(2) or XU(3) */
	buf[pos++] = 0x00U;

	return pos;
}

/* Video Streaming part of one function. Returns 0 if buf_len is too small or the table is invalid. */
static u16 usbd_uvc_build_vs(const usbd_uvc_dev_t *dev, u8 *buf, u16 buf_len)
{
	static const u8 guid_tail[12] = {0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71};
	const u8 itf = (u8)(dev->itf_base + 1U);
	const u32 xfer = dev->in_buf_size;
	u32 pos = 0U;
	u32 hdr;
	u32 f;

	if (((u32)USBD_UVC_DESC_ITF_LEN + USBD_UVC_DESC_VS_HDR_LEN + dev->nformats) > buf_len) {
		return 0U;
	}

	/* VS interface alt 0 (zero bandwidth) */
	buf[pos++] = USBD_UVC_DESC_ITF_LEN;
	buf[pos++] = USB_DESC_TYPE_INTERFACE;
	buf[pos++] = itf;
	buf[pos++] = 0x00U;                             /* bAlternateSetting */
	buf[pos++] = 0x00U;                             /* bNumEndpoints */
	buf[pos++] = 0x0EU;                             /* CC_VIDEO */
	buf[pos++] = 0x02U;                             /* SC_VIDEOSTREAMING */
	buf[pos++] = 0x00U;
	buf[pos++] = 0x00U;

	/* VS input header [UVC1.5 Tbl 3-14], wTotalLength patched below */
	hdr = pos;
	buf[pos++] = (u8)(USBD_UVC_DESC_VS_HDR_LEN + dev->nformats);
	buf[pos++] = USBD_UVC_DESC_CS_INTERFACE;
	buf[pos++] = USBD_UVC_VS_SUB_INPUT_HEADER;
	buf[pos++] = (u8)dev->nformats;                 /* bNumFormats */
	pos += 2U;                                      /* wTotalLength */
	buf[pos++] = dev->ep_isoc_in.info.addr;         /* bEndpointAddress */
	buf[pos++] = 0x00U;                             /* bmInfo */
	buf[pos++] = 0x04U;                             /* bTerminalLink: OT */
	buf[pos++] = 0x00U;                             /* bStillCaptureMethod */
	buf[pos++] = 0x00U;                             /* bTriggerSupport */
	buf[pos++] = 0x00U;                             /* bTriggerUsage */
	buf[pos++] = 0x01U;                             /* bControlSize */
	for (f = 0U; f < dev->nformats; f++) {
		buf[pos++] = 0x00U;                         /* bmaControls */
	}

	for (f = 0U; f < dev->nformats; f++) {
		const usbd_uvc_format_info_t *fmt = &dev->formats[f];
		const u8 *fourcc = NULL;
		u8 fmt_sub = 0U;
		u8 frm_sub = 0U;
		u8 bpp = usbd_uvc_fmt_desc_info(fmt->fcc, &fmt_sub, &frm_sub, &fourcc);
		u32 nframes = 0U;
		u32 fr;

		while ((fmt->frames != NULL) && (fmt->frames[nframes].width != 0U)) {
			nframes++;
		}
		if ((bpp == 0U) || (nframes == 0U) || ((pos + USBD_UVC_DESC_FMT_FRAME_LEN) > buf_len)) {
			return 0U;
		}

		/* Format descriptor */
		buf[pos + 1U] = USBD_UVC_DESC_CS_INTERFACE;
		buf[pos + 2U] = fmt_sub;
		buf[pos + 3U] = (u8)(f + 1U);              /* bFormatIndex */
		buf[pos + 4U] = (u8)nframes;               /* bNumFrameDescriptors */
		if (fourcc == NULL) {
			buf[pos] = USBD_UVC_DESC_FMT_MJPEG_LEN;
			buf[pos + 5U] = 0x01U;                 /* bmFlags: FixedSizeSamples */
			buf[pos + 6U] = 0x01U;                 /* bDefaultFrameIndex */
			buf[pos + 7U] = 0x00U;                 /* bAspectRatioX */
			buf[pos + 8U] = 0x00U;                 /* bAspectRatioY */
			buf[pos + 9U] = 0x00U;                 /* bmInterlaceFlags */
			buf[pos + 10U] = 0x00U;                /* bCopyProtect */
		} else {
			buf[pos] = (fmt_sub == USBD_UVC_VS_SUB_FMT_FRAME) ? USBD_UVC_DESC_FMT_FRAME_LEN : USBD_UVC_DESC_FMT_UNCOMP_LEN;
			usb_os_memcpy((void *)&buf[pos + 5U], (const void *)fourcc, 4U);
			usb_os_memcpy((void *)&buf[pos + 9U], (const void *)guid_tail, (u32)sizeof(guid_tail));
			buf[pos + 21U] = bpp;                  /* bBitsPerPixel */
			buf[pos + 22U] = 0x01U;                /* bDefaultFrameIndex */
			buf[pos + 23U] = 0x00U;                /* bAspectRatioX */
			buf[pos + 24U] = 0x00U;                /* bAspectRatioY */
			buf[pos + 25U] = 0x00U;                /* bmInterlaceFlags */
			buf[pos + 26U] = 0x00U;                /* bCopyProtect */
			if (fmt_sub == USBD_UVC_VS_SUB_FMT_FRAME) {
				buf[pos + 27U] = 0x01U;            /* bVariableSize */
			}
		}
		pos += buf[pos];

		/* Frame descriptors */
		for (fr = 0U; fr < nframes; fr++) {
			const usbd_uvc_frame_info_t *frame = &fmt->frames[fr];
			u32 frame_bytes = usbd_uvc_frame_bytes(fmt->fcc, frame->width, frame->height);
			u32 nint = 0U;
			u32 i;

			while ((nint < USBD_UVC_FRAME_INTERVAL_CNT) && (frame->intervals[nint] != 0U)) {
				nint++;
			}
			if ((nint == 0U) || ((pos + USBD_UVC_DESC_FRAME_LEN + (4U * nint)) > buf_len)) {
				return 0U;
			}
			buf[pos++] = (u8)(USBD_UVC_DESC_FRAME_LEN + (4U * nint));
			buf[pos++] = USBD_UVC_DESC_CS_INTERFACE;
			buf[pos++] = frm_sub;
			buf[pos++] = (u8)(fr + 1U);            /* bFrameIndex */
			buf[pos++] = 0x00U;                    /* bmCapabilities */
			pos += usbd_uvc_put16(&buf[pos], frame->width);
			pos += usbd_uvc_put16(&buf[pos], frame->height);
			/* dwMinBitRate / dwMaxBitRate at the slowest / fastest interval */
			pos += usbd_uvc_put32(&buf[pos], frame_bytes * 8U * (USBD_UVC_FRAME_INTERVAL_UNIT / frame->intervals[nint - 1U]));
			pos += usbd_uvc_put32(&buf[pos], frame_bytes * 8U * (USBD_UVC_FRAME_INTERVAL_UNIT / frame->intervals[0]));
			if (frm_sub == USBD_UVC_VS_SUB_FRM_FRAME) {
				pos += usbd_uvc_put32(&buf[pos], frame->intervals[0]);    /* dwDefaultFrameInterval */
				buf[pos++] = (u8)nint;                                    /* bFrameIntervalType */
				pos += usbd_uvc_put32(&buf[pos], 0U);                     /* dwBytesPerLine */
			} else {
				pos += usbd_uvc_put32(&buf[pos], frame_bytes);            /* dwMaxVideoFrameBufferSize */
				pos += usbd_uvc_put32(&buf[pos], frame->intervals[0]);    /* dwDefaultFrameInterval */
				buf[pos++] = (u8)nint;                                    /* bFrameIntervalType */
			}
			for (i = 0U; i < nint; i++) {
				pos += usbd_uvc_put32(&buf[pos], frame->intervals[i]);
			}
		}
	}

	if ((pos + USBD_UVC_DESC_COLOR_LEN + USBD_UVC_DESC_ITF_LEN + USBD_UVC_DESC_EP_LEN) > buf_len) {
		return 0U;
	}

	/* Color matching [UVC1.5 Tbl 3-29]: BT.709 primaries / transfer, SMPTE 170M matrix */
	buf[pos++] = USBD_UVC_DESC_COLOR_LEN;
	buf[pos++] = USBD_UVC_DESC_CS_INTERFACE;
	buf[pos++] = USBD_UVC_VS_SUB_COLORFORMAT;
	buf[pos++] = 0x01U;
	buf[pos++] = 0x01U;
	buf[pos++] = 0x04U;
	(void)usbd_uvc_put16(&buf[hdr + 4U], pos - hdr);

	/* VS interface alt 1 with the ISOC IN endpoint */
	buf[pos++] = USBD_UVC_DESC_ITF_LEN;
	buf[pos++] = USB_DESC_TYPE_INTERFACE;
	buf[pos++] = itf;
	buf[pos++] = 0x01U;                             /* bAlternateSetting */
	buf[pos++] = 0x01U;                             /* bNumEndpoints */
	buf[pos++] = 0x0EU;
	buf[pos++] = 0x02U;
	buf[pos++] = 0x00U;
	buf[pos++] = 0x00U;

	buf[pos++] = USBD_UVC_DESC_EP_LEN;
	buf[pos++] = USB_DESC_TYPE_ENDPOINT;
	buf[pos++] = dev->ep_isoc_in.info.addr;
	buf[pos++] = 0x05U;                             /* ISOC, asynchronous */
	/* wMaxPacketSize: bits[12:11] = additional transactions, bits[10:0] = 1024 */
	pos += usbd_uvc_put16(&buf[pos], (((xfer / USBD_UVC_ISOC_EP_MPS) - 1U) << 11U) | USBD_UVC_ISOC_EP_MPS);
	buf[pos++] = USBD_UVC_ISOC_EP_BINTERVAL;

	return (u16)pos;
}

u16 usbd_uvc_build_config_desc(const usbd_uvc_dev_t *devs, u8 num, u8 *buf, u16 buf_len)
{
	const u32 vc_len = USBD_UVC_DESC_IAD_LEN + USBD_UVC_DESC_ITF_LEN + USBD_UVC_DESC_VC_PART_LEN;
	u32 pos = USB_LEN_CFG_DESC;
	u16 len;
	u8 i;

	if ((devs == NULL) || (buf == NULL) || (num == 0U) || (buf_len < USB_LEN_CFG_DESC)) {
		return 0U;
	}

	for (i = 0U; i < num; i++) {
		if ((pos + vc_len) > buf_len) {
			return 0U;
		}
		pos += usbd_uvc_build_vc(&buf[pos], devs[i].itf_base);
		len = usbd_uvc_build_vs(&devs[i], &buf[pos], (u16)(buf_len - pos));
		if (len == 0U) {
			return 0U;
		}
		pos += len;
	}

	/* Configuration descriptor [USB2.0 9.6.3] */
	buf[0] = USB_LEN_CFG_DESC;
	buf[1] = USB_DESC_TYPE_CONFIGURATION;
	(void)usbd_uvc_put16(&buf[2], pos);              /* wTotalLength */
	buf[4] = (u8)(num * 2U);                         /* bNumInterfaces */
	buf[5] = 0x01U;                                  /* bConfigurationValue */
	buf[6] = 0x00U;                                  /* iConfiguration */
	buf[7] = 0x80U;                                  /* bmAttributes: bus powered */
	buf[8] = 0x32U;                                  /* bMaxPower: 100 mA */

	return (u16)pos;
}
