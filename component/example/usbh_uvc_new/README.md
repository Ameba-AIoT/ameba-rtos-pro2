# USB Host UVC Example — AmebaPro2

This example demonstrates USB Video Class (UVC) host functionality on the AmebaPro2
platform. It works with standard UVC-compliant USB cameras (webcams) connected to
the board's USB OTG port.

Three example applications are provided:

| Application | Description |
|-------------|-------------|
| **FATFS**   | Captures video frames to an SD card as image files |
| **RGB**     | Captures frames, converts JPEG/YUY2 → RGB888, saves as PPM |
| **EMPTY**   | Captures and discards frames (for testing) |

---

## Supported formats

| Resolution | YUY2    | MJPEG |
|------------|---------|-------|
| 160×120    | ✅      | ✅    |
| 320×240    | ✅      | ✅    |
| 640×480    | ⚠️\*    | ✅    |
| 1280×720   | ❌      | ✅    |

Actual support depends on the camera's own descriptors — the driver negotiates
the closest format/resolution the device advertises.

\* Many webcams cannot sustain uncompressed YUY2 at 640×480 over USB 2.0 High
Speed. Such a camera accepts the probe/commit but sends only header-only packets
(e.g. 12 bytes) with zero pixel payload. This is a camera bandwidth limitation,
not a driver bug — use MJPEG at higher resolutions.

---

## Example structure

### Application flow

```
main()
 └─ example_usbh_uvc()
     └─ rtw_create_task("example_usbh_uvc_thread")
         └─ example_usbh_uvc_task()
             ├─ otg_select_usb_mode(1)          ← Select USB host mode
             ├─ usbh_init()                      ← Init USB host controller
             ├─ usbh_uvc_init()                  ← Register UVC class driver
             │   └─ callback: uvc_cb_attach()    ← Camera detected
             ├─ usbh_uvc_set_param()             ← Negotiate format/resolution/fps
             ├─ usbh_uvc_stream_on()             ← Start streaming
             ├─ [loop]
             │   ├─ usbh_uvc_get_frame()         ← Wait for next frame
             │   ├─ uvc_img_prepare()            ← Copy frame data
             │   ├─ [save to SD or discard]
             │   └─ usbh_uvc_put_frame()         ← Return buffer
             ├─ usbh_uvc_stream_off()            ← Stop streaming
             └─ usbh_uvc_deinit()                ← Unregister UVC driver
```

### Key files

| File | Purpose |
|------|---------|
| `example_usbh_uvc_new.c` | Application code: init, streaming loop, SD save |
| `usb/host_new/uvc/usbh_uvc_intf.h` | Public UVC API |
| `usb/host_new/uvc/usbh_uvc_intf.c` | Public API implementation |
| `usb/host_new/uvc/usbh_uvc_class.c` | Class driver: attach/detach, state machine |
| `usb/host_new/uvc/usbh_uvc_stream.c` | Isochronous streaming, URB management, frame decode |
| `usb/host_new/uvc/usbh_uvc_parse.c` | Descriptor parser |
| `usb/host_new/uvc/usbh_uvc_desc.h` | UVC descriptor structs and constants |
| `usb/host_new/uvc/usbh_uvc.h` | Internal driver header |

---

## Public API

All functions declared in `usbh_uvc_intf.h`.

### `int usbh_uvc_init(usbh_uvc_cb_t *cb)`

Register the UVC class driver. Call once after `usbh_init()`.

`cb` may be NULL. The callback struct:

```c
typedef struct {
    int (*init)(void);      // called when driver initialises
    int (*deinit)(void);    // called when driver deinitialises
    int (*attach)(void);    // called when UVC device is detected
    int (*detach)(void);    // called when UVC device is removed
} usbh_uvc_cb_t;
```

### `void usbh_uvc_deinit(void)`

Unregister the UVC driver. Stops streaming if active.

### `int usbh_uvc_set_param(struct uvc_config *para)`

Negotiate format, resolution and frame rate.

```c
struct uvc_config {
    int fmt_type;     // UVC_FORMAT_MJPEG or UVC_FORMAT_UNCOMPRESSED
    int width;        // desired width  (closest match selected)
    int height;       // desired height (closest match selected)
    int frame_rate;   // desired fps    (closest match selected)
};
```

On success, the struct fields are updated to the actual values accepted.

### `int usbh_uvc_stream_on(void)`

Start isochronous video streaming.

### `int usbh_uvc_stream_off(void)`

Stop isochronous video streaming.

### `int usbh_uvc_stream_state(void)`

Returns `STREAMING_ON (1)` or `STREAMING_OFF (0)`.

### `uvc_frame *usbh_uvc_get_frame(void)`

Block until a complete video frame is available. Returns `NULL` after a
2-second timeout.

```c
typedef struct {
    u8  *buf;       // pixel data
    u32  byteused;  // bytes of valid data in buf
    u32  err;       // non-zero if frame has errors
} uvc_frame;
```

**You must call `usbh_uvc_put_frame()` to return the buffer.** Holding
it starves the driver of free buffers.

### `void usbh_uvc_put_frame(uvc_frame *frame)`

Return a frame buffer to the driver's free pool.

---

## Usage guidelines

### Correct streaming loop

```c
// Initialise
usbh_uvc_init(&uvc_cb);
usbh_uvc_set_param(&uvc_ctx);

// Stream ON once (NOT per-frame)
usbh_uvc_stream_on();
rtw_mdelay_os(500);          // allow camera to warm up

// Continuous capture
while (connected) {
    uvc_frame *frame = usbh_uvc_get_frame();
    if (frame == NULL) {
        rtw_mdelay_os(100);
        continue;
    }
    process(frame);
    usbh_uvc_put_frame(frame);
}

// Cleanup
usbh_uvc_stream_off();
usbh_uvc_deinit();
```

### Common mistake

Do **not** do this:

```c
while (1) {
    usbh_uvc_stream_on();     // ← wrong: called every frame
    frame = usbh_uvc_get_frame();
    usbh_uvc_stream_off();    // ← wrong
    rtw_mdelay_os(1000);
}
```

Cameras may not deliver the first frame after a fresh `stream_on` within
a single frame interval, and repeated on/off cycles can leave the camera
in an undefined state.

---

## Configuration

Edit the defines in `example_usbh_uvc_new.c`:

```c
// Application mode
#define CONFIG_USBH_UVC_APP    USBH_UVC_APP_FATFS   // or USBH_UVC_APP_EMPTY or USBH_UVC_APP_RGB

// Video format, resolution and frame rate
#define USBH_UVC_FORMAT_TYPE   UVC_FORMAT_UNCOMPRESSED
#define USBH_UVC_WIDTH         320
#define USBH_UVC_HEIGHT        240
#define USBH_UVC_FRAME_RATE    15

// Application buffer size (must hold one full frame)
// 320×240 YUY2 = 153600 bytes  →  (320*1024)
// 640×480 MJPEG ≈ 50000 bytes   →  (200*1024)
// 640×480 YUY2 = 614400 bytes   →  (640*1024)
#define USBH_UVC_BUF_SIZE      (320*1024U)

// Max images to save (FATFS mode only)
#define USBH_UVC_MAX_IMAGES    200
```

When changing the format, also update the filename extension in
`uvc_fatfs_thread()`:

```c
snprintf(filename, sizeof(filename), "img%d.jpg",  ...);  // MJPEG
snprintf(filename, sizeof(filename), "img%d.yuy2", ...);  // YUY2
```

## RGB conversion mode (`USBH_UVC_APP_RGB`)

The `USBH_UVC_APP_RGB` mode converts captured frames to RGB888 before saving:

| Input format | Conversion | Output | File extension |
|-------------|-----------|--------|----------------|
| MJPEG | `stb_image` JPEG decoder | RGB888 | `imgN.ppm` (PPM P6) |
| YUY2 | YUV→RGB via `rtscolor.h` | RGB888 | `imgN.ppm` (PPM P6) |

Output files use [PPM P6](http://netpbm.sourceforge.net/doc/ppm.html) format,
viewable in most image viewers (GIMP, ImageMagick, etc.).

**MJPEG note:** Webcam Motion-JPEG frames frequently omit the Huffman table
(DHT) segment and rely on the standard tables from the JPEG spec (Annex K). A
plain JPEG decoder cannot decode these ("bad huffman code"). `uvc_jpeg_to_rgb()`
detects a missing DHT and injects the standard MJPEG Huffman tables before
decoding, so headerless MJPEG streams decode correctly.

### Standalone API

```c
#include "uvc_to_rgb.h"

// JPEG → RGB888
int w, h;
uint8_t *rgb = uvc_jpeg_to_rgb(jpeg_buf, jpeg_len, &w, &h, NULL,
                                UVC_RGB_FORMAT_RGB888);
uvc_rgb_free(rgb);

// YUY2 → RGB888
uint8_t buf[640 * 480 * 3];
uvc_yuy2_to_rgb(yuy2_buf, 640, 480, buf, UVC_RGB_FORMAT_BGR888);
```

---

## Compiling and running

### Prerequisites

- AmebaPro2 SDK with GCC or IAR toolchain
- A UVC-compliant USB camera (webcam)
- Micro-USB OTG cable
- SD card (FATFS / RGB mode)

### Steps

1. **Build and flash**:

   ```bash
   cd project/realtek_amebapro2_v0_example/GCC-RELEASE/build
   cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DEXAMPLE=usbh_uvc_new
   cmake --build . --target flash
   ```

2. **Hardware setup**:
   - Connect the USB camera to the board's micro-USB OTG port
   - Insert an SD card (FAT32) into the SD slot
   - Power on and observe serial output

### Expected output (FATFS)

```
[UVC] USB host uvc demo started...
UVC init
USB physical connected
UVC attach

Set UVC parameters
UVC para: 320*240@15fps

UVC stream on

Create image file: 0:/img0.yuy2
Write 153600 bytes

Create image file: 0:/img1.yuy2
Write 153600 bytes
...
Reached 200 images, stopping capture
```

---

## SD card notes

- **Format:** FAT32 recommended (FAT16 root directory overflows after ~100 files)
- Insert the SD card **before** power-on
- Each 320×240 YUY2 frame is about 150 KB
- Warmup: first 30 frames discarded (camera stabilisation)
- Throttle: 1 in every 15 frames is saved
- Capture stops at `USBH_UVC_MAX_IMAGES` (default 200)
