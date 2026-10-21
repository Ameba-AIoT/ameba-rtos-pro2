# AmebaPro2 Local WebRTC Example

## Overview

This example demonstrates **direct WebRTC streaming from AmebaPro2 to a browser** without any cloud service, AWS KVS, intermediary PC, or Raspberry Pi.

The AmebaPro2 acts as a **standalone WebRTC endpoint** that:
- Captures video (H.264) and audio (Opus) via the MMF pipeline
- Runs an HTTP signaling server for SDP offer/answer exchange
- Establishes a WebRTC peer connection directly with the browser
- Streams live audio/video to the browser

All of this runs entirely on the AmebaPro2 MCU. No cloud services needed.

## Architecture

```
Browser (Chrome/Edge/Firefox)
    │
    │  1. HTTP GET  /          → receives viewer HTML page
    │  2. HTTP POST /webrtc/offer → sends SDP offer (with ICE candidates)
    │  3. HTTP ← SDP answer    → receives SDP answer
    │
    │  4. WebRTC (ICE/DTLS/SRTP/H.264) ← media stream
    ▼
AmebaPro2
├── MMF Pipeline: Camera → H.264 Encoder
├── MMF Pipeline: Mic    → Opus Encoder
├── HTTP Signaling Server (port 80)
├── WebRTC Peer Connection (ICE, DTLS, SRTP)
└── mbedTLS, libsrtp, usrsctp
```

## Prerequisites

1. **AmebaPro2 SDK** (this repository)
2. **Camera sensor** (e.g. GC2053, SC2336)
3. **WiFi connection** (same LAN as browser)

## Dependencies

This example reuses the WebRTC protocol stack from `kvs_webrtc_v2_mmf`. It requires
the open-source (Apache 2.0) Amazon KVS WebRTC protocol libraries for ICE, STUN,
SRTP, RTP, and SDP implementations.

### Automatic (recommended)

The cmake auto-fetches the libraries on first configure. No manual step needed.

### Manual

To download before building (e.g., for offline builds):

```bash
cd component/example/kvs_webrtc_v2_mmf
./fetch_aws_lib.sh
```

If the fetch fails (no network), the example is **skipped gracefully** without
blocking the rest of the build.

These libraries are used **without** connecting to any AWS service.

## SDK Configuration

The example requires the following SDK settings. The `cmake` step applies these automatically,
but you can verify them in the corresponding SDK files.

### mbedTLS version

Set the mbedTLS version to **3.0.0** in `project/realtek_amebapro2_v0_example/GCC-RELEASE/config.cmake`:

```cmake
set(mbedtls "mbedtls-3.0.0")
```

The WebRTC protocol stack uses DTLS-SRTP key derivation, which requires `mbedtls-3.x` features.

### mbedTLS FS_IO (automatically disabled)

The cmake disables `MBEDTLS_FS_IO` because `<dirent.h>` is not supported on ARM bare-metal
newlib. This is done via a custom config file (`local_webrtc_src/local_mbedtls_config.h`)
that wraps the default config and undefines `MBEDTLS_FS_IO`, passed through the
`MBEDTLS_CONFIG_FILE` compile definition. The SDK's original config file is never modified.

### lwIP IPv6 support (automatically enabled)

The shared ICE controller code uses `struct sockaddr_in6` and `INET6_ADDRSTRLEN` **unconditionally**
(not guarded by `#if LWIP_IPV6`). Since `lwipopts.h` uses a plain `#define LWIP_IPV6 0`
(not `#ifndef`-guarded), a compile definition alone would be overridden. The cmake creates a
patched copy in the build directory and uses `include_directories(BEFORE ..)` to ensure it
is found first. The SDK's original `lwipopts.h` is never modified.

## Build

```bash
cd project/realtek_amebapro2_v0_example/GCC-RELEASE
mkdir -p build && cd build
cmake .. -G"Unix Makefiles" \
    -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake \
    -DEXAMPLE=local_webrtc_mmf
cmake --build . --target flash -j4
```

Use the Realtek image tool to flash the generated image to AmebaPro2.

## Configuration

### WiFi
On first boot, the board will wait for WiFi. Use AT commands in the UART terminal:

```
ATW0=<WiFi_SSID>          # Set WiFi SSID
ATW1=<WiFi_Password>      # Set WiFi password
ATWC                      # Connect
```

### Video (edit example_local_webrtc_mmf.c)
```c
#define V1_RESOLUTION   VIDEO_HD    // VIDEO_VGA, VIDEO_HD, VIDEO_FHD
#define V1_FPS          15          // Frames per second
#define V1_GOP          15          // Keyframe interval
#define V1_BPS          (512*1024)  // Bitrate
```

### Audio (edit example_local_webrtc_mmf.c)
The example uses Opus audio at 8 kHz sample rate by default. Edit `audio_params` and `opusc_params` to adjust.

## Usage

1. Flash the image and boot the AmebaPro2
2. Wait for WiFi connection (check UART terminal for IP address)
3. Open a browser on the same network and navigate to:
   ```
   http://<AMEBA_IP_ADDRESS>/
   ```
4. The browser page will automatically:
   - Create an RTCPeerConnection
   - Wait for ICE candidate gathering to complete
   - POST the SDP offer to the AmebaPro2
   - Apply the received SDP answer
   - Start displaying the live video stream

### AT Commands
| Command | Action |
|---------|--------|
| `LWST`  | Stop the WebRTC example |

## How It Works

### Signaling (Non-Trickle ICE)
The example uses a simplified HTTP-based signaling protocol:

1. **Browser creates offer** — The JavaScript page creates an RTCPeerConnection, adds a receive-only video transceiver, creates an SDP offer, and **waits for ICE candidate gathering to complete** (`iceGatheringState === "complete"`).

2. **Browser sends complete offer** — The complete SDP offer (with all gathered ICE candidates embedded) is sent to the AmebaPro2 via `POST /webrtc/offer`.

3. **AmebaPro2 processes offer** — The HTTP server receives the offer, creates a peer connection session, sets the remote description (which includes the browser's candidates), sets the local description, and creates an SDP answer.

4. **Local ICE candidates collected** — The AmebaPro2 waits a short time (up to ~3 seconds) for local host candidates to be gathered via the ICE controller callback. These candidates are appended to the answer SDP.

5. **Answer returned** — The complete SDP answer (with the AmebaPro2's candidates) is returned as JSON in the HTTP response.

6. **Browser applies answer** — The browser applies the remote description and ICE connectivity checks begin.

7. **Media flows** — Once DTLS handshake and SRTP setup complete, the AmebaPro2 starts sending H.264 video and Opus audio frames to the browser.

### Why Non-Trickle ICE?
Using non-trickle ICE avoids the need for WebSocket or persistent HTTP connections for signaling. The entire SDP exchange happens in a single request-response cycle, making the implementation simpler and more suitable for embedded devices.

## Files

```
component/example/local_webrtc_mmf/
├── app_example.c                  # Entry point
├── example_local_webrtc_mmf.c     # MMF pipeline setup
├── example_local_webrtc_mmf.h     # Header
├── module_local_webrtc.c          # MMF module (frame bridge)
├── module_local_webrtc.h          # Header
├── local_webrtc_mmf.cmake         # Build configuration
├── readme.md                      # This file
├── local_webrtc_src/
│   ├── local_master.c             # Main WebRTC master task
│   ├── local_master.h             # Header
│   ├── local_app_common.c         # WebRTC session management
│   ├── local_app_common.h         # Header
│   └── demo_config.h              # Audio/video codec config
└── web/
    (HTML page is embedded in local_app_common.c)
```

## Limitations

- **LAN only**: By default, only host ICE candidates are used. Browser and AmebaPro2 must be on the same network.
- **No STUN/TURN**: Remote access across NAT requires a STUN/TURN server (e.g., coturn).
- **Single viewer**: The current implementation supports one browser at a time.
- **Non-trickle ICE**: The browser waits for ICE gathering before sending the offer, which adds ~1-2 seconds to the initial connection time.

## Extending

### Adding STUN/TURN for Remote Access
Edit `LocalAppCommon_StartPeerConnection()` in `local_app_common.c` to configure ICE servers:

```c
IceControllerIceServer_t stunServer;
memset(&stunServer, 0, sizeof(stunServer));
stunServer.serverType = ICE_CONTROLLER_ICE_SERVER_TYPE_STUN;
stunServer.protocol = ICE_SOCKET_PROTOCOL_UDP;
stunServer.iceEndpoint.transportAddress.port = 3478;
strncpy(stunServer.url, "stun.your-server.com", sizeof(stunServer.url));
// ... add to pcConfig.iceServers array
```

### Multiple Viewers
Increase `AWS_MAX_VIEWER_NUM` in `demo_config.h` and `LOCAL_WEBRTC_MAX_VIEWER_NUM` in `local_app_common.h`.

## Comparison: KVS WebRTC vs Local WebRTC

| Feature | kvs_webrtc_v2_mmf | local_webrtc_mmf |
|---------|-------------------|-------------------|
| Cloud service | AWS KVS required | None |
| AWS credentials | Required | Not needed |
| Signaling | AWS KVS WSS | HTTP (built-in server) |
| ICE servers | AWS TURN (included) | None (LAN only) |
| STUN | AWS STUN | Optional, self-hosted |
| Internet access | Required | Not required |
| Setup complexity | High (AWS account, IAM) | Low (just WiFi) |
| Browser viewer | KVS test page | Self-hosted HTML |

## Troubleshooting

| Symptom | Likely Cause | Solution |
|---------|-------------|---------|
| No WiFi connection | Wrong SSID/password | Use `ATW0`/`ATW1`/`ATWC` commands |
| Browser can't load page | Wrong IP or network | Check board IP in UART; ensure same network |
| "Invalid SDP JSON" | SDP exceeds buffer | Increase `PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH` |
| No video after connecting | Unsupported H.264 profile | Ensure browser supports H.264 (most do) |
| Audio but no video | Encoder not started | Check sensor type in `sensor.h` |

## License

Apache License 2.0 — see individual source headers for details.

All files in this example are licensed under the Apache License, Version 2.0.
This example uses the open-source Amazon KVS WebRTC protocol libraries
(Apache 2.0) for ICE, STUN, SRTP, RTP, and SDP protocol implementations.
