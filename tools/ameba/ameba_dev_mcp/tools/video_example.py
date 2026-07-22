"""
Video example selector tools for AmebaPro2.

Provides two MCP tools:
  list_video_examples_tool  — list all known video examples with their NN model
                              requirements; marks which example is currently active.
  set_video_example_tool    — activate a specific video example by:
                                1. editing video_example_media_framework.c
                                   (comment-out the old call, uncomment the new one)
                                2. updating amebapro2_fwfs_nn_models.json FWFS.files
                                   to match the required NN models
                              Returns the `nn` flag so the caller knows whether to
                              invoke build_firmware(video_example=True, nn=True/False).

After calling set_video_example_tool, re-building with video_example=True does NOT
require pristine=True — cmake was already configured with -DVIDEO_EXAMPLE=ON and make
will detect the source change automatically.  pristine=True is only needed when
switching FROM a non-video example.
"""

import json
import os
import re
from typing import List, Optional

from mcp.server.fastmcp import FastMCP

from ameba_dev_mcp._paths import SDK_ROOT

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

_VIDEO_EXAMPLE_C = os.path.join(
    SDK_ROOT,
    "project", "realtek_amebapro2_v0_example",
    "src", "mmfv2_video_example",
    "video_example_media_framework.c",
)

_NN_MODELS_JSON = os.path.join(
    SDK_ROOT,
    "project", "realtek_amebapro2_v0_example",
    "GCC-RELEASE", "mp",
    "amebapro2_fwfs_nn_models.json",
)

# ---------------------------------------------------------------------------
# Example catalog
# Each entry:
#   id          — short key used in tool arguments
#   func        — C function name (without parentheses)
#   name        — human-readable display name
#   description — one-sentence description
#   category    — grouping label
#   nn_models   — list of model keys required in FWFS.files (empty = no NN)
#   nn          — True if build_firmware(nn=True) is needed (flash_ntz.nn.bin)
# ---------------------------------------------------------------------------

_CATALOG: List[dict] = [
    # ── Pure video / A/V (no NN) ─────────────────────────────────────────
    {
        "id": "v1_rtsp",
        "func": "mmf2_video_example_v1_init",
        "name": "CH1 H264/HEVC → RTSP",
        "description": "Single-channel H264/HEVC video stream over RTSP (port 554).",
        "category": "video",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "v2_rtsp",
        "func": "mmf2_video_example_v2_init",
        "name": "CH2 H264/HEVC → RTSP",
        "description": "Channel-2 H264/HEVC video over RTSP.",
        "category": "video",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "v3_jpeg",
        "func": "mmf2_video_example_v3_init",
        "name": "CH3 JPEG → RTSP",
        "description": "JPEG frame stream over RTSP.",
        "category": "video",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "v1_snapshot",
        "func": "mmf2_video_example_v1_shapshot_init",
        "name": "CH1 H264 RTSP + Snapshot",
        "description": "H264/HEVC RTSP with periodic snapshot capture.",
        "category": "video",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "v1_snapshot_hr",
        "func": "mmf2_video_example_v1_snapshot_hr_init",
        "name": "CH1 Snapshot (high-res only)",
        "description": "High-resolution snapshot capture, no RTSP stream.",
        "category": "video",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "v1_snapshot_httpfs",
        "func": "mmf2_video_example_v1_shapshot_httpfs_init",
        "name": "CH1 Snapshot + HTTP File Server",
        "description": "Snapshot delivered via built-in HTTP file server.",
        "category": "video",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "simo",
        "func": "mmf2_video_example_simo_init",
        "name": "1 Video → 2 RTSP streams",
        "description": "Single video source split to two simultaneous H264/HEVC RTSP streams.",
        "category": "video",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "av",
        "func": "mmf2_video_example_av_init",
        "name": "Video + Audio → RTSP",
        "description": "H264/HEVC video with AAC audio over a single RTSP stream.",
        "category": "audio_video",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "av2",
        "func": "mmf2_video_example_av2_init",
        "name": "2 Video + 1 Audio → 2 RTSP",
        "description": "Two H264/HEVC streams each with shared audio, two RTSP sessions.",
        "category": "audio_video",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "av21",
        "func": "mmf2_video_example_av21_init",
        "name": "1 Video + 1 Audio → 2 RTSP",
        "description": "One video+audio source served as two simultaneous RTSP streams.",
        "category": "audio_video",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "av_mp4",
        "func": "mmf2_video_example_av_mp4_init",
        "name": "Video + Audio → MP4 (SD card)",
        "description": "Record H264/HEVC + AAC audio directly to MP4 on SD card.",
        "category": "recording",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "av_rtsp_mp4",
        "func": "mmf2_video_example_av_rtsp_mp4_init",
        "name": "Video + Audio → RTSP + MP4",
        "description": "Simultaneously stream H264+audio over RTSP and record to MP4.",
        "category": "recording",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "av_mp4_httpfs",
        "func": "mmf2_video_example_av_mp4_httpfs_init",
        "name": "Video + Audio → MP4 + HTTP File Server",
        "description": "Record A/V to MP4 and browse files via HTTP.",
        "category": "recording",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "joint_test",
        "func": "mmf2_video_example_joint_test_init",
        "name": "Joint Test (H264 + 2-way audio)",
        "description": "Full A/V test: H264 RTSP, AAC encode, G711 2-way audio.",
        "category": "audio_video",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "joint_test_rtsp_mp4",
        "func": "mmf2_video_example_joint_test_rtsp_mp4_init",
        "name": "Joint Test RTSP + MP4",
        "description": "H264 to both RTSP and MP4 simultaneously with 2-way audio.",
        "category": "recording",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "joint_test_fcs",
        "func": "mmf2_video_example_joint_test_rtsp_mp4_init_fcs",
        "name": "Joint Test RTSP+MP4 (Fast Camera Start)",
        "description": "RTSP+MP4 joint test with fast camera start (FCS) enabled.",
        "category": "recording",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "2way_audio_pcmu_doorbell",
        "func": "mmf2_video_example_2way_audio_pcmu_doorbell_init",
        "name": "H264 RTSP + 2-way PCMU Audio (Doorbell)",
        "description": "H264 RTSP with G711/PCMU 2-way audio and built-in doorbell PCM array.",
        "category": "audio_video",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "2way_audio_pcmu",
        "func": "mmf2_video_example_2way_audio_pcmu_init",
        "name": "H264 RTSP + 2-way PCMU Audio",
        "description": "H264 RTSP with G711/PCMU 2-way audio.",
        "category": "audio_video",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "array_rtsp",
        "func": "mmf2_video_example_array_rtsp_init",
        "name": "H264 Array → RTSP",
        "description": "Pre-encoded H264 array data streamed over RTSP.",
        "category": "video",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "v1_param_change",
        "func": "mmf2_video_example_v1_param_change_init",
        "name": "CH1 Dynamic Parameter Change",
        "description": "Demonstrates runtime video parameter changes (bitrate, resolution).",
        "category": "video",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "v1_day_night",
        "func": "mmf2_video_example_v1_day_night_change_init",
        "name": "CH1 Day/Night Mode Switch",
        "description": "Demonstrates ISP day/night mode switching.",
        "category": "video",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "v1_mask",
        "func": "mmf2_video_example_v1_mask_init",
        "name": "CH1 Privacy Mask",
        "description": "Apply privacy mask overlay on the video stream.",
        "category": "video",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "v1_rate_control",
        "func": "mmf2_video_example_v1_rate_control_init",
        "name": "CH1 Rate Control",
        "description": "Demonstrates dynamic bitrate/rate control on H264/HEVC stream.",
        "category": "video",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "md_rtsp",
        "func": "mmf2_video_example_md_rtsp_init",
        "name": "H264 RTSP + Motion Detection",
        "description": "H264 stream with software motion detection (no NN model needed).",
        "category": "video",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "md_mp4",
        "func": "mmf2_video_example_md_mp4_init",
        "name": "Motion Detection → Event MP4 Recording",
        "description": "Record to MP4 triggered by motion detection events.",
        "category": "recording",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "bayercap_rtsp",
        "func": "mmf2_video_example_bayercap_rtsp_init",
        "name": "Bayer Capture → RTSP",
        "description": "Raw Bayer sensor capture streamed over RTSP.",
        "category": "video",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "demuxer_rtsp",
        "func": "mmf2_video_example_demuxer_rtsp_init",
        "name": "MP4 → RTSP (Demuxer)",
        "description": "Play back an MP4 file over RTSP.",
        "category": "video",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "h264_pcmu_array_mp4",
        "func": "mmf2_video_example_h264_pcmu_array_mp4_init",
        "name": "H264 + PCMU Array → MP4",
        "description": "Record pre-encoded H264 + PCMU audio arrays to MP4.",
        "category": "recording",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "jpeg_external",
        "func": "mmf2_video_example_jpeg_external_init",
        "name": "External Data → JPEG Encode",
        "description": "Encode externally provided raw image data to JPEG.",
        "category": "video",
        "nn_models": [],
        "nn": False,
    },
    {
        "id": "timelapse_mp4",
        "func": "mmf2_video_example_timelapse_mp4_init",
        "name": "Timelapse → MP4",
        "description": "Capture timelapse frames and encode to MP4.",
        "category": "recording",
        "nn_models": [],
        "nn": False,
    },
    # ── NN examples ───────────────────────────────────────────────────────
    {
        "id": "vipnn_rtsp",
        "func": "mmf2_video_example_vipnn_rtsp_init",
        "name": "H264 RTSP + Object Detection (YOLOv4-tiny)",
        "description": "H264 RTSP stream with real-time YOLOv4-tiny object detection overlay.",
        "category": "nn_object_detect",
        "nn_models": ["yolov4_tiny"],
        "nn": True,
    },
    {
        "id": "face_rtsp",
        "func": "mmf2_video_example_face_rtsp_init",
        "name": "H264 RTSP + Face Detect + Recognition",
        "description": "H264 RTSP with SCRFD face detection and MobileFaceNet recognition.",
        "category": "nn_face",
        "nn_models": ["scrfd320p", "mobilefacenet_i8"],
        "nn": True,
    },
    {
        "id": "fd_lm_mfn_cascade",
        "func": "mmf2_video_example_fd_lm_mfn_sim_rtsp_init",
        "name": "3-Model Cascade: FaceDetect + Landmark + FaceNet",
        "description": "3-model cascade: SCRFD face detect → face landmark → MobileFaceNet recognition.",
        "category": "nn_face",
        "nn_models": ["scrfd320p", "mobilefacenet_i8"],
        "nn": True,
    },
    {
        "id": "all_nn_rtsp",
        "func": "mmf2_video_example_joint_test_all_nn_rtsp_init",
        "name": "Full NN Joint Test (Object + Face + Audio NN)",
        "description": "All NN models: YOLOv4-tiny object detect, SCRFD+FaceNet face, YAMNet audio classification.",
        "category": "nn_combined",
        "nn_models": ["yolov4_tiny", "scrfd320p", "mobilefacenet_i8", "yamnet_s_hybrid"],
        "nn": True,
    },
    {
        "id": "audio_vipnn",
        "func": "mmf2_video_example_audio_vipnn_init",
        "name": "Audio NN Classification (YAMNet)",
        "description": "Real-time audio scene/sound classification using YAMNet (no video NN).",
        "category": "nn_audio",
        "nn_models": ["yamnet_s_hybrid"],
        "nn": True,
    },
    {
        "id": "md_nn_rtsp",
        "func": "mmf2_video_example_md_nn_rtsp_init",
        "name": "Motion Detection → NN Object Detect + RTSP",
        "description": "Motion events trigger YOLOv4-tiny object detection on the RTSP stream.",
        "category": "nn_object_detect",
        "nn_models": ["yolov4_tiny"],
        "nn": True,
    },
    {
        "id": "vipnn_facedet",
        "func": "mmf2_video_example_vipnn_facedet_init",
        "name": "H264 RTSP + Face Detection (SCRFD)",
        "description": "H264 RTSP stream with SCRFD face detection bounding boxes.",
        "category": "nn_face",
        "nn_models": ["scrfd320p"],
        "nn": True,
    },
    {
        "id": "vipnn_facedet_sync",
        "func": "mmf2_video_example_vipnn_facedet_sync_init",
        "name": "H264 RTSP + Face Detection (Sync mode)",
        "description": "Face detection with synchronised H264 RTSP and snapshot outputs.",
        "category": "nn_face",
        "nn_models": ["scrfd320p"],
        "nn": True,
    },
    {
        "id": "vipnn_facedet_sync_snapshot",
        "func": "mmf2_video_example_vipnn_facedet_sync_snapshot_init",
        "name": "Face Detection + JPEG Snapshot (Sync)",
        "description": "SCRFD face detection with JPEG snapshot capture in sync mode.",
        "category": "nn_face",
        "nn_models": ["scrfd320p"],
        "nn": True,
    },
    {
        "id": "vipnn_handgesture",
        "func": "mmf2_video_example_vipnn_handgesture_init",
        "name": "H264 RTSP + Hand Gesture (Palm + Landmark)",
        "description": "Hand gesture recognition using palm detection + hand landmark NN models.",
        "category": "nn_gesture",
        "nn_models": ["palm_detection_lite_int16", "hand_landmark_lite_int16"],
        "nn": True,
    },
    {
        "id": "joint_test_vipnn_rtsp_mp4",
        "func": "mmf2_video_example_joint_test_vipnn_rtsp_mp4_init",
        "name": "Full NN Joint Test RTSP + MP4",
        "description": "V+A RTSP+MP4 with NN object detect, face detect+recognition, audio classification.",
        "category": "nn_combined",
        "nn_models": ["yolov4_tiny", "scrfd320p", "mobilefacenet_i8", "yamnet_s_hybrid"],
        "nn": True,
    },
    {
        "id": "vipnn_classify_rtsp",
        "func": "mmf2_video_example_vipnn_classify_rtsp_init",
        "name": "H264 RTSP + Image Classification (MobileNetV2)",
        "description": "H264 RTSP stream with MobileNetV2 image classification output.",
        "category": "nn_classify",
        "nn_models": ["mobilenetv2_int16"],
        "nn": True,
    },
    {
        "id": "dynamic_roi_rtsp",
        "func": "mmf2_video_example_dynamic_roi_rtsp_init",
        "name": "Dynamic ROI + Object Detection",
        "description": "Dynamic region-of-interest tracking driven by YOLOv4-tiny detection.",
        "category": "nn_object_detect",
        "nn_models": ["yolov4_tiny"],
        "nn": True,
    },
]

# Fast lookup by id
_CATALOG_BY_ID = {e["id"]: e for e in _CATALOG}

# ---------------------------------------------------------------------------
# C-file helpers
# ---------------------------------------------------------------------------

# Matches active (uncommented) mmf2_video_example_*_init() calls — tab-indented
_ACTIVE_CALL_RE = re.compile(
    r"^(\t)(mmf2_video_example_\w+\(\);)$",
    re.MULTILINE,
)


def _read_c_file() -> str:
    with open(_VIDEO_EXAMPLE_C, "r", encoding="utf-8") as fh:
        return fh.read()


def _write_c_file(content: str) -> None:
    with open(_VIDEO_EXAMPLE_C, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(content)


def _get_active_func(content: str) -> Optional[str]:
    """Return the function name of the currently active example, or None."""
    m = _ACTIVE_CALL_RE.search(content)
    if m:
        # m.group(2) == "mmf2_video_example_v1_init();"
        call = m.group(2)          # e.g. "mmf2_video_example_v1_init();"
        return call[: call.index("(")]  # strip "();"
    return None


def _comment_all_active_calls(content: str) -> str:
    """Prefix every active mmf2_video_example_*_init() call with //."""
    return _ACTIVE_CALL_RE.sub(r"\1//\2", content)


def _uncomment_call(content: str, func: str) -> tuple[str, bool]:
    """Uncomment a specific func call.  Returns (new_content, found).

    Handles both styles that appear in the stock file:
      \\t//mmf2_video_example_*_init();   (no space after //)
      \\t// mmf2_video_example_*_init();  (one space after //)
    """
    pattern = re.compile(
        r"^(\t)//\s*(" + re.escape(func) + r"\(\);)$",
        re.MULTILINE,
    )
    new_content, count = pattern.subn(r"\1\2", content)
    return new_content, count > 0


# ---------------------------------------------------------------------------
# JSON helpers
# ---------------------------------------------------------------------------

def _update_nn_models_json(models: List[str]) -> dict:
    """Update FWFS.files in the NN models JSON.

    Returns {"previous_models": [...], "new_models": [...]}.
    """
    with open(_NN_MODELS_JSON, "r", encoding="utf-8") as fh:
        data = json.load(fh)

    previous = list(data.get("FWFS", {}).get("files", []))
    data.setdefault("FWFS", {})["files"] = models

    with open(_NN_MODELS_JSON, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(data, fh, indent="\t")
        fh.write("\n")

    return {"previous_models": previous, "new_models": models}


# ---------------------------------------------------------------------------
# MCP tool registration
# ---------------------------------------------------------------------------

def register_video_example_tools(mcp: FastMCP) -> None:
    """Register video example selector tools with the MCP server."""

    @mcp.tool()
    async def list_video_examples_tool() -> dict:
        """List all available video examples for AmebaPro2.

        Reads video_example_media_framework.c to determine which example is
        currently active, then returns the full catalog grouped by category.

        Returns:
            success          bool
            current_example  id of the currently active example (None if unknown)
            current_func     C function name of the active call
            examples         list of example objects — each has:
                               id, func, name, description, category,
                               nn_models (list of model keys), nn (bool),
                               active (bool — True for the currently selected one)
            categories       sorted list of category names present in the catalog
        """
        try:
            content = _read_c_file()
        except OSError as exc:
            return {
                "success": False,
                "error": f"Cannot read C file: {exc}",
                "examples": [],
            }

        active_func = _get_active_func(content)
        active_id = next(
            (e["id"] for e in _CATALOG if e["func"] == active_func), None
        )

        examples = [
            {**e, "active": e["func"] == active_func}
            for e in _CATALOG
        ]
        categories = sorted({e["category"] for e in _CATALOG})

        return {
            "success": True,
            "current_example": active_id,
            "current_func": active_func,
            "examples": examples,
            "categories": categories,
        }

    @mcp.tool()
    async def set_video_example_tool(example_id: str) -> dict:
        """Activate a specific video example for AmebaPro2.

        Modifies two files:
          1. video_example_media_framework.c — comments-out the currently active
             mmf2_video_example_*_init() call and uncomments the chosen one.
          2. amebapro2_fwfs_nn_models.json — updates FWFS.files to the list of
             NN models required by the chosen example (empty list for non-NN
             examples).

        After this call, rebuild with:
          build_firmware(video_example=True, nn=<result.nn>)

        pristine=True is NOT needed when the build directory already exists with
        -DVIDEO_EXAMPLE=ON.  It IS needed when switching from a non-video example
        (e.g. mqtt → video) or when building for the first time.

        Args:
            example_id: id string from list_video_examples_tool (e.g. "v1_rtsp",
                        "vipnn_rtsp", "face_rtsp", …).

        Returns:
            success          bool
            example_id       the id that was set
            func             C function now active
            name             display name of the chosen example
            nn               bool — pass as the nn= argument to build_firmware
            nn_models        list of NN model keys now in FWFS.files
            previous_func    C function that was active before (may be None)
            previous_example id of the example that was active before (may be None)
            c_file_path      absolute path to the modified C file
            json_path        absolute path to the modified NN models JSON
            nn_json_change   {previous_models, new_models} — FWFS.files before/after
            build_hint       ready-made hint string for the next build call
        """
        entry = _CATALOG_BY_ID.get(example_id)
        if entry is None:
            valid = [e["id"] for e in _CATALOG]
            return {
                "success": False,
                "error": f"Unknown example_id '{example_id}'. Valid ids: {valid}",
            }

        # Read C file
        try:
            content = _read_c_file()
        except OSError as exc:
            return {"success": False, "error": f"Cannot read C file: {exc}"}

        # Record what was active before
        previous_func = _get_active_func(content)
        previous_id = next(
            (e["id"] for e in _CATALOG if e["func"] == previous_func), None
        )

        # 1. Comment-out all currently active calls
        content = _comment_all_active_calls(content)

        # 2. Uncomment the target call
        content, found = _uncomment_call(content, entry["func"])
        if not found:
            return {
                "success": False,
                "error": (
                    f"Function '{entry['func']}' not found (commented or otherwise) "
                    f"in {_VIDEO_EXAMPLE_C}. Is the SDK version compatible?"
                ),
            }

        # Write C file
        try:
            _write_c_file(content)
        except OSError as exc:
            return {"success": False, "error": f"Cannot write C file: {exc}"}

        # 3. Update NN models JSON
        try:
            nn_change = _update_nn_models_json(entry["nn_models"])
        except OSError as exc:
            return {
                "success": False,
                "error": f"C file updated OK, but cannot update NN models JSON: {exc}",
            }

        # Build hint
        nn_flag = entry["nn"]
        if nn_flag:
            build_hint = (
                f"build_firmware(video_example=True, nn=True)  "
                f"# produces flash_ntz.nn.bin with models: {entry['nn_models']}"
            )
        else:
            build_hint = (
                "build_firmware(video_example=True, nn=False)  "
                "# produces flash_ntz.bin (no NN models)"
            )

        return {
            "success": True,
            "example_id": example_id,
            "func": entry["func"],
            "name": entry["name"],
            "nn": nn_flag,
            "nn_models": entry["nn_models"],
            "previous_func": previous_func,
            "previous_example": previous_id,
            "c_file_path": _VIDEO_EXAMPLE_C,
            "json_path": _NN_MODELS_JSON,
            "nn_json_change": nn_change,
            "build_hint": build_hint,
        }
