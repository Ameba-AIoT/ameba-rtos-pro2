import os
import re
import sys
import subprocess
import shutil

EXAMPLES = [
    "mmf2_video_example_v1_init",
    "mmf2_video_example_v2_init",
    "mmf2_video_example_v3_init",
    "mmf2_video_example_v1_shapshot_init",
    "mmf2_video_example_simo_init",
    "mmf2_video_example_av_init",
    "mmf2_video_example_av2_init",
    "mmf2_video_example_av21_init",
    "mmf2_video_example_av_mp4_init",
    "mmf2_video_example_av_rtsp_mp4_init",
    "mmf2_video_example_joint_test_init",
    "mmf2_video_example_joint_test_rtsp_mp4_init",
    "mmf2_video_example_2way_audio_pcmu_doorbell_init",
    "mmf2_video_example_2way_audio_pcmu_init",
    "mmf2_video_example_array_rtsp_init",
    "mmf2_video_example_v1_param_change_init",
    "mmf2_video_example_v1_day_night_change_init",
    "mmf2_video_example_v1_mask_init",
    "mmf2_video_example_v1_rate_control_init",
    "mmf2_video_example_av_mp4_httpfs_init",
    "mmf2_video_example_vipnn_rtsp_init", 
    "mmf2_video_example_face_rtsp_init",
    "mmf2_video_example_fd_lm_mfn_sim_rtsp_init",
    "mmf2_video_example_joint_test_all_nn_rtsp_init",
    "mmf2_video_example_demuxer_rtsp_init",
    "mmf2_video_example_h264_pcmu_array_mp4_init",
    "mmf2_video_example_audio_vipnn_init",
    "mmf2_video_example_md_rtsp_init",
    "mmf2_video_example_md_mp4_init",
    "mmf2_video_example_bayercap_rtsp_init",
    "mmf2_video_example_md_nn_rtsp_init",
    "mmf2_video_example_joint_test_rtsp_mp4_init_fcs",
    "mmf2_video_example_vipnn_facedet_init",
    "mmf2_video_example_jpeg_external_init",
    "mmf2_video_example_vipnn_facedet_sync_init",
    "mmf2_video_example_vipnn_facedet_sync_snapshot_init",
    "mmf2_video_example_vipnn_handgesture_init",
    "mmf2_video_example_joint_test_vipnn_rtsp_mp4_init",
    "mmf2_video_example_vipnn_classify_rtsp_init",
    "mmf2_video_example_timelapse_mp4_init"
]

PROJECT_DIR = os.path.abspath(os.path.join(os.path.abspath(__file__), "..", "..", "..", "..", "project", "realtek_amebapro2_v0_example"))
SRC_DIR = os.path.join(PROJECT_DIR, "src", "mmfv2_video_example")
GCC_RELEASE_DIR = os.path.join(PROJECT_DIR, "GCC-RELEASE")
BUILD_DIR = os.path.join(GCC_RELEASE_DIR, "build")
BIN_OUTPUT_DIR = os.path.join(PROJECT_DIR, "bin_outputs")
TOOLCHAIN_FILE = os.path.join(GCC_RELEASE_DIR, "toolchain.cmake")
SRC_FILE = os.path.join(SRC_DIR, "video_example_media_framework.c")

def run(cmd):
    print(f"Running: {cmd}")
    result = subprocess.run(cmd, shell=True, text=True, capture_output=True, check=True)
    print(result.stdout)
    if result.stderr:
        print(result.stderr, file=sys.stderr)

def prepare_source_file(source_path, examples, target_example):
    with open(source_path, 'r') as file:
        content = file.read()

    # Comment out all example functions if not already commented
    for example in examples:
        content = re.sub(
            rf"^\s*{example}",
            f"//{example}",
            content,
            flags=re.MULTILINE
        )
    # Uncomment only the target example
    content = re.sub(
        rf"^\s*//\s*{target_example}",
        target_example,
        content,
        flags=re.MULTILINE
    )
    with open(source_path, 'w') as file:
        file.write(content)
        
def build_example(example):
    print(f"Building {example}...")
    prepare_source_file(SRC_FILE, EXAMPLES, example)

    build_dir = os.path.join(GCC_RELEASE_DIR, f"build_{example}")
    os.makedirs(build_dir, exist_ok=True)
    os.chdir(build_dir)

    # Run cmake config
    run(f'cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE={TOOLCHAIN_FILE} -DVIDEO_EXAMPLE=on')

    # Fix permissions for all .linux files
    mp_dir = os.path.join(GCC_RELEASE_DIR, "mp")
    run(f'find {mp_dir} -name "*.linux" -exec chmod +x {{}} \\;')

    # Build target
    if "nn" in example.lower():
        run('cmake --build . --target flash_nn -j4')
    else:
        run('cmake --build . --target flash -j4')
    
    # Copy built binary file to output directory
    built_bin_name = "flash_ntz.nn.bin" if "nn" in example.lower() else "flash_ntz.bin"
    built_bin_path = os.path.join(build_dir, built_bin_name)
    output_bin_path = os.path.join(BIN_OUTPUT_DIR, f"{example}.bin")
    os.makedirs(BIN_OUTPUT_DIR, exist_ok=True)
    shutil.copyfile(built_bin_path, output_bin_path)
    
    # Clean for next build
    run('make clean')
    os.chdir("..")

def main():
    try:
        # Get the list of examples passed
        examples_to_build = sys.argv[1:]
        if not examples_to_build:
            examples_to_build = EXAMPLES

        # Build all the examples sequentially within the same batch
        for example in examples_to_build:
            build_example(example)

    except FileNotFoundError as e:
        print(f"File error: {e.filename} not found.")
        exit(1)

    except subprocess.CalledProcessError as e:
        print(f"Subprocess failed with return code {e.returncode}")
        print(f"Command: {e.cmd}")
        print(f"Output:\n{e.output}")
        print(f"Error Output:\n{e.stderr}")
        exit(e.returncode)

    except Exception as e:
        print(f"Unexpected error: {str(e)}")
        exit(1)

if __name__ == "__main__":
    main()