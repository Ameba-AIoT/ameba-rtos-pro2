import os
import sys
import subprocess
import shutil

PROJECT_DIR = os.path.abspath(os.path.join(os.path.abspath(__file__), "..", "..", "..", "..", "project", "realtek_amebapro2_v0_example"))
GCC_RELEASE_DIR = os.path.join(PROJECT_DIR, "GCC-RELEASE")
SRC_MAIN_C = os.path.join(PROJECT_DIR, "src", "main.c")
EXAMPLE_SOURCES_DIR = os.path.join(PROJECT_DIR, "example_sources")
BIN_OUTPUT_DIR = os.path.join(PROJECT_DIR, "bin_outputs")

def run(cmd):
    print(f"Running: {cmd}")
    result = subprocess.run(cmd, shell=True, text=True, capture_output=True, check=True)
    print(result.stdout)
    if result.stderr:
        print(result.stderr, file=sys.stderr)

def prepare_source_file(example_name):
    example_main_path = os.path.join(EXAMPLE_SOURCES_DIR, example_name, "src", "main.c")
    if not os.path.isfile(example_main_path):
        raise FileNotFoundError(f"Source file not found: {example_main_path}")
    shutil.copyfile(example_main_path, SRC_MAIN_C)
    print(f"Replaced {SRC_MAIN_C} with source from {example_main_path}")

def build_example(example):
    print(f"Building example: {example}")
    prepare_source_file(example)

    build_dir = os.path.join(GCC_RELEASE_DIR, f"build_{example}")
    os.makedirs(build_dir, exist_ok=True)
    os.chdir(build_dir)

    run(f'cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake')
    run('cmake --build . --target flash -j4')

    built_bin_path = os.path.join(build_dir, "flash_ntz.bin")
    output_bin_path = os.path.join(BIN_OUTPUT_DIR, f"{example}.bin")
    os.makedirs(BIN_OUTPUT_DIR, exist_ok=True)
    shutil.copyfile(built_bin_path, output_bin_path)
    print(f"Output binary copied to {output_bin_path}")

    run('make clean')
    os.chdir("..")

def main():
    try:
        # Get the list of examples passed
        examples_to_build = sys.argv[1:]

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