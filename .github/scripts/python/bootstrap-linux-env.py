#!/usr/bin/env python3
import os
import subprocess
from pathlib import Path

TOOLCHAIN_URL = "https://github.com/Ameba-AIoT/ameba-toolchain/releases/download/V10.3.0-amebe-rtos-pro2/asdk-10.3.0-linux-newlib-build-3638-x86_64.tar.bz2"
TOOLCHAIN_DIR = Path.home() / "toolchain"
TOOLCHAIN_ARCHIVE = TOOLCHAIN_DIR / "toolchain.tar.bz2"
TOOLCHAIN_BIN_PATH = TOOLCHAIN_DIR / "asdk-10.3.0/linux/newlib/bin"

def run(cmd_list):
    print(f"Running: {' '.join(cmd_list)}")
    subprocess.run(cmd_list, check=True)

def download_and_extract_toolchain():
    TOOLCHAIN_DIR.mkdir(parents=True, exist_ok=True)
    run(["curl", "-L", TOOLCHAIN_URL, "-o", str(TOOLCHAIN_ARCHIVE)])
    run(["tar", "-jxvf", str(TOOLCHAIN_ARCHIVE), "-C", str(TOOLCHAIN_DIR)])

def append_to_github_path():
    github_path = os.getenv("GITHUB_PATH")
    with open(github_path, "a") as f:
        f.write(f"{TOOLCHAIN_BIN_PATH}\n")

def main():
    try:
        download_and_extract_toolchain()
        append_to_github_path()
        print(f"Bootstrap completed successfully.")
    except Exception as e:
        print(f"An error occurred: {e}")
        exit(1)
    
if __name__ == "__main__":
    main()