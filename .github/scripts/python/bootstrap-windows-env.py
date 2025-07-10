import zipfile
import subprocess
import urllib.request
from pathlib import Path

MSYS_URL = "https://github.com/Ameba-AIoT/ameba-tool-rtos-pro2/releases/download/msys64_v10_3/msys64_v10_3.7z"
CMAKE_URL = "https://github.com/Kitware/CMake/releases/download/v3.20.0-rc1/cmake-3.20.0-rc1-windows-x86_64.msi"
TOOLCHAIN_URL = "https://github.com/Ameba-AIoT/ameba-toolchain/releases/download/V10.3.0-amebe-rtos-pro2/asdk-10.3.0-mingw32-newlib-build-3633-x86_64.zip"

TOOLCHAIN_DIR = Path.home() / "toolchain"
TOOLCHAIN_ZIP = TOOLCHAIN_DIR / "toolchain.zip"
TOOLCHAIN_BIN = TOOLCHAIN_DIR / "asdk-10.3.0" / "mingw32" / "newlib" / "bin"
CMAKE_INSTALLER = Path.home() / "cmake-installer.msi"
CMAKE_PATH_WIN = Path("C:/Program Files/CMake/bin")
MSYS_7Z = Path.home() / "msys64_v10_3.7z"
MSYS_ROOT = Path.home() / "msys64_v10_3" / "msys64"

def run(cmd, shell=False):
    print(f"Running: {' '.join(cmd) if isinstance(cmd, list) else cmd}")
    subprocess.run(cmd, check=True, shell=shell)

def download_file(url, dest):
    if dest.exists():
        print(f"File already exists at {dest}, skipping download.")
        return
    print(f"Downloading from {url} to {dest} ...")
    urllib.request.urlretrieve(url, dest)

def download_extract_msys():
    print("Downloading MSYS2...")
    download_file(MSYS_URL, MSYS_7Z)
    run(["7z", "x", str(MSYS_7Z), f"-o{str(Path.home())}", "-y"])

def download_extract_toolchain():
    print("Downloading toolchain...")
    TOOLCHAIN_DIR.mkdir(parents=True, exist_ok=True)
    download_file(TOOLCHAIN_URL, TOOLCHAIN_ZIP)
    with zipfile.ZipFile(TOOLCHAIN_ZIP, 'r') as zip_ref:
        zip_ref.extractall(TOOLCHAIN_DIR)
    print(f"Toolchain extracted to: {TOOLCHAIN_DIR}")

def install_cmake():
    print("Installing CMake...")
    download_file(CMAKE_URL, CMAKE_INSTALLER)
    run(["msiexec", "/i", str(CMAKE_INSTALLER), "/quiet", "/norestart"], shell=True)

def main():
    try:
        download_extract_msys()
        download_extract_toolchain()
        install_cmake()
        print(f"Bootstrap completed successfully.")
    except Exception as e:
        print(f"An error occurred: {e}")
        exit(1)

if __name__ == "__main__":
    main()