#!/bin/bash

# ==============================================================================
# AWS WebRTC Libraries Extraction Script (Fixed Relative Path Version)
# ==============================================================================

# 1. Automatically determine the script's exact physical location
# This ensures the script works perfectly no matter where you execute it from.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

# 2. Define the target directory relative to the script's location
# Script is at: <root>/component/example/kvs_webrtc_v2_mmf/
# Target is at: <root>/project/realtek_amebapro2_v0_example/src/amazon_kvs/lib_amazon_v2
TARGET_DIR="${SCRIPT_DIR}/../../../project/realtek_amebapro2_v0_example/src/amazon_kvs/lib_amazon_v2"

TEMP_DIR="${SCRIPT_DIR}/.temp_aws_clone"
AWS_REF_REPO="https://github.com/awslabs/freertos-webrtc-reference-on-amebapro-for-amazon-kinesis-video-streams.git"

echo "Starting the extraction of AWS WebRTC Libraries..."
echo "Target destination: ${TARGET_DIR}"

# 3. Clean up existing directories to ensure a fresh environment
if [ -d "$TARGET_DIR/libraries" ]; then
    echo "Existing 'libraries' directory found. Cleaning up..."
    rm -rf "$TARGET_DIR/libraries"
fi
if [ -d "$TEMP_DIR" ]; then
    rm -rf "$TEMP_DIR"
fi

# 4. Clone the AWS Reference Repository to a temporary directory
echo "Cloning the AWS Reference Repository to temporary directory..."
git clone "$AWS_REF_REPO" "$TEMP_DIR"

cd "$TEMP_DIR" || exit

# 5. Initialize and update ONLY the required submodules
echo "Fetching specific submodules..."

# Third-party open-source libraries
git submodule update --init --recursive libraries/libsrtp
git submodule update --init --recursive libraries/wslay
rm -rf libraries/usrsctp
echo "Cloning customized usrsctp (webrtc-on-freertos branch)..."
git clone -b webrtc-on-freertos https://github.com/ambiot-mini/usrsctp.git libraries/usrsctp

# AWS IoT libraries
git submodule update --init --recursive libraries/coreHTTP
git submodule update --init --recursive libraries/coreJSON

# SigV4 module inside crypto
git submodule update --init --recursive libraries/crypto/SigV4-for-AWS-IoT-embedded-sdk

# AWS KVS WebRTC core components
git submodule update --init --recursive libraries/components/amazon-kinesis-video-streams-dcep
git submodule update --init --recursive libraries/components/amazon-kinesis-video-streams-ice
git submodule update --init --recursive libraries/components/amazon-kinesis-video-streams-rtcp
git submodule update --init --recursive libraries/components/amazon-kinesis-video-streams-rtp
git submodule update --init --recursive libraries/components/amazon-kinesis-video-streams-sdp
git submodule update --init --recursive libraries/components/amazon-kinesis-video-streams-signaling
git submodule update --init --recursive libraries/components/amazon-kinesis-video-streams-stun

cd ..

# 6. Move the extracted 'libraries' folder to the actual target directory
echo "Moving extracted libraries to the target directory..."
# Create the parent directories if they don't exist
mkdir -p "$TARGET_DIR"
mv "$TEMP_DIR/libraries" "$TARGET_DIR/"

echo "Removing empty unused SDK folders..."
rm -rf "$TARGET_DIR/libraries/ambpro2_sdk"

# 7. Remove the temporary AWS repository wrapper
echo "Removing temporary repository..."
rm -rf "$TEMP_DIR"

echo "Extraction completed successfully!"