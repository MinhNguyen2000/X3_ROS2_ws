# Script to build the ros2_humble image

#!/bin/bash
IMAGE_NAME="ros2_humble"

# Get the directory where THIS script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${SCRIPT_DIR}"

# Docker image build
echo "Building ${IMAGE_NAME}"
docker build -t "${IMAGE_NAME}" .
docker image prune -f
echo "Build complete!"