#!/bin/bash
set -e

echo "Starting GTSAM build from source..."

# Install Required Dependencies (Boost >= 1.70 is required)
echo "Installing Boost dependencies..."
sudo apt-get update
sudo apt-get install -y libboost-all-dev

# Directory to build GTSAM
BUILD_DIR="${HOME}/gtsam_ws"
mkdir -p ${BUILD_DIR}
cd ${BUILD_DIR}

# Clone the GTSAM repository if it doesn't exist
if [ ! -d "gtsam" ]; then
    echo "Cloning GTSAM..."
    git clone https://github.com/borglab/gtsam.git
fi

cd gtsam
git checkout 4.2a9 # Checking out a stable/known tag

# Create build directory
mkdir -p build
cd build

# Configure CMake
echo "Configuring CMake..."
# We disable tests and examples to speed up the build for VINS-Mono integration
cmake .. \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DCMAKE_BUILD_TYPE=Release

# Build and Install
echo "Building GTSAM (this may take a while)..."
make -j$(nproc)

echo "Installing GTSAM..."
sudo make install

echo "GTSAM successfully installed from source!"
