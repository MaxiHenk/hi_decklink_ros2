
set -e # Fail on error

# Install compiler and tools
apt-get -qq install build-essential

# Install minimal set of required dependencies
apt-get -qq install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev


cd /opt/

# Download opencv-git & checkout appropriate tag
git clone https://github.com/opencv/opencv.git
cd opencv/ 
git checkout 3.3.1
cd .. 

# Download opencv-contrib-git & checkout appropriate tag
git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib
git checkout 3.3.1
cd ..

# Build OpenCV
mkdir opencv/cmake-build
cd opencv/cmake-build

cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      ..

make -j$(nproc)
make install
