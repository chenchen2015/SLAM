### OpenCV cmake build flags
#### With CUDA
Need to set `CUDA_NVCC_FLAGS="-ccbin gcc-6"` to use `gcc6` explicitly to compile CUDA NVCC sources since it only support up to `gcc6`.

```bash
cmake .. -DCMAKE_BUILD_TYPE=RELEASE \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DWITH_CUDA=ON \
    -DENABLE_FAST_MATH=ON \
    -DCUDA_FAST_MATH=ON \
    -DWITH_CUBLAS=ON \
    -DWITH_NVCUVID=ON \
    -DWITH_V4L=ON \
    -DWITH_LIBV4L=ON \
    -DWITH_OPENGL=ON \
    -DWITH_FFMPEG=ON \
    -DOPENCV_ENABLE_NONFREE=ON \
    -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules
```

#### Without CUDA
```bash
cmake .. -DCMAKE_BUILD_TYPE=RELEASE \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DENABLE_FAST_MATH=ON \
    -DWITH_VTK=ON \
    -DWITH_OPENGL=ON \
    -DWITH_FFMPEG=ON \
    -DOPENCV_ENABLE_NONFREE=ON \
    -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules
```

When using `cmake >= 3.11`, [CMP0072 needs to be noticed](https://cmake.org/cmake/help/git-stage/policy/CMP0072.html)

Sometimes, `curl` might fail due to `https` not supported. This seems to have caused by `cmake`'s own `curl` was not built with `https` support. Hence, we need to pass an additional flag when building `cmake` to use `curl` that we built on our own with `https` supported by passing: `--system-curl`:
```bash
# bootstrap cmake build
./bootstrap --system-curl
```
This is detailed in [this Stackoverflow answer](https://stackoverflow.com/a/33512778)

### AWS EC2 GPU Instance Setup
First, follow the [AWS tutorial on setting up NVIDIA driver](https://docs.aws.amazon.com/AWSEC2/latest/UserGuide/install-nvidia-driver.html).
Then install CUDA toolkit:
```bash
wget https://developer.nvidia.com/compute/cuda/10.0/Prod/local_installers/cuda_10.0.130_410.48_linux
sudo sh cuda_10.0.130_410.48_linux.run
```

### Setup Dependencies

#### LLVM and Clang 7
```bash
wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key|sudo apt-key add -
sudo add-apt-repository "deb http://apt.llvm.org/cosmic/ llvm-toolchain-cosmic main"
```
Go with full install
```bash
sudo apt-get install libllvm-7-ocaml-dev libllvm7 llvm-7 llvm-7-dev llvm-7-doc llvm-7-examples llvm-7-runtime
sudo apt-get install clang-7 clang-tools-7 clang-7-doc libclang-common-7-dev libclang-7-dev libclang1-7 clang-format-7 python-clang-7
sudo apt-get install libfuzzer-7-dev lldb-7 lld-7 libc++-7-dev libc++abi-7-dev libomp-7-dev
```
or just essentials
```bash
sudo apt install clang-7 clang-format-7 libc++-7-dev python-clang-7
```

#### Axel
for faster download, use `axel`
```bash
sudo apt install axel -y -qq
```

#### Curl
Build curl from source to enable `SSL` support for `https`
First, get `OpenSSL` and `nghttp2`
```bash
sudo apt-get install libssl-dev nghttp2 zlib1g-dev
```
then compile `curl` with `SSL` enabled.
```bash
wget https://curl.haxx.se/download/curl-7.64.0.tar.gz
tar -xf curl-7.64.0.tar.gz
cd curl-7.64.0
./configure --with-ssl --with-nghttp2 --with-zlib
make -j4
sudo make install
```
After that, build/rebuild `cmake` to use system's `curl` we just built.

#### cmake
dependencies
```bash
sudo apt install libncurses5-dev ccache qt5-default qtbase5-dev qttools5-dev -y -qq
```
and build `cmake`, enabling `cmake-gui` and the system curl distribution we just built (with `https` protocol enabled)
```bash
axel -an 8 https://github.com/Kitware/CMake/releases/download/v3.13.4/cmake-3.13.4.tar.gz
tar -xf cmake*
cd cmake-3.13.4
./configure --qt-gui --system-curl --parallel=8
make -j8
sudo make install
```

#### SuiteSparse and SuperLU
```bash
sudo apt install libsuitesparse-dev libsuperlu-dev -y -qq
```

#### CUDA 10
After installation, update path:
```bash
echo 'export PATH="$PATH:/usr/local/cuda-10.0/bin"' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH="/usr/local/cuda-10.0/lib64:$LD_LIBRARY_PATH"' >> ~/.bashrc
```

#### OpenGL dependencies
```bash
sudo apt install freeglut3-dev libglew-dev -y -qq
```

#### Eigen3
```bash
wget http://bitbucket.org/eigen/eigen/get/3.3.7.tar.gz
tar -xf 3.3.7.tar.gz
cd eigen*
mkdir build
cd build
cmake .. 
sudo make install
```

#### Boost
```bash
sudo apt install libicu-dev -yqq
wget https://dl.bintray.com/boostorg/release/1.69.0/source/boost_1_69_0.tar.gz
tar -xf boost_1_69_0.tar.gz
cd boost_*
./bootstrap.sh --prefix=/usr/local --with-icu=
sudo ./b2 install -a -j4
```

#### VTK
```bash
wget https://www.vtk.org/files/release/8.2/VTK-8.2.0.tar.gz
tar -xf VTK-8.2.0.tar.gz
cd VTK-8.2.0
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

#### PCL
```bash
sudo apt install -yqq libflann-dev libpng-dev libglew-dev
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.9.1.tar.gz
tar -xf pcl-1.9.1.tar.gz
cd pcl-pcl*
mkdir build
cd build
ccmake ..
make -j4
sudo make install
```

#### g2o
```bash
sudo apt -qq install qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5 -y
git clone https://github.com/RainerKuemmerle/g2o
cd g2o
mkdir build
cd build
ccmake ..
make -j4
```

#### Ceres-Solver
```bash
sudo apt-get install libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev -yq
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
mkdir build
cd build
ccmake ..
sudo make install
```

#### Sophus
```bash
git clone https://github.com/strasdat/Sophus
cd Sophus
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

#### FFMPEG
- To enable `NVENC`, follow [these steps from ffmpeg's official docs](https://trac.ffmpeg.org/wiki/HWAccelIntro#NVENC).
- Compile flags are recommended from [NVIDIA's website](https://developer.nvidia.com/ffmpeg)
- plus, a not-so-perfect fix for `nvcuvid.h` header issue due to depreciated header file in post-CUDA8 versions: https://github.com/opencv/opencv/issues/9544#issuecomment-423488897

```bash
git clone https://git.videolan.org/git/ffmpeg/nv-codec-headers.git
cd nv-codec-headers
make
sudo make install
```

When building `ffmpeg`, get dependencies first
```bash
sudo apt-get update -qq && sudo apt-get -y install \
  autoconf \
  automake \
  build-essential \
  cmake \
  git-core \
  libass-dev \
  libfreetype6-dev \
  libsdl2-dev \
  libtool \
  libva-dev \
  libvdpau-dev \
  libvorbis-dev \
  libxcb1-dev \
  libxcb-shm0-dev \
  libxcb-xfixes0-dev \
  pkg-config \
  texinfo \
  wget \
  zlib1g-dev \
  libavdevice-dev \
  nasm
```



Then clone and build `ffmpeg` with the following configuration

```bash
./configure --enable-shared \
    --enable-pic \
    --enable-cuda \
    --enable-cuvid \
    --enable-nvenc \
    --enable-nonfree \
    --enable-libnpp \
    --arch=x86_64 \
    --extra-cflags=-I/usr/local/cuda/include \
    --extra-ldflags=-L/usr/local/cuda/lib64
```

#### OpenCV
```bash
git clone https://github.com/opencv/opencv
git clone https://github.com/opencv/opencv_contrib
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt install build-essential pkg-config libavcodec-dev libavformat-dev libswscale-dev libavresample-dev -yq 
sudo apt install python3-dev python3-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper1 libjasper-dev libdc1394-22-dev -yq
```
