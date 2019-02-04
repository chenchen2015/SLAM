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
    -DCUDA_NVCC_FLAGS="-ccbin gcc-6" \
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
    -DWITH_V4L=ON \
    -DWITH_LIBV4L=ON \
    -DWITH_OPENGL=ON \
    -DWITH_FFMPEG=ON \
    -DOPENCV_ENABLE_NONFREE=ON \
    -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules
```

When using `cmake >= 3.11`, [CMP0072 needs to be noticed](https://cmake.org/cmake/help/git-stage/policy/CMP0072.html)

### AWS EC2 GPU Instance Setup
First, follow the [AWS tutorial on setting up NVIDIA driver](https://docs.aws.amazon.com/AWSEC2/latest/UserGuide/install-nvidia-driver.html).
Then install CUDA toolkit:
```bash
wget https://developer.nvidia.com/compute/cuda/10.0/Prod/local_installers/cuda_10.0.130_410.48_linux
sudo sh cuda_10.0.130_410.48_linux.run
```

### Setup Dependencies

#### cmake
```bash
sudo apt -qq install cmake cmake-curses-gui -y
```

#### SuiteSparse
```bash
sudo apt -qq install libsuitesparse-dev -y
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
sudo apt install -yqq libflann-dev
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
cmake ..
make -j4
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

#### OpenCV
```bash
git clone https://github.com/opencv/opencv
git clone https://github.com/opencv/opencv_contrib
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt install build-essential pkg-config libavcodec-dev libavformat-dev libswscale-dev -yq 
sudo apt install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper1 libjasper-dev libdc1394-22-dev -yq
```
