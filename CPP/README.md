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

#### OpenCV
