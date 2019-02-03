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

### AWS EC2 GPU instance setup
FIrst, follow the [AWS tutorial on setting up NVIDIA driver](https://docs.aws.amazon.com/AWSEC2/latest/UserGuide/install-nvidia-driver.html).
