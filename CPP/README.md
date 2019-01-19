### OpenCV cmake build flags
```bash
cmake .. -DCMAKE_BUILD_TYPE=RELEASE \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DWITH_CUDA=ON \
    -DENABLE_FAST_MATH=1 \
    -DCUDA_FAST_MATH=1 \
    -DWITH_CUBLAS=1 \
    -DOPENCV_ENABLE_NON_FREE=1 \
    -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules
```
