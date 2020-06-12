# Catkin setup
Due to how catkin_make handles non ROS packages the instructions are different if you use 'catkin_make' to 'catkin build'.
Catkin build is recommened as everything can be built together. This instructions show how to build with catkin_make.
You shold follow the origin README.md until the 'Catkin setup' section then follow these instructions and go back and continue from 'Setup workspace'.

TODO add catkin_make instructions...

## Build OpenCV

```
cmake --cmake-args -DCMAKE_BUILD_TYPE=RELEASE \
    -DWITH_CUDA=ON \
    -DCUDA_GENERATION=Auto \
    -DWITH_CUBLAS=ON \
    -DWITH_TBB=ON \
    -DWITH_V4L=ON \
    -DWITH_QT=ON \
    -DWITH_OPENGL=ON \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_TIFF=ON \
    -DENABLE_CXX11=ON \
    -DWITH_PROTOBUF=OFF \
    -DBUILD_opencv_legacy=OFF \
    -DENABLE_PRECOMPILED_HEADERS=OFF \
    -DINSTALL_PYTHON_EXAMPLES=ON \
    -DINSTALL_C_EXAMPLES=OFF \
    -DOPENCV_EXTRA_MODULES_PATH=../../src/opencv_contrib/modules \
    -DCUDA_NVCC_FLAGS="-D_FORCE_INLINES" \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DWITH_CUDNN=ON \
    -DOPENCV_DNN_CUDA=ON \
    -DENABLE_FAST_MATH=1 \
    -DCUDA_FAST_MATH=1 \
    -DBUILD_opencv_dnn=OFF \
    -DBUILD_opencv_text=OFF ..
```


## Build Octomap

## Build AprilTag

## Build RTabMap

## Build catkin workspace
```
catkin_make
```

## Setup OpenCV DIR:
Set OpenCV_DIR so packages use this OpenCV version rather than the ros inbuilt version
Add this to ~/.bashrc otherwise will have to run this everytime you build
```
echo 'export OpenCV_DIR="PATH_TO_REPO/deps/opencv"' >> ~/.bashrc
```
*Make sure to change 'PATH_TO_REPO' to the path to where you cloned this repository*

## Add license for I3DRSGM
License file should be provided from I3DR.
Contact info@i3drobotics.com for a license to use I3DRSGM.

Copy I3DRSGM license file (.lic) to 
```
PATH_TO_REPO/devel/lib/i3dr_stereo_camera/
```
If you do not have a license then use build option -DWITH_I3DRSGM=OFF