# I3DR Mapping ROS Workspace (RTABMap)
This is a ROS workspace for mapping using I3DR Stereo Camera Systems with RTABMap. 

# Setup dependencies

## Install usb camera dependencies (Required for Deimos only)
```
sudo apt-get install libv4l-dev v4l-utils qv4l2 v4l2ucp
```

## Install pylon (Required for Phobos only)
Download sdk from pylon website: https://www.baslerweb.com/de/support/downloads/downloads-software/

Add rosdep dependencies for pylon
```
sudo sh -c 'echo "yaml https://raw.githubusercontent.com/basler/pylon-ros-camera/master/pylon_camera/rosdep/pylon_sdk.yaml" > /etc/ros/rosdep/sources.list.d/30-pylon_camera.list'
rosdep update
```

## Install CUDA (Required for using OpenCV CUDA stereo matchers and I3DRSGM):
Install CUDA (10.2):
```
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/cuda-ubuntu1604.pin
sudo mv cuda-ubuntu1604.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget http://developer.download.nvidia.com/compute/cuda/10.2/Prod/local_installers/cuda-repo-ubuntu1604-10-2-local-10.2.89-440.33.01_1.0-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu1604-10-2-local-10.2.89-440.33.01_1.0-1_amd64.deb
sudo apt-key add /var/cuda-repo-10-2-local-10.2.89-440.33.01/7fa2af80.pub
sudo apt-get update
sudo apt-get -y install cuda
```
Add the following to ~/.bashrc
```
export LD_LIBRARY_PATH=/usr/local/cuda/lib64
export PATH=$PATH:/usr/local/cuda-10.2/bin
```

## Install OpenCV dependencies:
Install OpenCV dependencies
```
sudo apt install --assume-yes build-essential cmake git pkg-config unzip ffmpeg qtbase5-dev python-dev python3-dev python-numpy python3-numpy
sudo apt install libhdf5-dev
sudo apt install --assume-yes libgtk-3-dev libdc1394-22 libdc1394-22-dev libjpeg-dev libpng12-dev libtiff5-dev libjasper-dev
sudo apt install --assume-yes libavcodec-dev libavformat-dev libswscale-dev libxine2-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
sudo apt install --assume-yes libv4l-dev libtbb-dev libfaac-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev
sudo apt install --assume-yes libvorbis-dev libxvidcore-dev v4l-utils
```
Set OpenCV_DIR so packages use this OpenCV version rather than the ros inbuilt version
Add this to ~/.bashrc otherwise will have to run this everytime you build
```
echo 'export OpenCV_DIR="PATH_TO_REPO/build/opencv3"' >> ~/.bashrc
```
*Make sure to change 'PATH_TO_REPO' to the path to where you cloned this repository*

## Install rtabmap dependencies:
```
sudo apt-get install libsqlite3-dev libpcl-dev libopencv-dev git cmake libproj-dev libqt5svg5-dev
sudo apt-get install python3-dev
```

## Install I3DRSGM library:
Request I3DRSGM library installer from I3DR (info@i3drobotics.com)

Install library deb:
```
sudo dpkg -i Phobos-1.0.54-x86_64_reducedTemplates.deb
```
Add library path to LD_LIBRARY_PATH (add this line to the end of ~/.bashrc):
```
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/Phobos/lib/' >> ~/.bashrc
```

# Setup workspace

## Initalise wstool
```
mkdir PATH_TO_REPO/src
cd PATH_TO_REPO/src
wstool init
```
*Make sure to change 'PATH_TO_REPO' to the path to where you cloned this repository*

## Get i3dr_stereo_camera-ros package
```
git clone https://github.com/i3drobotics/i3dr_stereo_camera-ros.git
```

## Add i3dr packages to wstool
```
wstool merge i3dr_stereo_camera-ros/install/i3dr_stereo_camera_ros_https.rosinstall
```

## Add RTabMap packages to wstool
```
wstool merge i3dr_stereo_camera-ros/install/i3dr_rtabmap_ros_https.rosinstall
```

## Update wstool (download packages)
```
wstool update
```

## Install ros package dependcies:
```
export ROS_DISTRO=kinetic
cd PATH_TO_REPO
rosdep install --from-paths src --ignore-src -r -y
```
*Make sure to change 'PATH_TO_REPO' to the path to where you cloned this repository*

## Setup usb camera permissions (Required for Deimos only)
Copy udev rules for usb permission:
```
cd PATH_TO_REPO/src/i3dr_deimos-ros/
sudo cp udev/99-uvc.rules /etc/udev/rules.d/
```
*Make sure to change 'PATH_TO_REPO' to the path to where you cloned this repository*

## Make OpenCV ROS compatible
Copy package.xml file from 'deps' in this repository to the opencv folder.
This gives the required information to catkin to be able to build it in ROS.
```
sudo cp PATH_TO_REPO/deps/opencv/package.xml PATH_TO_REPO/src/opencv/package.xml
```
*Make sure to change 'PATH_TO_REPO' to the path to where you cloned this repository*

# Build workspace
Configure catkin workspace (this will also build OpenCV, RTabMap, and Octomap with all the required options)
```
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RELEASE \
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
    -DBUILD_opencv_text=OFF \
    -DENABLE_RTABMAP_OCTOMAP=ON \
    -DWITH_I3DRSGM=ON
```
Build workspace
```
cd PATH_TO_REPO
catkin init
catkin config -DWITH_CUDA=OFF -DWITH_QT=ON -DWITH_I3DRSGM=ON
catkin build
```

# Apply license for I3DRSGM
Contact info@i3drobotics.com for a license to use I3DRSGM.

Copy I3DRSGM license file (.lic) to 
```
PATH_TO_REPO/devel/.private/i3dr_stereo_camera/lib/i3dr_stereo_camera/
```
If you do not have a license then use build option -DWITH_I3DRSGM=OFF

# Source workspace
Source workspace
```
source devel/setup.bash
```

# Run
```
roslaunch i3dr_phobos phobos.launch rviz:=true map:=true

roslaunch i3dr_deimos deimos.launch rviz:=true map:=true
```

# Developer only
(**Do not run thesethis if you don't know what you are doing**)

## Build arduino firmare
Build arduino firmware
```
catkin config -DARDUINO_SDK_PATH=PATH_TO_REPO/src/rosserial_adafruit_bno055/tools/arduino-linux
e.g. catkin config -DARDUINO_SDK_PATH=/home/i3dr/i3dr_mapping_ws/src/rosserial_adafruit_bno055/tools/arduino-linux
```
*Make sure to change 'PATH_TO_REPO' to the path to where you cloned this repository*