# I3DR Mapping ROS Workspace (RTABMap)
This is a ROS workspace for mapping using I3DR Stereo Camera Systems with RTABMap. 

# Setup dependencies

## ROS
This guide assumes ROS Kinetic / Melodic is fully installed. For install instructions see [Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) or [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

wstool and catkin build are used to setup and build the workspace so these needs to be installed with the following command:
``` 
sudo apt-get install python-wstool python-catkin-tools
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
Download CUDA 10.2 for Ubuntu 16.04):
```
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/cuda-ubuntu1604.pin
sudo mv cuda-ubuntu1604.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget http://developer.download.nvidia.com/compute/cuda/10.2/Prod/local_installers/cuda-repo-ubuntu1604-10-2-local-10.2.89-440.33.01_1.0-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu1604-10-2-local-10.2.89-440.33.01_1.0-1_amd64.deb
sudo apt-key add /var/cuda-repo-10-2-local-10.2.89-440.33.01/7fa2af80.pub
```
Or download CUDA 10.2 for Ubuntu 18.04):
```
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin
sudo mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub
sudo add-apt-repository "deb http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/ /"
```
Install CUDA 10.2
```
sudo apt-get update
sudo apt-get -y install cuda
```
Add the following to ~/.bashrc
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64
export PATH=$PATH:/usr/local/cuda-10.2/bin
export CUDA_CACHE_MAXSIZE=2147483648
export CUDA_CACHE_DISABLE=0
```
The CUDA_CACHE variables are set to avoid problems when stereo matching large images. 

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

## Install rtabmap dependencies:
```
sudo apt-get install libsqlite3-dev libpcl-dev libopencv-dev git cmake libproj-dev libqt5svg5-dev python3-dev
```

# Initalise workspace

## Initalise wstool
```
cd PATH_TO_REPO
mkdir src
cd src
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

## Add i3dr camera specific packages to wstool
(Required for Deimos only)
```
wstool merge i3dr_stereo_camera-ros/install/i3dr_deimos_ros_https.rosinstall
```
(Required for Phobos only)
```
wstool merge i3dr_stereo_camera-ros/install/i3dr_phobos_ros_https.rosinstall
```

## Add RTabMap packages to wstool
```
wstool merge i3dr_stereo_camera-ros/install/i3dr_rtabmap_ros_https.rosinstall
```

## Update wstool (download packages)
```
wstool update
```

# Catkin setup
Due to how catkin_make handles non ROS packages the build process is different if you use 'catkin_make' to 'catkin build'.
Catkin build is recommened as everything can be built together. (Catkin_make instructions will be added at a later date).

## Setup OpenCV DIR:
Set OpenCV_DIR so packages use this OpenCV version rather than the ros inbuilt version.

Add the following to ~/.bashrc otherwise will have to run this everytime you build:
```
export OpenCV_DIR="PATH_TO_REPO/build/opencv3"
```
*Make sure to change 'PATH_TO_REPO' to the path to where you cloned this repository*

## Make OpenCV ROS compatible
Copy package.xml file from 'deps' in this repository to the opencv folder.
This gives the required information to catkin to be able to build it in ROS.
```
cd PATH_TO_REPO
cp deps/opencv/package.xml src/opencv/package.xml
```
This is done so that the latest 3.4 version of opencv can be used without relying on a 3rd party to keep the package up to date.
*Make sure to change 'PATH_TO_REPO' to the path to where you cloned this repository*

# Setup workspace
## Define ros distro version
Add the following to ~/.bashrc:
```
export ROS_DISTRO=kinetic
```
## Install ros package dependcies:
```
cd PATH_TO_REPO
rosdep install --from-paths src --ignore-src -r -y
```
*Make sure to change 'PATH_TO_REPO' to the path to where you cloned this repository*

# Build workspace
Configure catkin workspace (this will also build OpenCV, RTabMap, and Octomap with all the required options)
```
cd PATH_TO_REPO
catkin init
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
*Make sure to change 'PATH_TO_REPO' to the path to where you cloned this repository*

Build workspace
```
catkin build
```
*Note: This fully builds OpenCV with the required modules inside the workspace so may take a while (approx 20mins on a fast PC)*

## Add license for I3DRSGM
License file should be provided from I3DR.
Contact info@i3drobotics.com for a license to use I3DRSGM.

Copy I3DRSGM license file (.lic) to 
```
PATH_TO_REPO/devel/.private/i3dr_stereo_camera/lib/i3dr_stereo_camera/
```
If you do not have a license then use build option -DWITH_I3DRSGM=OFF

## Setup usb camera permissions (Required for Deimos only)
Copy udev rules for usb permission:
```
cd PATH_TO_REPO/src/i3dr_deimos-ros/
sudo cp udev/99-uvc.rules /etc/udev/rules.d/
```
*Make sure to change 'PATH_TO_REPO' to the path to where you cloned this repository*

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
(**Do not run these if you don't know what you are doing**)

## Build arduino firmware
Arduino firmware requires building of the package with arduino sdk path. 
Then the firmware built after the package due to issues with the build order.
Build with arduino sdk
```
catkin config -DARDUINO_SDK_PATH=PATH_TO_REPO/src/i3dr_rosserial_phobos-ros/tools/arduino-linux
e.g. catkin config -DARDUINO_SDK_PATH=/home/i3dr/i3dr_mapping_ws/src/i3dr_rosserial_phobos-ros/tools/arduino-linux
catkin build i3dr_rosserial_phobos
```
*Make sure to change 'PATH_TO_REPO' to the path to where you cloned this repository*

Now the package has been built, build arduino firmware
```
source devel/setup.bash
catkin build --no-deps i3dr_rosserial_phobos --make-args i3dr_rosserial_phobos_firmware_arduino_micro
```
Upload it to the arduino:
```
catkin build --no-deps i3dr_rosserial_phobos --make-args i3dr_rosserial_phobos_firmware_arduino_micro-upload
```
