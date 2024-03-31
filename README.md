# ROS 2 node for libcamera

This ROS 2 node provides support for a variety of cameras via [libcamera](https://libcamera.org). Amongst others, this node supports V4L2 and [Raspberry Pi cameras](https://www.raspberrypi.com/documentation/computers/camera_software.html).

## Install

### Binary

Binary packages are available via the ROS package repository for some Linux and ROS distributions (check with `rosdep resolve camera_ros`). If it's available, you can install the DEB or RPM packages via:
```sh
# source ROS distribution
source /opt/ros/humble/setup.bash
# DEB package
sudo apt install ros-$ROS_DISTRO-camera-ros
# RPM package
sudo dnf install ros-$ROS_DISTRO-camera-ros
```

### Source

#### libcamera dependency

The `camera_ros` node depends on libcamera version 0.1 or later.

Some Linux and ROS distributions provide binary libcamera packages. Check your package manager for `libcamera` and `rosdep resolve libcamera` to see if binary packages are available.

If your distribution does not provide a binary libcamera package, you have to compile libcamera from source either independent of the colcon workspace according to the [official build instructions](https://libcamera.org/getting-started.html) or as part of the colcon workspace. Either way, you have to install libcamera's build dependencies manually:
```sh
# DEB
sudo apt install pkg-config python3-yaml python3-ply python3-jinja2 openssl libyaml-dev libssl-dev libudev-dev libatomic1 meson
# RPM
sudo dnf install pkgconfig python3-yaml python3-ply python3-jinja2 openssl libyaml-devel openssl-devel libudev-devel libatomic meson
```

#### build camera_ros

The `camera_ros` package is built in a colcon workspace. The following instructions assume that you are building libcamera from source in the colcon workspace.

```sh
# create workspace with camera_ros package
mkdir -p ~/camera_ws/
cd ~/camera_ws/
git clone https://github.com/christianrauch/camera_ros.git src/camera_ros

# optional: build libcamera in colcon workspace
pip install colcon-meson
git clone https://git.libcamera.org/libcamera/libcamera.git src/libcamera

# resolve binary dependencies and build workspace
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --skip-keys=libcamera
colcon build
```

If you installed libcamera externally, you can omit the `colcon-meson` and `libcamera` part. Additionally, if there is a binary package and a rosdep entry for libcamera (check with `rosdep resolve libcamera`) you can also omit `--skip-keys=libcamera` and have this binary dependency resolved automatically.

## Usage

### Node

### Parameters
