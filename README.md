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

## Launching the Node

The package provides a standalone node executable
```sh
ros2 run camera_ros camera_node
```
and a composable node (`camera::CameraNode`):
```sh
ros2 component standalone camera_ros camera::CameraNode
```

## Parameters

The node provides two sets of parameters:
1. static read-only parameters to configure the camera stream once at the beginning
2. dynamic parameters which are generated from the camera controls to change per-frame settings and which can be changed at runtime

Those parameters can be set on the command line:
```sh
# standalone executable
ros2 run camera_ros camera_node --ros-args -p param1:=arg1 -p param2:=arg2
# composable node
ros2 component standalone camera_ros camera::CameraNode -p param1:=arg1 -p param2:=arg2
```
or dynamically via the [ROS parameter API](https://docs.ros.org/en/rolling/Concepts/Basic/About-Parameters.html).

### Static Camera Stream Configuration

The camera stream is configured once when the node starts via the following static read-only parameters:

| name              | type                  | description |
| ----------------- | --------------------- |  ---------- |
| `camera`          | `integer` or `string` | selects the camera by index (e.g. `0`) or by name (e.g. `/base/soc/i2c0mux/i2c@1/ov5647@36`) [default: `0`]
| `role`            | `string`              | configures the camera with a `StreamRole` (possible choices: `raw`, `still`, `video`, `viewfinder`) [default: `video`]
| `format`          | `string`              | a `PixelFormat` that is supported by the camera [default: auto]
| `width`, `height` | `integer`             | desired image resolution [default: auto]


The configuration is done in the following order:
1. select camera via `camera`
2. configure camera stream via `role`
3. set the pixel format for the stream via `format`
4. set the image resolution for the stream via `width` and `height`

Each stream role only supports a discrete set of data stream configurations as a combination of the image resolution and pixel format. The selected stream configuration is validated at the end of this sequence and adjusted to the closest valid stream configuration.

By default, the node will select the first camera it finds and configures it with the default pixel format and image resolution. If a parameter has not been set, the node will print the available options and inform the user about the default choice.

The node avoids memory copies of the image data by directly mapping from a camera pixel format to a ROS image format, with the exception of converting between "raw" and "compressed" image formats when requested by the user. As an effect, not all pixel formats that are supported by the camera may be supported by the ROS image message. Hence, the options for `format` are limited to pixel formats that are supported by the camera and the raw ROS image message.


### Dynamic Frame Configuration (controls)

The dynamic parameters are created at runtime when the node starts by inspecting the

#### framerate

NOTE no parameter for framerate, needs to be set via frame timing in dynamic params


## FAQ & Trouble Shooting

### Reporting bugs

> [!IMPORTANT]
> test

```sh
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

```sh
export LIBCAMERA_LOG_LEVELS=*:DEBUG
ros2 run --prefix 'gdb -ex run --args' camera_ros camera_node --ros-args -p width:=160 -p height:=120
```


### Q&A

Q1: The node exits with `no cameras available`.
> A1: Check your camera connection and test with the libcamera examples that the camera is supported and accessible.

Q2: The node exits with `Failed to allocate buffers`.
> A2: The requested image size and pixel format may be too large. Set `width` and `height` to a lower resolution or chose a compressed pixel format.
