### About

Simple image subscriber and viewer for ROS2 (tested in Foxy, Humble).

### Limitations

Currently viewer only works with RGB raw and JPEG compressed images. Other image formats like PNG, Theora or depth can be added in the future when the need arises.

### Dependencies

#### External

 - [ros2](https://github.com/ros2) for providing ROS2 environment
 - [glfw](https://github.com/glfw/glfw]) version 3 for windows management,
   graphics context etc.

#### Included

 - [glad2](https://github.com/Dav1dde/glad) OpenGL loader.
 - [stb](https://github.com/nothings/stb) for image manipulation.

### Build

 * Install external dependencies
 * Setup ROS2 environment (follow ROS2 guide)
 * Compile

```shell
~/> mkdir build
~/> cd build
~/build> cmake .. && make
```

Build will produce 2 binaries: `rosimg` and `rosbag-preview`

### Run (w/o installation)

 * Example for topic with uncompressed image:

```shell
# make sure ros2 bag player is run elsewhere
~/build> ./rosimg /image_raw
# this will read images straight from ROS bag file
# so no ros2 bag player is needed
~/build> ./rosbag-preview rosbag.db3 /image_raw
```

### Install (optional)

 * Option 1 (sudo might be required)

```shell
~/build> make install
```

 * Option 2

```shell
~/build> cp -v ./rosimg $HOME/bin/
~/build> cp -v ./rosbag-preview $HOME/bin/
```

### Usage

 * Example for topic with compressed camera image:

```shell
~/> rosimg /camera/compressed
~/> rosbag-preview rosbag.db3 /camera/compressed
```
