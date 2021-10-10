### About

Simple image subscriber and viewer for ROS2 (tested in Foxy).

### Limitations

Currently it only works with RGB JPEG compression. Other image formats support will be added when the need arises.

### Dependencies

 - [ros2](https://github.com/ros2) for providing ROS2 environment
 - [glfw](https://github.com/glfw/glfw]) version 3 for windows management,
   graphics context etc.
 - [glad2](https://github.com/Dav1dde/glad) OpenGL loader.
 - [stb](https://github.com/nothings/stb) for image manipulation.

### Build

 * Install required dependecies
 * Setup ROS2 environment (follow ROS2 guide)
 * Compile

        ~/> mkdir build
        ~/> cd build
        ~/build> cmake .. && make

### Run (w/o installation)

 * Example for /camera0/compressed topic:

        ~/build> ./image-viewer /camera0/compressed

### Install (optional)

 * Option 1 (sudo might be required)

        ~/build> make install

 * Option 2

        ~/build> cp -v ./image-viewer $HOME/bin/

### Usage

 * Example for /camera0/compressed topic:

        ~/> image-viewer /camera0/compressed
