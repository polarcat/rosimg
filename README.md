### About

Simple image subscriber and viewer for ROS2 (tested in Foxy).

### Limitations

Currently viewer only works with RGB raw and compressed images (jpg, png). Other image formats like theora or depth might be added in the future when the need arises.

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

        ~/> mkdir build
        ~/> cd build
        ~/build> cmake .. && make

### Run (w/o installation)

 * Example for topic with uncompressed image:

        ~/build> ./rosimg /image_raw

### Install (optional)

 * Option 1 (sudo might be required)

        ~/build> make install

 * Option 2

        ~/build> cp -v ./rosimg $HOME/bin/

### Usage

 * Example for topic with compressed image:

        ~/> rosimg /image_raw/compressed
