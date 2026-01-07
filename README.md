## opencv_ros_camera
this is a simple ros pacakge for interfacing camera using opencv.
it publishes on topics:

````
<camera_frame_id>/image
```` 

A simple ros package for reading published image data the [opencv_image_subscriber](https://github.com/robocre8/opencv_image_subscriber)

#

#### Installation Steps
- install the following on the Pi or Pc
  ```shell
  sudo apt install libopencv-dev python3-opencv
  pip3 install opencv-python
  ```

- create your <ros_ws> in the home dir. (replace <ros_ws> with your workspace name)
  ```shell
  mkdir -p ~/<ros_ws>/src
  cd ~/<ros_ws>
  colcon build
  source ~/<ros_ws>/install/setup.bash
  ```

- cd into the src folder of your <ros_ws> and download the package
  ```shell
  cd ~/<ros_ws>/src
  git clone https://github.com/robocre8/opencv_ros_camera.git
  ```

- cd into the root directory of your <ros_ws> and run rosdep to install all necessary ros  package dependencies
  ```shell
  cd ~/<ros_ws>/
  rosdep install --from-paths src --ignore-src -r -y
  ```

- build your <ros_ws>
  ```shell
  cd ~/<ros_ws>/
  colcon build --symlink-install
  ```

- don't forget to source your <ros_ws> in any new terminal
  ```shell
  source ~/<ros_ws>/install/setup.bash
  ```

#

#### Use the package

- set the video port of your camera from the `camera_publisher.launch.py` file
  ```shell
  source ~/<ros_ws>/install/setup.bash
  ```

- run the ros opencv camera publisher node
  ```shell
  ros2 launch opencv_ros_camera camera_publisher.launch.py
  ```

#### Publish full compress images with image transport

````
<camera_frame_id>/image
<camera_frame_id>/image_raw
<camera_frame_id>/image_raw/compressed
<camera_frame_id>/image_raw/compressedDepth
<camera_frame_id>/image_raw/theora
<camera_frame_id>/image_raw/zstd
```` 

- set the video port of your camera from the `camera_publisher.launch.py` file
  ```shell
  source ~/<ros_ws>/install/setup.bash
  ```

- run the ros opencv camera publisher node
  ```shell
  ros2 launch opencv_ros_camera image_transport.launch.py
  ```

#

#### View published camera data on your PC
- install rqt_image_view
  ```shell
  sudo apt-get install ros-[DISTRO]-rqt-image-view
  ```
  
- run to view published images (compressed or raw)
  ```shell
  ros2 run rqt_image_view rqt_image_view
  ```