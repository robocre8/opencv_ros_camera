## opencv_ros_camera
this is a simple ros pacakge for interfacing camera using opencv.
it publishes on topics:

````
<camera_optical_frame_id>/image_raw
<camera_optical_frame_id>/image_raw/compressed
```` 

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

- cd into the src folder of your <ros_ws> and download the mobo_bot packages
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

#

#### View published camera data on your PC
- install rqt_image_view
  ```shell
  sudo apt-get install ros-humble-rqt-image-view
  ```
  
- run to view published images (compressed or raw)
  ```shell
  ros2 run rqt_image_view rqt_image_view
  ```