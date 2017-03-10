# rosproxy

ROS proxy node for expanding abbreviated nodes into standard nodes.

Useful for resource limited devices like arduino or anything using rosserial to be able to publish messages that may otherwise be too large or bandwidth intensive to publish.

Currently implements proxy nodes for Odom messages (nav_msgs/Odometry.h) and Imu messages (sensor_msgs/Imu.h).

## Usage

Clone to src directory of catkin workspace, then 'catkin_make'.

Copy rosproxy_arduino/sketchbook/libraries/ros_lib/rosproxy_msgs to the ros_lib directory of your Arduino libraries.

Sample launch files in rosproxy_server/launch.

Sample Arduino code in rosproxy_arduino/sketchbook.

## Authors

* **Chad Attermann** - *Initial work* - [Ecos Technologies](https://github.com/ecostech)

## License

This project is licensed under the BSD License.

