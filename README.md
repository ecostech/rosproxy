# rosproxy

ROS proxy node for expanding abbreviated nodes into standard nodes.

Useful for resource limited devcices like arduino or anything using rosserial to be able to publish messages that may otherwise be too large or bandwidth intensive to publish.

Currently implements proxy nodes for Odom messages (nav_msgs/Odometry.h) and Imu messages (<sensor_msgs/Imu.h>).

## Authors

* **Chad Attermann** - *Initial work* - [Ecos Technologies](https://github.com/ecostech)

## License

This project is licensed under the BSD License.

