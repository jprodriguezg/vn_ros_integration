# VN ROS Integration
[![Latest Version](https://img.shields.io/github/release/grvcPerception/vn_ros_integration)](https://github.com/grvcPerception/vn_ros_integration/releases)
[![License       ](https://img.shields.io/github/license/grvcPerception/vn_ros_integration)](LICENSE)
[![Size          ](https://img.shields.io/github/repo-size/grvcPerception/vn_ros_integration)](README.md)

2020, [ GRVC Robotics Laboratory at the University of Seville](https://grvc.us.es/).

[VN-200](https://www.vectornav.com/products/vn-200) ROS integration.

## Topics
| Name         | Description                                              | Message type                  |
|--------------|----------------------------------------------------------|-------------------------------|
| /vn/imu      | IMU (orientation, angular velocity, linear acceleration) | sensor_msgs::Imu              |
| /vn/mag      | Magnetometer                                             | sensor_msgs::MagneticField    |
| /vn/gps_time | GPS time reference                                       | sensor_msgs::TimeReference    |
| /vn/pos_lla  | GPS longitude, latitude and altitude                     | sensor_msgs::NavSatFix        |
| /vn/pos_ecef | GPS ECEF position                                        | geometry_msgs::Vector3Stamped |
| /vn/vel_body | Body frame velocity                                      | geometry_msgs::Vector3Stamped |
| /vn/acc_body | Body frame acceleration                                  | geometry_msgs::Vector3Stamped |

