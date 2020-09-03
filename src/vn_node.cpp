/******************************************************************************/
/*****                              vn_node.c                             *****/
/*****                             Raul Tapia                             *****/
/*****                          GRIFFIN Project                           *****/
/*****                         GRVC-Robotics Lab.                         *****/
/******************************************************************************/

/**
 * @file    vn_node.c
 * @author  Raul Tapia (raultapia _at_ us.es | github.com/raultapia)
 * @brief   ROS node for VN integration
 */

#include "ros/ros.h"
#include "sensor_msgs/TimeReference.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "VNdata.h"
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char **argv){
								ros::init(argc, argv, "vn");
								ros::NodeHandle n;

								init_vn(DEFAULT_UART_PORT);
								VNMeasures vn;

								ros::Publisher pubGpsTime = n.advertise<sensor_msgs::TimeReference>("vn/gps_time", 1000);
								ros::Publisher pubImu  = n.advertise<sensor_msgs::Imu>("vn/imu", 1000);
								ros::Publisher pubMag  = n.advertise<sensor_msgs::MagneticField>("vn/mag", 1000);
								ros::Publisher pubPosLla = n.advertise<sensor_msgs::NavSatFix>("vn/pos_lla", 1000);
								ros::Publisher pubPosEcef = n.advertise<geometry_msgs::Vector3Stamped>("vn/pos_ecef", 1000);
								ros::Publisher pubVelBody = n.advertise<geometry_msgs::Vector3Stamped>("vn/vel_body", 1000);
								ros::Publisher pubAccBody = n.advertise<geometry_msgs::Vector3Stamped>("vn/acc_body", 1000);

								while (ros::ok()) {
																get_vn_data(&vn);

																/*** --- Messages --- ***/
																sensor_msgs::TimeReference gpsTimeMsg;
																sensor_msgs::Imu imuMsg;
																sensor_msgs::MagneticField magMsg;
																sensor_msgs::NavSatFix posLlaMsg;
																geometry_msgs::Vector3Stamped posEcefMsg;
																geometry_msgs::Vector3Stamped velBodyMsg;
																geometry_msgs::Vector3Stamped accBodyMsg;

																/*** --- Header --- ***/
																std_msgs::Header header;
																header.stamp = ros::Time::now();
																header.frame_id = "VN200";

																/*** --- Message composition --- ***/
																gpsTimeMsg.header = header;
																gpsTimeMsg.time_ref = ros::Time(vn.gpstime);

																imuMsg.header = header;
																tf2::Quaternion q;
																q.setRPY(vn.attitude.roll, vn.attitude.pitch, vn.attitude.yaw);
																q.normalize();
																imuMsg.orientation.x = q[0];
																imuMsg.orientation.y = q[1];
																imuMsg.orientation.z = q[2];
																imuMsg.orientation.w = q[3];
																imuMsg.angular_velocity.x = vn.gyro.x;
																imuMsg.angular_velocity.y = vn.gyro.y;
																imuMsg.angular_velocity.z = vn.gyro.z;
																imuMsg.linear_acceleration.x = vn.accel.x;
																imuMsg.linear_acceleration.y = vn.accel.y;
																imuMsg.linear_acceleration.z = vn.accel.z;

																magMsg.header = header;
																magMsg.magnetic_field.x = vn.mag.x;
																magMsg.magnetic_field.y = vn.mag.y;
																magMsg.magnetic_field.z = vn.mag.z;

																posLlaMsg.header = header;
																posLlaMsg.latitude = vn.poslla.latitude;
																posLlaMsg.longitude = vn.poslla.longitude;
																posLlaMsg.altitude = vn.poslla.altitude;

																posEcefMsg.header = header;
																posEcefMsg.vector.x = vn.posecef.x;
																posEcefMsg.vector.y = vn.posecef.y;
																posEcefMsg.vector.z = vn.posecef.z;

																velBodyMsg.header = header;
																velBodyMsg.vector.x = vn.velbody.x;
																velBodyMsg.vector.y = vn.velbody.y;
																velBodyMsg.vector.z = vn.velbody.z;

																accBodyMsg.header = header;
																accBodyMsg.vector.x = vn.accbody.x;
																accBodyMsg.vector.y = vn.accbody.y;
																accBodyMsg.vector.z = vn.accbody.z;

																/*** --- Publications --- ***/
																pubGpsTime.publish(gpsTimeMsg);
																pubImu.publish(imuMsg);
																pubMag.publish(magMsg);
																pubPosLla.publish(posLlaMsg);
																pubPosEcef.publish(posEcefMsg);
																pubVelBody.publish(velBodyMsg);
																pubAccBody.publish(accBodyMsg);
								}

								return 0;
}
