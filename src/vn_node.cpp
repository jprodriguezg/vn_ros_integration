/******************************************************************************/
/*****                              VNdata.c                              *****/
/*****                             Raul Tapia                             *****/
/*****                          GRIFFIN Project                           *****/
/*****                         GRVC-Robotics Lab.                         *****/
/******************************************************************************/

/**
 * @file    vn_node.c
 * @brief   ROS node for VN integration
 * @author  Raul Tapia
 */

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "VNdata.h"
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "vn");
	ros::NodeHandle n;

	init_vn(DEFAULT_UART_PORT);
	VNMeasures vn;

	ros::Publisher pubImu = n.advertise<sensor_msgs::Imu>("vn/imu", 1000);
	ros::Publisher pubPosLLA = n.advertise<sensor_msgs::NavSatFix>("vn/lla_position", 1000);
	ros::Publisher pubPosECEF = n.advertise<geometry_msgs::Vector3Stamped>("vn/ecef_position", 1000);
	ros::Publisher pubBodyVelocity = n.advertise<geometry_msgs::Vector3Stamped>("vn/body_velocity", 1000);

	while (ros::ok()){
		get_vn_data(&vn);

		/*** --- Messages --- ***/
		sensor_msgs::Imu imuMsg;
		sensor_msgs::NavSatFix posLLAMsg;
		geometry_msgs::Vector3Stamped posECEFMsg;
		geometry_msgs::Vector3Stamped bodyVelocityMsg;

		/*** --- Header --- ***/
		std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.frame_id = "VN200";
		
		/*** --- Message composition --- ***/
		tf2::Quaternion q;
		q.setRPY(vn.attitude.roll, vn.attitude.pitch, vn.attitude.yaw);
		q.normalize();
		imuMsg.header = header;
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

		posLLAMsg.header = header;
		posLLAMsg.latitude = vn.poslla.latitude;
		posLLAMsg.longitude = vn.poslla.longitude;
		posLLAMsg.altitude = vn.poslla.altitude;
		
		posECEFMsg.header = header;
		posECEFMsg.vector.x = vn.posecef.x;
		posECEFMsg.vector.y = vn.posecef.y;
		posECEFMsg.vector.z = vn.posecef.z;

		bodyVelocityMsg.header = header;
		bodyVelocityMsg.vector.x = vn.velbody.x;
		bodyVelocityMsg.vector.y = vn.velbody.y;
		bodyVelocityMsg.vector.z = vn.velbody.z;

		/*** --- Publications --- ***/
		pubImu.publish(imuMsg);
		pubPosLLA.publish(posLLAMsg);
		pubPosECEF.publish(posECEFMsg);
		pubBodyVelocity.publish(bodyVelocityMsg);
	}

	return 0;
}
