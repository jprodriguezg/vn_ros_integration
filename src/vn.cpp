#include "ros/ros.h"
#include "vn/VNdata.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "vn");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<vn::VNdata>("vn", 1000);

	while (ros::ok()){
		vn::VNdata msg;
		msg.a = 1;
		msg.b = 2;
		msg.c = 3;

		pub.publish(msg);
	}

	return 0;
}
