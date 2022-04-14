#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <sstream>

void testCallback(const sensor_msgs::JointState::ConstPtr& msg){
	ROS_INFO("Velocity: [%f]", msg->velocity);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_listener");
	ros::NodeHandle n;
	ROS_INFO("Node started");

	ros::Subscriber test = n.subscribe("/wheel_states", 1000, testCallback);

	ros::spin();

	return 0;
}