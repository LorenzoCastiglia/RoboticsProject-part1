#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "project_1/WheelsVel.h"

class WheelsVelocities {

public:
	WheelsVelocities(){
		this -> velInput=this->n.subscribe("/cmd_vel", 1000, &WheelsVelocities::velCallback, this);
		this -> wheelPub=this->n.advertise<project_1::WheelsVel>("/wheels_rpm", 1000);
	}
    void mainLoop(){
        ros::Rate loop_rate(10);
        ROS_INFO("Wheel velocity node started\n");
        ros::spin();
        
    }
    void velCallback(const geometry_msgs::TwistStamped::ConstPtr &msg){
        vx = msg -> twist.linear.x;
        ROS_INFO("Vel x: %f", vx);
        vy = msg -> twist.linear.y;
        ROS_INFO("Vel y: %f", vy);
        wz = msg -> twist.angular.z;
        ROS_INFO("W z: %f", wz);

        wfl = 1/wheelRadius * (vx - vy - (halfLength + halfWidth) * wz);
        ROS_INFO("Rot fl: %f", wfl);
        wfr = 1/wheelRadius * (vx + vy + (halfLength + halfWidth) * wz);
        ROS_INFO("Rot fr: %f", wfr);
        wrl = 1/wheelRadius * (vx + vy - (halfLength + halfWidth) * wz);
        ROS_INFO("Rot rr: %f", wrl);
        wrr = 1/wheelRadius * (vx - vy + (halfLength + halfWidth) * wz);
        ROS_INFO("Rot rl: %f", wrr);

        project_1::WheelsVel wheelsMsg;
        wheelsMsg.rpm_fl = wfl;
        wheelsMsg.rpm_fr = wfr;
        wheelsMsg.rpm_rr = wrr;
        wheelsMsg.rpm_rl = wrl;
        this -> wheelPub.publish(wheelsMsg);

    }
private:
	ros::NodeHandle n;
	ros::Subscriber velInput;
	ros::Publisher wheelPub;
    const int gearRatio = 5;
    const double wheelRadius = 0.07;
    const double halfLength = 0.2;
    const double halfWidth = 0.169;
    double wfl, wfr, wrl, wrr;
    double vx, vy, wz;
};

int main (int argc, char **argv) {
	ros::init(argc, argv, "WheelsVelocities");
    WheelsVelocities wheelsVel;
    wheelsVel.mainLoop();
}