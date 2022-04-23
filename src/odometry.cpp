#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"

class ComputeOdometry {
public:
    ComputeOdometry() {
        this->velInput=this->n.subscribe("/cmd_vel", 1000, &ComputeOdometry::eulerOdo, this);
    }
    void mainLoop(){
        ros::Rate loop_rate(10);
        ROS_INFO("Odometry node started\n");
        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    void eulerOdo(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        
    }
private:
    ros::NodeHandle n;
	ros::Subscriber velInput;
};

int main (int argc, char **argv) {
    ros::init(argc, argv, "ComputeOdometry");
    ComputeOdometry compOdo;
    compOdo.mainLoop();
}