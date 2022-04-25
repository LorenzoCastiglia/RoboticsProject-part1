#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"

class ComputeOdometry {
public:
    ComputeOdometry() {
        this->velInput=this->n.subscribe("/cmd_vel", 1000, &ComputeOdometry::eulerOdo, this);
    }
    void mainLoop(){
        ros::Rate loop_rate(10);
        ROS_INFO("Odometry node started\n");
        x0=0.0;
        y0=0.0;
        theta0=0.0;
        ts0=0.0;
        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    void eulerOdo(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        if(ts0==0.0){
            ts0 = msg->header.stamp.sec + msg->header.stamp.nsec * pow(10, -9);
        }
        else {
            ts = msg->header.stamp.sec + msg->header.stamp.nsec * pow(10, -9);
            deltats = ts - ts0;
            theta = theta0 + msg->twist.angular.z * deltats;
            x = x0 + msg->twist.linear.x * deltats;
            y = y0 + msg->twist.linear.y * deltats;
            theta0 = theta;
            x0 = x;
            y0 = y;
            ts0 = ts;

            nav_msgs::Odometry odomMsg;
            odomMsg.pose.pose.position.x = this -> x;
            odomMsg.pose.pose.position.y = this -> y;
            odomMsg.pose.pose.orientation.z = this -> theta;
            this -> odomPub.publish(odomMsg);
        }
    }
    void rungeKuttaOdo(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        
    }
private:
    ros::NodeHandle n;
	ros::Subscriber velInput;
    ros::Publisher odomPub;
    double x, y, theta;
    double x0, y0, theta0;
    double ts0;
    double ts;
    double deltats;
};

int main (int argc, char **argv) {
    ros::init(argc, argv, "ComputeOdometry");
    ComputeOdometry compOdo;
    compOdo.mainLoop();
}