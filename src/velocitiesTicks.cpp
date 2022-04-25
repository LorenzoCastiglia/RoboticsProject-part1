#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"

class ComputeVelocity {
public:
	ComputeVelocity(){
		this->sensorInput=this->n.subscribe("/wheel_states", 1000, &ComputeVelocity::sensorCallback,this);
		this->velocitiesPub=this->n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1000);
	}
    void mainLoop(){
        ros::Rate loop_rate(10);
        ticfl0=0.0;
        ticfr0=0.0;
        ticrr0=0.0;
        ticrl0=0.0;
        tswfl0=0.0;
        tswfr0=0.0;
        tswrr0=0.0;
        tswrl0=0.0;
        ROS_INFO("Geometry node started\n");
        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }
        
    }
    void sensorCallback(const sensor_msgs::JointState::ConstPtr& msg){
        
        if(numMsg%intMsg==0){
            ticfl = msg->position[0]/gearRatio;
            tswfl = msg->header.stamp.sec + msg->header.stamp.nsec * pow(10, -9);
            wfl = (ticfl - ticfl0)/(tswfl - tswfl0);
            ROS_INFO("Rot fl: %f", wfl);
            ticfl0 = ticfl;
            tswfl0 = tswfl;

            ticfr = msg->position[1]/gearRatio;
            tswfr = msg->header.stamp.sec + msg->header.stamp.nsec * pow(10, -9);
            wfr = (ticfr - ticfr0)/(tswfr - tswfr0);
            ROS_INFO("Rot fr: %f", wfr);
            ticfr0 = ticfr;
            tswfr0 = tswfr;

            ticrl = msg->position[2]/gearRatio;
            tswrl = msg->header.stamp.sec + msg->header.stamp.nsec * pow(10, -9);
            wrl = (ticrl - ticrl0)/(tswrl - tswrl0);
            ROS_INFO("Rot rl: %f", wrl);
            ticrl0 = ticrl;
            tswrl0 = tswrl;

            ticrr = msg->position[3]/gearRatio;
            tswrr = msg->header.stamp.sec + msg->header.stamp.nsec * pow(10, -9);
            wrr = (ticrr - ticrr0)/(tswrr - tswrr0);
            ROS_INFO("Rot rr: %f", wrr);
            ticrr0 = ticrr;
            tswrr0 = tswrr;

            vx = (wfl+wfr+wrr+wrl)*wheelRadius/4;
            ROS_INFO("Vel x: %f", vx);
            vy = (-wfl+wfr+wrl-wrr)*wheelRadius/4;
            ROS_INFO("Vel y: %f", vy);
            wz = (-wfl+wfr-wrl+wrr)*wheelRadius/(4*(wheelRadius+halfLength));
            ROS_INFO("W z: %f", wz);

            geometry_msgs::TwistStamped velMsg;
            velMsg.twist.linear.x = this -> vx;
            velMsg.twist.linear.y = this -> vy;
            velMsg.twist.angular.z = this -> wz;
            this -> velocitiesPub.publish(velMsg);
        }

        ++numMsg;

    }
private:
	ros::NodeHandle n;
	ros::Subscriber sensorInput;
	ros::Publisher velocitiesPub;
    const int gearRatio = 5;
    const double wheelRadius = 0.07;
    const double halfLength = 0.2;
    const double halfWidth = 0.169;
    double wfl, wfr, wrr, wrl;
    uint32_t ticfl, ticfr, ticrr, ticrl;
    uint32_t ticfl0, ticfr0, ticrr0, ticrl0;
    double tswfl, tswfr, tswrr, tswrl;
    double tswfl0, tswfr0, tswrr0, tswrl0;
    double vx, vy, wz;
    int numMsg = 0;
    const int intMsg = 5;
};

int main (int argc, char **argv) {
	ros::init(argc, argv, "ComputeVelocitiesTick");
    ComputeVelocity compVel;
    compVel.mainLoop();
}