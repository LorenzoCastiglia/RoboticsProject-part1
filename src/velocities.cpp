#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"

class ComputeVelocity {
public:
	ComputeVelocity(){
		this->sensorInput=this->n.subscribe("/wheel_states", 1000, &ComputeVelocity::sensorCallback,this);
		this->velocitiesPub=this->n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1000);

        n.getParam("/gearRatio", this->gearRatio);
        n.getParam("/wheelRadius", this->wheelRadius);
        n.getParam("/halfLenght", this->halfLength);
        n.getParam("/halfWidth", this->halfWidth);
        //n.getParam("/tickRes", this->tickResolution);
	}
    void mainLoop(){
        ros::Rate loop_rate(10);
        ROS_INFO("Geometry node started\n");
        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }
        
    }
    void sensorCallback(const sensor_msgs::JointState::ConstPtr& msg){
        wfl = msg->velocity[0]/(60*gearRatio);
        ROS_INFO("Rot fl: %f", wfl);
        wfr = msg->velocity[1]/(60*gearRatio);
        ROS_INFO("Rot fr: %f", wfr);
        wrl = msg->velocity[2]/(60*gearRatio);
        ROS_INFO("Rot rl: %f", wrl);
        wrr = msg->velocity[3]/(60*gearRatio);
        ROS_INFO("Rot rr: %f", wrr);

        vx = (wfl+wfr+wrl+wrr)*wheelRadius/4;
        ROS_INFO("Vel x: %f", vx);
        vy = (-wfl+wfr+wrl-wrr)*wheelRadius/4;
        ROS_INFO("Vel y: %f", vy);
        wz = (-wfl+wfr-wrl+wrr)*wheelRadius/(4*(wheelRadius+halfLength));
        ROS_INFO("W z: %f", wz);

        geometry_msgs::TwistStamped velMsg;
        velMsg.header.stamp.sec = msg->header.stamp.sec;
        velMsg.header.stamp.nsec = msg -> header.stamp.nsec;
        velMsg.twist.linear.x = this -> vx;
        velMsg.twist.linear.y = this -> vy;
        velMsg.twist.angular.z = this -> wz;
        this -> velocitiesPub.publish(velMsg);

    }
private:
	ros::NodeHandle n;
	ros::Subscriber sensorInput;
	ros::Publisher velocitiesPub;
    int gearRatio = 5;
    double wheelRadius = 0.07;
    double halfLength = 0.2;
    double halfWidth = 0.169;
    double wfl, wfr, wrl, wrr;
    double vx, vy, wz;
};

int main (int argc, char **argv) {
	ros::init(argc, argv, "ComputeVelocities");
    ComputeVelocity compVel;
    compVel.mainLoop();
}