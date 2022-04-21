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
        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }
        
    }
    void sensorCallback(const sensor_msgs::JointState::ConstPtr& msg){
        for (int i = 0; i<4; i++) {
            rot_spd[i] = msg->velocity[i];
        }
    }
private:
	ros::NodeHandle n;
	ros::Subscriber sensorInput;
	ros::Publisher velocitiesPub;
    double rot_spd[4];
};

int main (int argc, char **argv) {
	ros::init(argc, argv, "ComputeVelocities");
    ComputeVelocity compVel;
    compVel.mainLoop();
    
}