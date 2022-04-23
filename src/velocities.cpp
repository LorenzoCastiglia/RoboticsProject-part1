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
        ROS_INFO("Geometry node started\n");
        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }
        
    }
    void sensorCallback(const sensor_msgs::JointState::ConstPtr& msg){
        //size_t rot_size = msg->velocity.size();
        //double rot_spd[rot_size];
        //ROS_INFO("Dim: %d", rot_size);
        /*for (int i = 0; i<4; i++) {
            rot_spd[i] = msg->velocity[i]/(60*gearRatio);
            ROS_INFO("Rot spd %d: %f", i, rot_spd[i]);
        }

        for (int i = 0; i<4; i++) {
            ticks[i] = msg->position[i];
            ROS_INFO("Ticks %d: %f", i, ticks[i]);
        }*/
        w_fl = msg->velocity[0]/(60*gearRatio);
        ROS_INFO("Rot fl: %f", w_fl);
        w_fr = msg->velocity[1]/(60*gearRatio);
        ROS_INFO("Rot fr: %f", w_fr);
        w_rr = msg->velocity[2]/(60*gearRatio);
        ROS_INFO("Rot rr: %f", w_rr);
        w_rl = msg->velocity[3]/(60*gearRatio);
        ROS_INFO("Rot rl: %f", w_rl);

        v_x = (w_fl+w_fr+w_rr+w_rl)*wheelRadius/4;
        ROS_INFO("Vel x: %f", v_x);
        v_y = (-w_fl+w_fr+w_rr-w_rl)*wheelRadius/4;
        ROS_INFO("Vel y: %f", v_y);
        w_z = (-w_fl+w_fr-w_rr+w_rl)*wheelRadius/(4*(wheelRadius+halfLength));
        ROS_INFO("W z: %f", w_z);

        geometry_msgs::TwistStamped velMsg;
        velMsg.twist.linear.x = this -> v_x;
        velMsg.twist.linear.y = this -> v_y;
        velMsg.twist.angular.z = this -> w_z;
        this -> velocitiesPub.publish(velMsg);

    }
private:
	ros::NodeHandle n;
	ros::Subscriber sensorInput;
	ros::Publisher velocitiesPub;
    //double rot_spd[4];
    //double ticks[4];
    const int gearRatio = 5;
    const double wheelRadius = 0.07;
    const double halfLength = 0.2;
    const double halfWidth = 0.169;
    double w_fl, w_fr, w_rr, w_rl;
    double v_x, v_y, w_z;
};

int main (int argc, char **argv) {
	ros::init(argc, argv, "ComputeVelocities");
    ComputeVelocity compVel;
    compVel.mainLoop();
}