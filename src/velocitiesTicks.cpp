#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"

class ComputeVelocity {

public:

	ComputeVelocity(){
		this->sensorInput=this->n.subscribe("/wheel_states", 1000, &ComputeVelocity::sensorCallback,this);
		this->velocitiesPub=this->n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1000);

        this->ticfl0=0.0;
        this->ticfr0=0.0;
        this->ticrr0=0.0;
        this->ticrl0=0.0;
        this->ts0=0.0;
	}

    void mainLoop(){
        ROS_INFO("Geometry node started\n");

        ros::spin();
        /*
        ros::Rate loop_rate(10);
        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }
        */
    }
    
    void sensorCallback(const sensor_msgs::JointState::ConstPtr& msg){
        
        if(msgCount%msgInterval == 0){
            // The velocities are computed every msgInterval number of messages received
            // to reduce noise

            if(msgCount == 0) {
                // When the first message is received, the first values are saved but
                // the velocities are not computed

                ts0 = msg->header.stamp.sec + msg->header.stamp.nsec * pow(10, -9);

                ticfl0 = msg->position[0]/gearRatio;
                ticfr0 = msg->position[1]/gearRatio;
                ticrl0 = msg->position[2]/gearRatio;
                ticrr0 = msg->position[3]/gearRatio;
            }
            else {
                // Starting from the second iteration, velocities are computed and published

                ts = msg->header.stamp.sec + msg->header.stamp.nsec * pow(10, -9);

                // front-left wheel
                ticfl = msg->position[0]/gearRatio;
                wfl = (ticfl - ticfl0)/(ts - ts0);
                ROS_INFO("Rot fl: %f", wfl);
                ticfl0 = ticfl;

                // front-right wheel
                ticfr = msg->position[1]/gearRatio;
                wfr = (ticfr - ticfr0)/(ts - ts0);
                ROS_INFO("Rot fr: %f", wfr);
                ticfr0 = ticfr;

                // rear-left wheel
                ticrl = msg->position[2]/gearRatio;
                wrl = (ticrl - ticrl0)/(ts - ts0);
                ROS_INFO("Rot rl: %f", wrl);
                ticrl0 = ticrl;

                // rear-right wheel
                ticrr = msg->position[3]/gearRatio;
                wrr = (ticrr - ticrr0)/(ts - ts0);
                ROS_INFO("Rot rr: %f", wrr);
                ticrr0 = ticrr;

                ts0 = ts;

                // robot velocities
                vx = (wfl+wfr+wrr+wrl)*wheelRadius/4;
                ROS_INFO("Vel x: %f", vx);
                vy = (-wfl+wfr+wrl-wrr)*wheelRadius/4;
                ROS_INFO("Vel y: %f", vy);
                wz = (-wfl+wfr-wrl+wrr)*wheelRadius/(4*(wheelRadius+halfLength));
                ROS_INFO("W z: %f", wz);

                // publishing cmd_vel message
                geometry_msgs::TwistStamped velMsg;
                velMsg.twist.linear.x = this -> vx;
                velMsg.twist.linear.y = this -> vy;
                velMsg.twist.angular.z = this -> wz;
                this -> velocitiesPub.publish(velMsg);
            }
            
        }

        ++msgCount;

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
    double ts;
    double ts0;
    double vx, vy, wz;
    int msgCount = 0;
    const int msgInterval = 5;

};

int main (int argc, char **argv) {
    
	ros::init(argc, argv, "ComputeVelocitiesTick");

    ComputeVelocity compVel;
    compVel.mainLoop();

    return 0;
}