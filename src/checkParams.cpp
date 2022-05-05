#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2/LinearMath/Matrix3x3.h"
class checkParams {

public:
    checkParams() {
        message_filters::Subscriber<sensor_msgs::JointState> wheelSub(n, "/wheel_states", 1);
        message_filters::Subscriber<geometry_msgs::PoseStamped> poseSub(n, "/robot/pose", 1);
        message_filters::TimeSynchronizer<geometry_msgs::PoseStamped, sensor_msgs::JointState> sync (poseSub, wheelSub, 10);
        sync.registerCallback(boost::bind(&checkParams::bagCallback, this, _1, _2));
        this->x0 = 0.0;
        this->y0 = 0.0;
        this->theta0 = 0.0;
        this->msgCount = -1;
        n.getParam("/gearRatio", this->gearRatio);
        n.getParam("/wheelRadius", this->wheelRad);
        n.getParam("/halfLenght", this->halfLength);
        n.getParam("/halfWidth", this->halfWidth);
        n.getParam("/tickRes", this->tickRes);
    }

    void mainLoop(){
        ROS_INFO("Param node started\n");

        ros::spin();
        /*
        ros::Rate loop_rate(10);
        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }
        */
    }

    void bagCallback(const geometry_msgs::PoseStamped::ConstPtr& msg1, const sensor_msgs::JointState::ConstPtr& msg2) {
        ROS_INFO("Callback");
        if(this->msgCount == -1) {
            ts0 = msg1->header.stamp;
            this -> x0 = msg1 -> pose.position.x;
            this -> y0 = msg1 -> pose.position.y;
            quat_msg = msg1 -> pose.orientation;
            tf2::fromMsg(quat_msg, q);
            tf2::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
            this -> theta0 = yaw;

            this -> ticfl0 = msg2->position[0];
            this -> ticfr0 = msg2->position[1];
            this -> ticrl0 = msg2->position[2];
            this -> ticrr0 = msg2->position[3];
        }
        else{
            this -> x = msg1 -> pose.position.x;
            this -> y = msg1 -> pose.position.y;
            quat_msg = msg1 -> pose.orientation;
            tf2::fromMsg(quat_msg, q);
            tf2::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
            this -> theta = yaw;
            this -> ts = msg1 -> header.stamp;

            this -> deltats = ts - ts0;
            this -> wz = (theta - theta0)/deltats.toSec();
            this -> vx = (sin(theta0)*(y-y0)+cos(theta0)*(x-x0))/deltats.toSec();
            this -> vy = (cos(theta0)*(y-y0)-sin(theta0)*(x-x0))/deltats.toSec();
            x0 = x;
            y0 = y;
            theta0 = theta;

            this -> ticfl = msg2->position[0];
            this -> ticfr = msg2->position[1];
            this -> ticrl = msg2->position[2];
            this -> ticrr = msg2->position[3];

            if (msgCount%2 == 0) {
                this -> wheelRad = (wz*deltats.toSec()*gearRatio*tickRes*(halfLength + halfWidth)*4)/(2*M_PI*(-(ticfl-ticfl0)+(ticfr-ticfr0)-(ticrl-ticrl0)+(ticrr-ticrr0)));
                ROS_INFO("Wheel radius: %f", wheelRad);
            }
            if (msgCount%2 == 1) {
                this -> tickRes = ((-(ticfl-ticfl0)+(ticfr-ticfr0)-(ticrl-ticrl0)+(ticrr-ticrr0))*2*M_PI*wheelRad)/(deltats.toSec()*gearRatio*wz*(halfLength + halfWidth)*4);
                ROS_INFO("Tick res: %f", tickRes);
            }
            ticfl0 = ticfl;
            ticfr0 = ticfr;
            ticrl0 = ticrl;
            ticrr0 = ticrr;
        }
        ++msgCount;
    }

    void wheelCallback(const geometry_msgs::PoseStamped::ConstPtr& msg1) {

    }

private:
    ros::NodeHandle n;
    ros::Subscriber wheelSub;
    ros::Subscriber poseSub;
    double x, y, theta;
    double x0, y0, theta0;
    double roll, pitch, yaw;
    double wz, vx, vy;
    ros::Time ts, ts0;
    ros::Duration deltats;
    tf2::Quaternion q;
    geometry_msgs::Quaternion quat_msg;
    int msgCount;
    double wheelRad, halfLength, halfWidth, tickRes, gearRatio;
    double ticfl, ticfr, ticrr, ticrl;
    double ticfl0, ticfr0, ticrr0, ticrl0;
};

int main (int argc, char **argv) {
    ros::init(argc, argv, "checkParams");
    checkParams checkPar;
    checkPar.mainLoop();
    return 0;
}