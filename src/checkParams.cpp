#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

class CheckParams
{
public:
    /*CheckParams() {
        this -> poseSub = n.subscribe("/robot/pose", 1000, &ComputeOdometry::callOdometryMethod, this);
    }*/
private:
    ros::NodeHandle n;
    ros::Subscriber poseSub;
    ros::Subscriber sensorSub;
};

int main () {

}