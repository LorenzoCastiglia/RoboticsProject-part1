#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class tfBroadcast
{
public:
    tfBroadcast(/* args */){
        this -> odomSub = n.subscribe("/odom", 1000, &tfBroadcast::transformCallback, this);
    }

    void transformCallback(const nav_msgs::Odometry::ConstPtr &msg){
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = msg->pose.pose.position.x;
        transformStamped.transform.translation.y = msg->pose.pose.position.y;
        transformStamped.transform.translation.y = msg->pose.pose.position.z;
        transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
        transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
        transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
        transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;

        toBaseLBr.sendTransform(transformStamped);

    }
private:
    ros::NodeHandle n;
    ros::Subscriber odomSub;
    tf2_ros::TransformBroadcaster toBaseLBr;
    geometry_msgs::TransformStamped transformStamped;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "tfBroadcast");
  tfBroadcast tfBroadcaster;
  ros::spin();
  return 0;
}