#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "project_1/Reset.h"

class ComputeOdometry {

public:

    ComputeOdometry() {
        
        this->velInput=this->n.subscribe("/cmd_vel", 1000, &ComputeOdometry::eulerOdometry, this);
        this-> resetService = this->n.advertiseService("reset", &ComputeOdometry::resetCallback, this);
                
        this->x0 = 0.0;
        this->y0 = 0.0;
        this->theta0 = 0.0;
        this->ts0 = 0.0;
    }

    void mainLoop(){
        ROS_INFO("Odometry node started\n");

        ros::spin();
        /*
        ros::Rate loop_rate(10);
        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }
        */
    }

    double computeTimeStamp(geometry_msgs::TwistStamped::ConstPtr &msg) {
        return msg->header.stamp.sec + msg->header.stamp.nsec * pow(10, -9);
    }

    void publishOdometry() {
        nav_msgs::Odometry odomMsg;
        odomMsg.pose.pose.position.x = this -> x0;
        odomMsg.pose.pose.position.y = this -> y0;
        odomMsg.pose.pose.orientation.z = this -> theta0;
        this -> odomPub.publish(odomMsg);
    }

    void eulerOdometry(const geometry_msgs::TwistStamped::ConstPtr &msg) {
        if(ts0 == 0.0) {
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

            // creating and publishing the odometry message
            publishOdometry();
            /*
            nav_msgs::Odometry odomMsg;
            odomMsg.pose.pose.position.x = this -> x;
            odomMsg.pose.pose.position.y = this -> y;
            odomMsg.pose.pose.orientation.z = this -> theta;
            this -> odomPub.publish(odomMsg);
            */
        }
    }

    void rungeKuttaOdometry(const geometry_msgs::TwistStamped::ConstPtr &msg) {
        // 2nd order Runge Kutta integration, theta changes during the translation

        if(ts0 == 0.0) {
            // When the first message is received, the new timestamp is saved but no
            // odometry is performed since we only have the first value of v and omega
            ts0 = msg->header.stamp.sec + msg->header.stamp.nsec * pow(10, -9);
        }
        else {
            // starting from the second message received, the odometry is computed
            ts = msg->header.stamp.sec + msg->header.stamp.nsec * pow(10, -9);
            deltats = ts - ts0;
            theta = theta0 + msg->twist.angular.z * deltats;
            v_k = sqrt(pow(msg->twist.linear.x,2) + pow(msg->twist.linear.y,2));
            x = x0 + v_k * deltats * cos(theta0 + (msg->twist.angular.z * deltats / 2));
            y = y0 + v_k * deltats * sin(theta0 + (msg->twist.angular.z * deltats / 2));

            // updating past values with current values
            theta0 = theta;
            x0 = x;
            y0 = y;
            ts0 = ts;

            // creating and publishing the odometry message
            publishOdometry();
        }
    }

    bool resetCallback(project_1::Reset::Request  &req, project_1::Reset::Response &res) {

        res.x_old = this->x0;
        res.y_old = this->y0;
        res.theta_old = this->theta0;

        this->x0 = req.x_new;
        this->y0 = req.y_new;
        this->theta0 = req.theta_new;

        publishOdometry();

        ROS_INFO("Old pose: (%lf,%lf%,lf) - New pose: (%lf,%lf%,lf)", 
            res.x_old, res.y_old, res.theta_old, req.x_new, req.y_new, req.theta_new);

        return true;
    }

private:

    ros::NodeHandle n;
	ros::Subscriber velInput;
    ros::Publisher odomPub;
    ros::ServiceServer resetService;
    double x, y, theta;
    double x0, y0, theta0;
    double ts0;
    double ts;
    double deltats;
    double v_k;             // absolute value of the linear velocity

};

int main (int argc, char **argv) {

    ros::init(argc, argv, "ComputeOdometry");

    ComputeOdometry compOdo;
    compOdo.mainLoop();

    return 0;
}