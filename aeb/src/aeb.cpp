#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include "std_msgs/String.h"
#include <string>
#include <cmath>

class AEBPlanner {
private:
    // A ROS node
    ros::NodeHandle n;

    // car parameters
    double max_speed, max_steering_angle, car_width, car_length;

    // Publish drive data
    ros::Publisher drive_pub;

    //Listens for Laser Scan
    ros::Subscriber scan_sub;

    // previous desired steering angle
    double prev_angle=0.0;

    //double minimunTTC = .22;
    double carRadius=0;
    double previousSpeed = 0;

public:
    AEBPlanner() {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        std::string drive_topic, scan_topic;
        n.getParam("custom_nav_drive_topic", drive_topic);
        n.getParam("scan_topic", scan_topic);
        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle);
        n.getParam("width", car_width);
        n.getParam("wheelbase", car_length);
        previousSpeed = max_speed;
        carRadius = sqrt(pow(car_length,2)+pow(car_width,2));
        // Make a publisher for drive messages
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
        
        scan_sub = n.subscribe(scan_topic, 1, &AEBPlanner::scan_callback, this);
        /* Way to output debug info
        std::string testString;
        testString = "test";
        ROS_INFO("%s",testString.c_str());
        */
    }
    /*only looks for single point in front of car
     Limited but can be simple to implement if having issues with other versions 
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
        float angleMin = msg->angle_min;
        float angleMax = msg->angle_max;
        float angleIncrement = msg->angle_increment;
        int index = static_cast<int>(angleMin*-1 / angleIncrement);
        float distance = msg->ranges.at(index);
        float timeToCollision = distance/max_speed;
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;
        drive_msg.steering_angle = 0;
        if(timeToCollision<0.5){
            drive_msg.speed =0;
        }else{
            drive_msg.speed =max_speed;
        }
        drive_st_msg.drive = drive_msg;
        drive_pub.publish(drive_st_msg);

    }
    */
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
        double timeToCollision = INFINITY;
        double angleForMinimumTTC = 0;
        for(int i =0; i< static_cast<int>(msg->ranges.size());i++){
            double absoluteAngle = i*msg->angle_increment + msg->angle_min;
            double relativeAngle = abs(absoluteAngle);
            if(relativeAngle <90){
                double timeToCollisionForArch =  abs((msg->ranges.at(i) - carRadius)) / abs(previousSpeed * cos(relativeAngle));
                if(timeToCollision > timeToCollisionForArch){
                    timeToCollision = timeToCollisionForArch;
                    angleForMinimumTTC = absoluteAngle;
                } 
            }
        }
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;
        drive_msg.steering_angle = 0;
        if(timeToCollision<0.22){
            drive_msg.speed =0;
        }// ADD movement adjuster here
        else if(timeToCollision<.5){ 
            if(angleForMinimumTTC < 0){
                drive_msg.steering_angle = angleForMinimumTTC + 90;
            }else{
                drive_msg.steering_angle = angleForMinimumTTC -90;
            }
            drive_msg.speed = 0.75;
        }
        else{
            drive_msg.speed =1;
        }
        std::string testString;
        testString = "Steering Angle: " + std::to_string(drive_msg.steering_angle) +" Speed: "+std::to_string(drive_msg.speed);
        ROS_INFO("%s",testString.c_str());
        previousSpeed = drive_msg.speed;
        if(previousSpeed < 1){
            previousSpeed = 1;
        }
        drive_st_msg.drive = drive_msg;
        drive_pub.publish(drive_st_msg);

    }
}; 

int main(int argc, char ** argv) {
    ros::init(argc, argv, "AEB_Planner");
    AEBPlanner rw;
    ros::spin();
    return 0;
}