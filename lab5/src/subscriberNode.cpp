#include <ros/ros.h>
#include <headerFiles/ScanMatcher.h>
// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>

// Subscribe to a topic with this message type
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

#include "std_msgs/String.h"
#include <string>
#include <cmath>
class AEBWalker {
private:
    // A ROS node
    ros::NodeHandle n;

    // car parameters
    double max_speed, max_steering_angle;

    // Publish drive data
    ros::Publisher drive_pub;

    //Listens for Laser Scan
    ros::Subscriber scan_sub;
    
    laserData lastMessage;
public:
    float Kp = 5;
    float Ki = 0;
    float Kd = 0.5;
    float previousError = 0;
    float totalError =0;
    bool firstCall = true;
    float previousVelocity =0;
    ros::Time before =ros::Time::now();
    ros::Time now = ros::Time::now();
    const float desiredDistance = .7;
    AEBWalker() {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // get topic names
        std::string drive_topic, scan_topic;
        std::string testString;
        testString = "THISISATEST";
        ROS_INFO("TESTING");
        ROS_INFO("%s",testString.c_str());
        n.getParam("custom_nav_drive_topic", drive_topic);
        n.getParam("scan_topic", scan_topic);
        ROS_INFO("%s",drive_topic.c_str());
        // get car parameters
        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle);
        
        // Make a publisher for drive messages
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
        
        //SHOULDNT NEED THE FOLLOWING CODE - Kept as backup
        // Start a subscriber to listen to odom messages
        //odom_sub = n.subscribe(odom_topic, 1, &AEBWalker::odom_callback, this);

        scan_sub = n.subscribe(scan_topic, 1, &AEBWalker::scan_callback, this);
        std::vector<float> emptyArray;
        lastMessage.ranges = emptyArray;
    }
    //only looks for single point in front of car. can be limited by 
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
        transformation newBestGuess;
        newBestGuess.angle =0;
        newBestGuess.xDistance =0;
        newBestGuess.yDistance =0;
        transformation lastBestGuess;
        lastBestGuess.angle=0;
        lastBestGuess.xDistance=0;
        lastBestGuess.yDistance=0;
        bool firstIteration =true;
        laserData newScan;
        newScan.angle_increment = msg->angle_increment;
        newScan.angle_max = msg->angle_max;
        newScan.angle_min = msg->angle_min;
        newScan.ranges = msg->ranges;
        if(lastMessage.ranges.size()!=0){
        //while((newBestGuess.angle!=lastBestGuess.angle && lastBestGuess.xDistance != newBestGuess.xDistance && newBestGuess.yDistance != lastBestGuess.yDistance) || firstIteration){
        for(int i =0;i<20;i++){
        firstIteration = false;
        newBestGuess = ScanMatcher::optimalTransformation(ScanMatcher::findCorrespondence(lastMessage,newScan,lastBestGuess),lastBestGuess); 
        lastBestGuess.angle = newBestGuess.angle;
        lastBestGuess.xDistance=newBestGuess.xDistance;
        lastBestGuess.yDistance = newBestGuess.yDistance;
        }
        }
         ROS_INFO("%s",((std::string)("The optimal transformation ")).c_str());
         ROS_INFO("%s",(std::to_string(newBestGuess.angle)+ " ").c_str());
         ROS_INFO("%s",(std::to_string(newBestGuess.xDistance)+ " ").c_str());
         ROS_INFO("%s",(std::to_string(newBestGuess.yDistance)+ " \n").c_str());
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;
        // 25 degree angle
        float angleBetween = 0.436332;
        //90 degree angle
        int indexOfPoint2 = static_cast<int>(((0-(M_PI / 2)) - msg->angle_min)/msg->angle_increment);
        int indexOfPoint1 = static_cast<int>(((0-(M_PI / 2)+angleBetween) - msg->angle_min)/msg->angle_increment);
        float point1 = msg->ranges.at(indexOfPoint1);
        float point2 = msg->ranges.at(indexOfPoint2);
        SteeringData driveInfo = getInfo(point1,point2,angleBetween);
        
        
        drive_msg.steering_angle = driveInfo.steering_angle;
        drive_msg.speed = driveInfo.speed;
        drive_st_msg.drive = drive_msg;
        drive_pub.publish(drive_st_msg);
        lastMessage = newScan;

    }
    float getCarAngleFromWall(float point1, float point2, float angleBetween){
        return std::atan((point1*std::cos(angleBetween)-point2)/(point1*std::sin(angleBetween)));
    }
    float getCarDistanceFromWall(float point1, float point2, float angleBetween){
        return point2 * std::cos(getCarAngleFromWall(point1,point2,angleBetween));
    }
    struct SteeringData{
        float steering_angle;
        float speed;
    };
    SteeringData getInfo(float point1, float point2, float angleBetween){
        SteeringData info;
        info.steering_angle = 0;
        info.speed=0;
        before = now;
        now = ros::Time::now();
        //ROS_INFO("Point 1: %s",std::to_string(point1).c_str());
        //ROS_INFO("Point 2: %s",std::to_string(point2).c_str());
        float angle = getCarAngleFromWall(point1,point2,angleBetween);
        float distance = getCarDistanceFromWall(point1,point2,angleBetween);
        //ROS_INFO("Angle From Wall: %s",std::to_string(angle).c_str());
        //ROS_INFO("Distance to Wall: %s",std::to_string(distance).c_str());
        float desiredSteeringAngle = Kp*currentError(distance,angle) + Ki * integral(distance) + Kd* derivative(distance,angle);
        info.steering_angle = desiredSteeringAngle;
        if(std::abs(info.steering_angle)<10){
            info.speed = 1.5;
        }
        else if(std::abs(info.steering_angle)<20){
            info.speed = 1;
        }
        else{
            info.speed = 0.5;
        }
        previousVelocity = info.speed;
        previousError = currentError(distance,angle);
        ROS_INFO("Steering Angle: %s",std::to_string(info.steering_angle).c_str());
        return info;
    }
    float currentError(float distance, float angle){
        return desiredDistance - (distance + previousVelocity* (now.toSec()-before.toSec()) *std::cos(angle));
    }
    float integral(float distance){
        totalError += distance * (now.toSec()- before.toSec());
        return totalError;
    }
    float derivative(float distance, float angle){
        float currentDerivative = (currentError(distance,angle) - previousError) / (now.toSec()-before.toSec());
        return currentDerivative;
    }
    //default PID configuration kp = 14 ki =0 kd = 0.09

}; // end of class definition

int main(int argc, char ** argv) {
    ros::init(argc, argv, "AEB_walker");
    AEBWalker rw;
    ros::spin();
    return 0;
}