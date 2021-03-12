#ifndef SCANMATCHER_H_
#define SCANMATCHER_H_
#include <ros/ros.h>
// Subscribe to a topic with this message type
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

#include "std_msgs/String.h"
#include <string>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <sstream>
struct point;
struct correspondence;
struct transformation;
struct jumpValues;
struct laserData;
class ScanMatcher{
    public:
     static transformation optimalTransformation(std::vector<correspondence> correspondenceVector, transformation bestGuess);
     static std::vector<correspondence> findCorrespondence(laserData oldScan, laserData newScan, transformation currentGuess);
    static double estCos(double angle);
    static double estSin(double angle);
    static double estACos(double value);
    static double estATan(double value);
    static point normalToLine(point pointToMeasure,point pointOnLine1,point PointOnLine2);
    static point rotatedPoint(point pointToRotate,transformation currentTransform);
};
struct point{
    double x;
    double y;
};
struct correspondence{
    point pointToMatchToLine;
    point closestPoint1;
    point closestPoint2;
};
struct transformation{
    double angle;
    double xDistance;
    double yDistance;        
};
struct jumpValues{
    int upBigger;
    int upSmaller;
    int downBigger;
    int downSmaller;
};
struct laserData{
    float angle_min;
    float angle_max;
    float angle_increment;
    std::vector<float> ranges;
    std::vector<float> intensities;
};

#endif