#include <headerFiles/ScanMatcher.h>
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

    transformation ScanMatcher::optimalTransformation(std::vector<correspondence> correspondenceVector,transformation bestGuess ){
        transformation optimalTransformation;
        Eigen::MatrixXd A(static_cast<int>(correspondenceVector.size()),3);
        Eigen::VectorXd B(static_cast<int>(correspondenceVector.size()));
        for(int i =0;i<static_cast<int>(correspondenceVector.size());i++){
            point n = ScanMatcher::normalToLine(correspondenceVector.at(i).pointToMatchToLine,correspondenceVector.at(i).closestPoint1,correspondenceVector.at(i).closestPoint2);
            point d = correspondenceVector.at(i).closestPoint1;
            point s = rotatedPoint(correspondenceVector.at(i).pointToMatchToLine,bestGuess);
            A(i,0) = n.y * s.x - n.x * s.y;
            A(i,1) = n.x;
            A(i,2) = n.y;
            B(i) =n.x *d.x + n.y*d.y - n.x *s.x - n.y*s.y;
        }
        Eigen::VectorXd solution(3); 
        solution = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
        optimalTransformation.angle = solution(0);
        optimalTransformation.xDistance = solution(1);
        optimalTransformation.yDistance = solution(2);
       
        return optimalTransformation;
    }
    std::vector<correspondence> ScanMatcher::findCorrespondence(laserData oldScan, laserData newScan, transformation currentGuess){
       
        //puts coordinates from new scan into the fram of the old scan so we can compare distances
        std::vector<point> transformedPointsFromNewScan(static_cast<int>(newScan.ranges.size()));
        for(int i=0;i<static_cast<int>(newScan.ranges.size());i++){
            double xInOldFrame = newScan.ranges.at(i) * ScanMatcher::estCos(newScan.angle_min+i*newScan.angle_increment);
            double yInOldFrame = newScan.ranges.at(i) * ScanMatcher::estSin(newScan.angle_min+i*newScan.angle_increment);
            point newPoint;
            // this should be verified to make sure this is a correct coordinate transformation
            newPoint.x = ScanMatcher::estCos(currentGuess.angle)*xInOldFrame - ScanMatcher::estSin(currentGuess.angle)*yInOldFrame + currentGuess.xDistance;
            newPoint.y = ScanMatcher::estSin(currentGuess.angle)*xInOldFrame + ScanMatcher::estCos(currentGuess.angle)*yInOldFrame + currentGuess.yDistance;
            transformedPointsFromNewScan.push_back(newPoint);
        }
        //Puts old scan into x and y format instead of angle format
        std::vector<point> oldScanXY(static_cast<int>(oldScan.ranges.size()));
        for(int i=0;i<static_cast<int>(newScan.ranges.size());i++){
            point newPoint;
            newPoint.x = oldScan.ranges.at(i) * ScanMatcher::estCos(oldScan.angle_min+i*oldScan.angle_increment);
            newPoint.y = oldScan.ranges.at(i) * ScanMatcher::estSin(oldScan.angle_min+i*oldScan.angle_increment);
            oldScanXY.push_back(newPoint);
        }
        // Formulate Jump Table
        std::vector<jumpValues> jumpTable(static_cast<int>(oldScan.ranges.size()));
        for(int i =0;i<static_cast<int>(oldScan.ranges.size());i++){
            jumpValues currentJump;
            int sizeOfScan = static_cast<int>(oldScan.ranges.size());
            int j;
            j=i+1;
            (currentJump).upBigger=j-i;
            while(j<sizeOfScan && oldScan.ranges.at(j)<=oldScan.ranges.at(i)){
                j++;
                (currentJump).upBigger= j-i;
            }
            j=i+1;
            (currentJump).upSmaller = j-i;
            while(j<sizeOfScan && oldScan.ranges.at(j)>=oldScan.ranges.at(i)){
                j++;
                (currentJump).upSmaller= j-i;
            }
            j=i-1;
            (currentJump).downBigger =j-i;
            while(j>=0 && oldScan.ranges.at(j)<=oldScan.ranges.at(i)){
                j--;
                (currentJump).downBigger= j-i;
            }
            j=i-1;
            (currentJump).downSmaller=j-i;
            while(j>=0 && oldScan.ranges.at(j)>=oldScan.ranges.at(i)){
                j--;
                (currentJump).downSmaller= j-i;
            }
            jumpTable.at(i)=(currentJump);
        }
        jumpValues test1 = jumpTable.at(0);
        jumpValues test2 = jumpTable.at(1);

        // START OF MATCHING ALGORITH 
        std::vector<correspondence> matches;
        int indexOfLastBestGuess = -1;
        for(int i=0;i<static_cast<int>(transformedPointsFromNewScan.size());i++){
            point currentPoint = transformedPointsFromNewScan.at(i);
            int bestIndex = i;
            int bestDistance =  sqrt(pow(oldScanXY.at(i).x-currentPoint.x,2)+pow(oldScanXY.at(i).y-currentPoint.y,2));
            double estimatedStartAngle = ScanMatcher::estACos(currentPoint.x /(sqrt(currentPoint.x*currentPoint.x+currentPoint.y*currentPoint.y)));
            if(currentPoint.x ==0 && currentPoint.y == 0){
                estimatedStartAngle = oldScan.angle_min + oldScan.angle_increment * i;
            }
            if(indexOfLastBestGuess == -1){
            indexOfLastBestGuess = static_cast<int>((estimatedStartAngle-newScan.angle_min)/newScan.angle_increment)-1;
            }
            if(indexOfLastBestGuess<0){
                indexOfLastBestGuess = 0;
            }
            indexOfLastBestGuess++;
            int upIndex = indexOfLastBestGuess+1;
            if(upIndex>=static_cast<int>(oldScanXY.size())){
                upIndex =static_cast<int>(oldScanXY.size())-1;
            }
            int downIndex = indexOfLastBestGuess;
             if(downIndex>=static_cast<int>(oldScanXY.size())){
                downIndex =static_cast<int>(oldScanXY.size())-1;
            }
            //check these two doubles to make sure they were correct. Infinity wouldnt worry so this is what it should be
            double lastDownDistance = sqrt(pow(oldScanXY.at(downIndex).x-currentPoint.x,2)+pow(oldScanXY.at(downIndex).y-currentPoint.y,2));
            double lastUpDistance = sqrt(pow(oldScanXY.at(upIndex).x-currentPoint.x,2)+pow(oldScanXY.at(upIndex).y-currentPoint.y,2));
            bestDistance = lastUpDistance;
            bool upStopped = false;
            bool downStopped = false;
            int maxPossibleIndex= static_cast<int>(oldScan.ranges.size())-1;
            int minPossibleIndex =0;
            while(!(upStopped && downStopped)){
                bool nowUp = downStopped || (!upStopped && (lastUpDistance <= lastDownDistance));
                if(nowUp){
                    if(upIndex>=static_cast<int>(oldScan.ranges.size()) || upIndex >maxPossibleIndex || upIndex<0){
                        upStopped=true;
                        continue;
                    }
                    lastUpDistance = sqrt(pow(oldScanXY.at(upIndex).x-currentPoint.x,2)+pow(oldScanXY.at(upIndex).y-currentPoint.y,2));
                    if(lastUpDistance<bestDistance){
                        bestIndex=upIndex;
                        bestDistance = lastUpDistance;
                        if(currentPoint.x == 0 && currentPoint.y ==0){
                            maxPossibleIndex = estimatedStartAngle;
                            minPossibleIndex = estimatedStartAngle;
                        }else{
                            maxPossibleIndex = estimatedStartAngle + ScanMatcher::estATan(bestDistance/( sqrt(currentPoint.x*currentPoint.x+currentPoint.y*currentPoint.y) ));
                            minPossibleIndex = estimatedStartAngle - ScanMatcher::estATan(bestDistance/( sqrt(currentPoint.x*currentPoint.x+currentPoint.y*currentPoint.y) ));
                        }
                    }
                    // if distance from origin of point up < point current
                    if(oldScan.ranges.at(upIndex)<sqrt(currentPoint.x*currentPoint.x+currentPoint.y*currentPoint.y)){
                        int test = jumpTable.at(upIndex).upBigger;
                        upIndex = upIndex +jumpTable.at(upIndex).upBigger;
                    }else{
                         int test = jumpTable.at(upIndex).upSmaller;
                        upIndex = upIndex +jumpTable.at(upIndex).upSmaller;
                    }
                }else{ // when Searching down
                    if(downIndex<=0 || downIndex > minPossibleIndex){
                        downStopped=true;
                        continue;
                    }
                    lastDownDistance = sqrt(pow(oldScanXY.at(downIndex).x-currentPoint.x,2)+pow(oldScanXY.at(downIndex).y-currentPoint.y,2));
                    if(lastDownDistance<bestDistance){
                        bestIndex=downIndex;
                        bestDistance = lastDownDistance;
                        if(currentPoint.x ==0 && currentPoint.y ==0){
                            maxPossibleIndex = estimatedStartAngle;
                            minPossibleIndex = estimatedStartAngle;
                        }else{
                            maxPossibleIndex = estimatedStartAngle + ScanMatcher::estATan(bestDistance/( sqrt(currentPoint.x*currentPoint.x+currentPoint.y*currentPoint.y) ));
                            minPossibleIndex = estimatedStartAngle - ScanMatcher::estATan(bestDistance/( sqrt(currentPoint.x*currentPoint.x+currentPoint.y*currentPoint.y) ));
                        }
                    }
                    // if distance from origin of point up < point current
                    if(oldScan.ranges.at(downIndex)<sqrt(currentPoint.x*currentPoint.x+currentPoint.y*currentPoint.y)){
                         int test = jumpTable.at(downIndex).downBigger;
                        downIndex =downIndex + jumpTable.at(downIndex).downBigger;
                    }else{
                         int test = jumpTable.at(downIndex).downSmaller;
                        downIndex =downIndex + jumpTable.at(downIndex).downSmaller;
                    }
                }
            }// Best Point Should be found at end of this loop
            correspondence newMatch;
            newMatch.pointToMatchToLine = currentPoint;
            newMatch.closestPoint1 =oldScanXY.at(bestIndex);
            if(bestIndex ==0){
                newMatch.closestPoint2 = oldScanXY.at(1);
            }else{
                newMatch.closestPoint2 = oldScanXY.at(bestIndex-1);
            }
            matches.push_back(newMatch);
        }
        // END OF MATCHING ALGORITH
        return matches;
    }
double ScanMatcher::estCos(double angle){
    return cos(angle);//switch to taylor polynomail for efficiency
}
double ScanMatcher::estSin(double angle){
    return cos(angle);//switch to taylor polynomail for efficiency
}
double ScanMatcher::estACos(double value){
    return acos(value);
}
double ScanMatcher::estATan(double value){
    return atan(value);
}
point ScanMatcher::normalToLine(point pointToMeasure,point pointOnLine1,point PointOnLine2){
    point ViDi, ViSi, unitNormalVector;
    ViDi.x = pointOnLine1.x-PointOnLine2.x;
    ViDi.y = pointOnLine1.y-PointOnLine2.y;
    ViSi.x = pointToMeasure.x-PointOnLine2.x;
    ViSi.y = pointToMeasure.y-PointOnLine2.y;
    
    double dotProductOfViDiViSi = ViDi.x * ViSi.x + ViDi.y * ViSi.y;
    double lengthOfViDi = sqrt(ViDi.x*ViDi.x+ViDi.y*ViDi.y);
    if(lengthOfViDi == 0){
        unitNormalVector.x =0;
        unitNormalVector.y =0;
        return unitNormalVector;
    }
    double xOfNormalVector = ViSi.x - ViDi.x * (dotProductOfViDiViSi / lengthOfViDi);
    double yOfNormalVector = ViSi.y - ViDi.y * (dotProductOfViDiViSi / lengthOfViDi);
    
    double lengthOfNormalVector = sqrt(xOfNormalVector * xOfNormalVector + yOfNormalVector * yOfNormalVector);
    if(lengthOfNormalVector == 0){
        unitNormalVector.x =0;
        unitNormalVector.y =0;
        return unitNormalVector;
    }
    unitNormalVector.x = xOfNormalVector / lengthOfNormalVector;
    unitNormalVector.y = yOfNormalVector / lengthOfNormalVector;
    return unitNormalVector;
}
point ScanMatcher::rotatedPoint(point pointToRotate, transformation currentTransform){
        point newPoint;
        newPoint.x = ScanMatcher::estCos(currentTransform.angle) * pointToRotate.x - ScanMatcher::estSin(currentTransform.angle) * pointToRotate.y + currentTransform.xDistance;
        newPoint.y = ScanMatcher::estSin(currentTransform.angle) * pointToRotate.x + ScanMatcher::estCos(currentTransform.angle) * pointToRotate.y + currentTransform.yDistance;
        return newPoint;
}