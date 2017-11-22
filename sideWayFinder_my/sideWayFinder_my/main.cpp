//
//  main.cpp
//  sideWayFinder_my
//
//  Created by 张青青 on 11/21/17.
//  Copyright © 2017 张青青. All rights reserved.
//

//#include <iostream>
//
//int main(int argc, const char * argv[]) {
//    // insert code here...
//    std::cout << "Hello, World!\n";
//    return 0;
//}

#include <vector>
#include <math.h>

#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"

#include "control/sideWay.h"
#include "control/angleDistanceError.h"
#include "control/pid_input.h"

#define dataSize (1081)
#define validSize (150)
#define ratio (7.525)    //link the simulation world to real world

ros::Publisher pub, pubError, pub2Python;

typedef struct
{
    double x,y;
} Point;


Point convertCoord(double range,int index)    //in vehicle coordinate system
{
    double theta = (-135+index*0.25)*3.14159265358979323846264/180.0;    // deg to rad for math
    Point ans;
    ans.x = range*cos(theta);
    ans.y = range*sin(theta);
    return ans;
}

inline double distanceOfPoints(Point A,Point B)
{
    return sqrt(    (A.x-B.x)*(A.x-B.x)  +  (A.y-B.y)*(A.y-B.y)    );
}

void linearFit(std::vector<Point> P,double &A,double &B)
{
    std::vector<Point>::iterator it;
    double Sxy=0, Sx=0, Sy=0, Sxx=0, N;
    N=P.size();
    A=0;
    B=0;
    for (it=P.begin();it!=P.end();it++)
    {
        Sxy += (it -> x) * (it -> y);
        Sx  += it -> x;
        Sy  += it -> y;
        Sxx += (it -> x) * (it -> x);
    }
    A = (Sxy-Sx*Sy/N)/(Sxx-Sx*Sx/N);
    B = Sy/N-A*Sx/N;
    //    ROS_INFO("Xave:%lf Yave:%lf A:%lf B:%lf C:%lf",Xave,Yave,A,B,C);
}

void calculateAngleDistance(double A,double B,double &ang, double &dist)    //A = tan(ang), B = r, ang in degree
{
    ang = -atan(A);
    dist = cos(ang)*B;
    ang = ang*180/3.14159265;    //rad to deg
}

void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    control::sideWay side;
    control::angleDistanceError error;
    control::pid_input pid_error;
    /////////////////////////////////Wall
    std::vector<Point> rightWall,leftWall;
    leftWall.clear();    //clear data in the vector
    rightWall.clear();
    int outrange_count = 0;
    Point lastPoint, currentPoint;
    for (int i = 1; i < dataSize; i++)    //right wall detection
    {
        if (msg->ranges[i] > 4 * ratio && msg->ranges[i] < 100 && rightWall.size() > 0)
            outrange_count++;
        
        if (outrange_count>20)
            break;
        
        if (msg->ranges[i] < 3 * ratio)
        {
            currentPoint = convertCoord(msg->ranges[i], i);
            if (rightWall.size() > 0 && distanceOfPoints(currentPoint, lastPoint) > 0.3 * ratio)
                break;
            rightWall.push_back(currentPoint);    //add data to the vector
            lastPoint = currentPoint;
        }
    }
    
    outrange_count=0;
    
    for (int i = dataSize - 1; i > 1; i--)    //left wall detection
    {
        if (msg->ranges[i] > 4 * ratio && msg->ranges[i] < 100 && leftWall.size() > 0)
            outrange_count++;
        
        if (outrange_count > 20)
            break;
        
        if (msg->ranges[i] < 3 * ratio)    //only care object in 3m
        {
            currentPoint = convertCoord(msg->ranges[i], i);
            if (leftWall.size() > 0 && distanceOfPoints(currentPoint, lastPoint) > 0.3 * ratio)    //check the continuity
                break;
            leftWall.push_back(currentPoint);
            lastPoint = currentPoint;
        }
    }
    
    linearFit(leftWall,  side.LA, side.LB);
    linearFit(rightWall, side.RA, side.RB);
    
    double Lang,Rang,Ldist,Rdist;
    calculateAngleDistance(side.LA, side.LB, Lang, Ldist);
    calculateAngleDistance(side.RA, side.RB, Rang, Rdist);
    /////////////////////////////////
    int count_obst = 0;
    int flg_obst = 0;
    
    int count_obst1 = 0;
    int flg_obst1 = 0;
    
    double mid=(msg->ranges[540]+msg->ranges[541]+msg->ranges[539])/3/ratio;
    
    for (int i = 540 - (20 * 4); i < 540 + (20 * 4); i++)    //mid_rarrow
    {
        if (msg->ranges[i] < 0.7 * ratio)
            count_obst++;
        if (count_obst >= 40)
        {
            flg_obst = 1;
            break;
        }
        else
            flg_obst = 0;
    }
    
    for (int i = 540 - (70 * 4); i < 540 + (70 * 4); i++)    //mid_wide
    {
        if (msg->ranges[i] < 0.5 * ratio)
            count_obst1++;
        if (count_obst1 >= 20)
        {
            flg_obst1 = 1;
            break;
        }
        else
            flg_obst1 = 0;
    }
    ///////////////////////////////////
    //Point lastPoint, currentPoint;
    Point gapR, gapL;
    int flg_cvR = 0;
    int flg_cvL = 0;
    
    lastPoint = convertCoord(msg->ranges[180], 180);
    for (int i = 180; i <  540; i++)                //right_side detection
    {
        currentPoint = convertCoord(msg->ranges[i], i);
        if (distanceOfPoints(currentPoint, lastPoint) > 0.2 * ratio)
        {
            flg_cvR = 1;
            gapR = currentPoint;
            ROS_INFO("gapPoint:(%lf,%lf)", gapR.x,gapR.y);
            break;
        }
        lastPoint = currentPoint;
    }
    
    lastPoint = convertCoord(msg->ranges[900], 900);
    for (int i = 900; i > 540; i--)                //left_side detection
    {
        currentPoint = convertCoord(msg->ranges[i], i);
        if (distanceOfPoints(currentPoint, lastPoint) > 0.2 * ratio)
        {
            flg_cvL = 1;
            gapL = currentPoint;
            ROS_INFO("gapPoint:(%lf,%lf)", gapL.x,gapL.y);
            break;
        }
        lastPoint = currentPoint;
    }
    //dist (-L)(+R)
    if (flg_cvR == 0 && flg_cvL == 1)    //make left turn
    {
        error.dist =  -0.3 * ratio + (-Rdist);;    //Go to Right Side Wall
        error.ang  =  Rang;
    }
    else if (flg_cvR == 1 && flg_cvL == 0)    //make right turn
    {
        error.dist =  (0.3 * ratio - Ldist);        //Go to Left Side Wall
        error.ang  =  Lang;
    }
    else
        error.dist = 0;
    ROS_INFO("!!!!!!!!!!??????????error.dist: %f", error.dist);
    //error.ang  =  0;
    pid_error.pid_vel = 50;
    
    //////////////
    if (flg_obst == 1 || flg_obst1 == 1)    //emergency stop
    {
        error.ang  =  0;
        error.dist =  0;
        pid_error.pid_vel = 0;
    }
    
    //ROS_INFO("obstacle:%d, Count:%d, mid:%f", flg_obst, count_obst, mid);    //Look obstacle
    ROS_INFO("cvL: %d, cvR: %d, distL: %f, distR: %f", flg_cvL, flg_cvR, msg->ranges[900], msg->ranges[180]);
    ROS_INFO("error.dist: %f", error.dist);
    
    //////////////////////////////////////////////////////////////////////////////////////
    pid_error.pid_error = (error.dist / ratio + error.ang / 45 * 1.5) * 100;
    
    pub.publish(side);
    pubError.publish(error);
    pub2Python.publish(pid_error);
}

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"side_way_controller");
    ROS_INFO("Side Way Finder Start");
    
    ros::NodeHandle rosHandle;
    ros::Subscriber sub = rosHandle.subscribe("catvehicle/front_laser_points", 100, callback);
    
    pub = rosHandle.advertise<control::sideWay>("control/sideWay", 100);
    pubError = rosHandle.advertise<control::angleDistanceError>("control/angleDistanceError", 100);
    pub2Python = rosHandle.advertise<control::pid_input>("control/error", 100);
    
    ros::spin();
    return 0;
}

