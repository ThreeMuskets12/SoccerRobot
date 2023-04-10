#ifndef ROBOTKINEMATICS_H
#define ROBOTKINEMATICS_H

#pragma once

#include <math.h>
#include <cmath>
#include <vector>
#include <iostream>
#include "GrSimRobotParam.h"
#include "pid.h"

using namespace std;

class RobotKinematics
{
    private:
        double x0=0.0;
        double y0=0.0;
        double theta0=0.0;
        double deltaX=0.0;
        double deltaY=0.0;
        vector<double> deltaDistance=vector<double>(2);
        double angle=0.0;
        double deltaTheta=0.0;
        double deltaTime=0.0;
        vector<vector<double>> wheelToRobot;
        vector<vector<double>> robotToWheel;
        vector<double> velocities;
        vector<double> wheelSpeeds;
    public:
        double R=RobotRadius;
        double r=WheelRadius;
        double a1=AngleToWheel1;
        double a2=AngleToWheel2;
        double a3=AngleToWheel3;
        double a4=AngleToWheel4;
        bool usePID=false;
        //pu=4,6,8
        PID pid_x = PID(0.02, 0.5, -0.5, 15, 0, 1); //20.1928
        PID pid_y = PID(0.02, 0.5, -0.5, 15, 0, 1); //.6*50*(45/8)
        PID pid_theta = PID(0.02, 18, -18, 12, 0, 0.145); // Do Not Touch 
        RobotKinematics();
        ~RobotKinematics();

        //Setter x0
        void setX0(double newX0);

        //Getter x0
        double getX0();


        //Setter y0
        void setY0(double newY0);

        //Getter y0
        double getY0();

        void setUsePID(bool newbool);

        bool getUsePID();


        //Setter theta0
        void setTheta0(double newTheta0);

        //Getter theta0
        double getTheta0();


        //Setter dx
        void setDeltaX(double newDeltaX);

        //Getter dx
        double getDeltaX();


        //Setter dy
        void setDeltaY(double newDeltaY);

        //Getter dy
        double getDeltaY();

        //Setter dy
        void setDeltaDistance();

        //Getter dy
        vector<double> getDeltaDistance();

        //Setter dy
        void setAngle();

        //Getter dy
        double getAngle();


        //Setter dtheta
        void setDeltaTheta(double newDeltaTheta);

        //Getter dtheta
        double getDeltaTheta();


        //Setter dt
        void setDeltaTime(double newDeltaTime);

        //Getter dt
        double getDeltaTime();


        //Setter WheelToRobot
        void setWheelToRobot(vector<vector<double>> newWheelToRobot);
        //Getter WheelToRobot
        vector<vector<double>> getWheelToRobot();


        //Setter RobotToWheel
        void setRobotToWheel(vector<vector<double>> newRobotToWheel);
        //Getter RobotToWheel
        vector<vector<double>> getRobotToWheel();

        //Setter x0
        void setVelocities(vector<double> newVelocities);

        //Getter x0
        vector<double> getVelocities();

        void setWheelSpeeds(vector<double> newWheelSpeeds);

        vector<double> getWheelSpeeds();

        vector<double> robotToGrsim(vector<double> wheelSpeed);

        void MoveToPoint(double x, double y, double theta);
};

#endif // MYUDP_H

// int main(){
//     RobotKinematics myKinematics;
//     vector<double> speeds=myKinematics.MoveToPoint(0,1,0);
//     // cout << speeds[0] << endl;
//     // cout << speeds[1] << endl;
//     // cout << speeds[2] << endl;
//     // cout << speeds[3] << endl;
//     return 0;
// }


