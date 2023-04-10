#pragma once
#include "Tree.h"
#include <random>
#include <set>
#include <queue>
#include <iostream>
#include "GrSimRobotParam.h"
#include <vector>
#include <cmath>
#include "RobotKinematics.h"
#include <tuple>

#define THRESHOLD (1)//cm
// #define D_ADD (0.1)//m

using namespace std;

class Obstacle;
class RRT{
    public:
        Tree tree;
        int numObstruction=0;
        vector<Obstacle*> obstacles;
        int n=5;
        int N;
        double D_ADD=0.03;//m
        bool finished = false;
        double angle;
        bool hasCollided=false;
        vector<vector<double>> transform;
        RRT();
        // vector<int> generatePoint();
        bool isInObstacle(vector<double> newPoint);
        list<vector<double>> calculatePath(vector<double> myStart, vector<double> myEnd);
        void setAngle(vector<double> start, vector<double> end);
        double getAngle();
        void setTransform(double newAngle);
        vector<vector<double>> getTransform();
        vector<double> calcNewPoint(vector<double> newPoint);
        void setObstacles(vector<Obstacle*> newObstacles);
        vector<Obstacle*> getObstacles();
        
        
        // vector<double> y;

        // Node newNode(vector<int> point);

    
};

class Obstacle{
    public:
        // tuple<double,double,double,double,bool,double,bool,double,double,double,double,double,bool> myCommand;
        struct command{
            double id=0; 
            double xVel=0; 
            double yVel=0;
            double wVel=0; 
            bool setWheelSpeed=true;
            double chipVel=0; 
            bool isYellow=true;
            double wheelOneVel=0; 
            double wheelTwoVel=0;
            double wheelThreeVel=0; 
            double wheelFourVel=0;
            double kickVel=0; 
            bool isSpin=false;
        };
        command myCommand;
        int ID;
        bool color;
        double radius=(RobotRadius*2)+0.02;
        RobotKinematics myRK;
        RRT myRRT;
        list<vector<double>> myPath;
        vector<double> finalDestination;
        vector<double> nextPoint;
        bool hasPath=false;
        bool overriding=false;
        int whoCollision;
        int lastCollision;
        Obstacle(int ID, bool color);
        Obstacle();
        // Obstacle();
        ~Obstacle();
        bool isInObstacle(vector<double> point);
        void followPath();
        bool isCollision();
        // void goTo(vector<double> point,bool override=0);
        void setPath(vector<double> destination);
        void setPath();
        list<vector<double>> getPath();


};