#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <iostream>
#include <vector>
#include "RRT.h"
#include "RobotKinematics.h"
// #include "Simulation.h"


using namespace std;
class Navigation
{
    public:
        void NavigationMain();
        void updateNavigationInfo( vector<vector<double>>, vector<vector<double>> );            
        void setCommand(int id, bool color);
        void setPath(int id, bool color, vector<double> newDestination);
        
        vector<Obstacle::command*> navCommand=vector<Obstacle::command*>(12);
  
    private:
        Obstacle robotB0 = Obstacle (0,false);
        Obstacle robotB1 = Obstacle (1,false);
        Obstacle robotB2 = Obstacle (2,false);
        Obstacle robotB3 = Obstacle (3,false);
        Obstacle robotB4 = Obstacle (4,false);
        Obstacle robotB5 = Obstacle (5,false);
        vector<Obstacle> BlueTeam = {robotB0,robotB1,robotB2,robotB3,robotB4,robotB5};

        Obstacle robotY0 = Obstacle (0,true);
        Obstacle robotY1 = Obstacle (1,true);
        Obstacle robotY2 = Obstacle (2,true);
        Obstacle robotY3 = Obstacle (3,true);
        Obstacle robotY4 = Obstacle (4,true);
        Obstacle robotY5 = Obstacle (5,true);
        vector<Obstacle> YellowTeam = {robotY0,robotY1,robotY2,robotY3,robotY4,robotY5};

        vector<Obstacle*> myObstacles=vector<Obstacle*>(12);
        bool initialized =false;
        Obstacle* obs;
        Obstacle* obs2;

};



#endif