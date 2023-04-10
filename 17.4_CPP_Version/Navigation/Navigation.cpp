#include "Navigation.h"

void Navigation::updateNavigationInfo( vector<vector<double>> Blue, vector<vector<double>> Yellow ){
    // clock_t c_start=clock();
    if(!initialized){
        for(int i=0; i<Blue.size();i++){
            myObstacles[i]=&BlueTeam[i];
            myObstacles[i+6]=&YellowTeam[i];
        }
        initialized=true;
    }
    for(int i=0; i<Blue.size();i++){
        // obs=&BlueTeam[i];
        myObstacles[i]->myRK.setX0(Blue[i].at(0));
        myObstacles[i]->myRK.setY0(Blue[i].at(1));
        myObstacles[i]->myRK.setTheta0(Blue[i].at(2));
        // cout<<myObstacles[i]->isCollision()<<endl;
        
        if(!myObstacles[i]->nextPoint.empty()){
            myObstacles[i]->myRK.setDeltaX(myObstacles[i]->nextPoint.at(0) - myObstacles[i]->myRK.getX0());
            myObstacles[i]->myRK.setDeltaY(myObstacles[i]->nextPoint.at(1) - myObstacles[i]->myRK.getY0());
            myObstacles[i]->myRK.setDeltaDistance();
            myObstacles[i]->myRK.setAngle();
            if(myObstacles[i]->nextPoint.at(2) - myObstacles[i]->myRK.getTheta0()>M_PI){
                myObstacles[i]->myRK.setDeltaTheta((myObstacles[i]->nextPoint.at(2) - myObstacles[i]->myRK.getTheta0())-2*M_PI);
            }
            else{
                myObstacles[i]->myRK.setDeltaTheta(myObstacles[i]->nextPoint.at(2) - myObstacles[i]->myRK.getTheta0());
            }
        }
        
        // obs2=&YellowTeam[i];
        myObstacles[i+6]->myRK.setX0(Yellow[i].at(0));
        myObstacles[i+6]->myRK.setY0(Yellow[i].at(1));
        myObstacles[i+6]->myRK.setTheta0(Yellow[i].at(2));
        // cout<<myObstacles[i+6]->isCollision()<<endl;
        // if(myObstacles[i]->isCollision()&&myObstacles[i]->lastCollision!=myObstacles[i]->whoCollision){
        //     myObstacles[i]->setPath();
        // }
        // if(myObstacles[i+6]->isCollision()){
        //     myObstacles[i+6]->setPath();
        // }
        if(!myObstacles[i+6]->nextPoint.empty()){
            myObstacles[i+6]->myRK.setDeltaX(myObstacles[i+6]->nextPoint.at(0) - myObstacles[i+6]->myRK.getX0());
            myObstacles[i+6]->myRK.setDeltaY(myObstacles[i+6]->nextPoint.at(1) - myObstacles[i+6]->myRK.getY0());
            myObstacles[i+6]->myRK.setDeltaDistance();
            myObstacles[i+6]->myRK.setAngle();
            if(myObstacles[i+6]->nextPoint.at(2)-myObstacles[i+6]->myRK.getTheta0()>M_PI){
                myObstacles[i+6]->myRK.setDeltaTheta(myObstacles[i+6]->nextPoint.at(2)-myObstacles[i+6]->myRK.getTheta0()-2*M_PI);
            }
            else{
                myObstacles[i+6]->myRK.setDeltaTheta(myObstacles[i+6]->nextPoint.at(2) - myObstacles[i+6]->myRK.getTheta0());
            }
        }
        // myObstacles[i]=obs;
        // myObstacles[i+6]=obs2;
    }
    for(int i=0; i<6;i++){
        myObstacles[i]->myRRT.setObstacles(myObstacles);
        myObstacles[i+6]->myRRT.setObstacles(myObstacles);
        myObstacles[i]->myRRT.obstacles.erase(myObstacles[i]->myRRT.obstacles.begin()+i);
        myObstacles[i+6]->myRRT.obstacles.erase(myObstacles[i+6]->myRRT.obstacles.begin()+i+6);
    }
    // clock_t c_end=clock();
    // cout << "Update Time: " << c_end-c_start << endl;
}

void Navigation::setCommand(int id, bool color){
    Obstacle::command* newCommand;
    if(color&&!myObstacles[id+6]->myRK.getWheelSpeeds().empty()){
        newCommand=&myObstacles[id+6]->myCommand;
        newCommand->id=id;
        newCommand->isYellow=color;
        newCommand->wheelOneVel=myObstacles[id+6]->myRK.getWheelSpeeds().at(0);
        newCommand->wheelTwoVel=myObstacles[id+6]->myRK.getWheelSpeeds().at(1);
        newCommand->wheelThreeVel=myObstacles[id+6]->myRK.getWheelSpeeds().at(2);
        newCommand->wheelFourVel=myObstacles[id+6]->myRK.getWheelSpeeds().at(3);
        navCommand[id+6]=newCommand;
        // return newCommand; 
    }
    else if(!color&&!myObstacles[id]->myRK.getWheelSpeeds().empty()){
        newCommand=&myObstacles[id]->myCommand;
        newCommand->id=id;
        newCommand->isYellow=color;
        newCommand->wheelOneVel=myObstacles[id]->myRK.getWheelSpeeds().at(0);
        newCommand->wheelTwoVel=myObstacles[id]->myRK.getWheelSpeeds().at(1);
        newCommand->wheelThreeVel=myObstacles[id]->myRK.getWheelSpeeds().at(2);
        newCommand->wheelFourVel=myObstacles[id]->myRK.getWheelSpeeds().at(3);
        navCommand[id]=newCommand;
        // return newCommand;
    }
    else{
        if(color){
            newCommand=&myObstacles[id+6]->myCommand;
            newCommand->id=id;
            newCommand->isYellow=color;
            newCommand->wheelOneVel=0;
            newCommand->wheelTwoVel=0;
            newCommand->wheelThreeVel=0;
            newCommand->wheelFourVel=0;
            navCommand[id+6]=newCommand;
        }
        else{
            newCommand=&myObstacles[id]->myCommand;
            newCommand->id=id;
            newCommand->isYellow=color;
            newCommand->wheelOneVel=0;
            newCommand->wheelTwoVel=0;
            newCommand->wheelThreeVel=0;
            newCommand->wheelFourVel=0;
            navCommand[id]=newCommand;
        }
        // return newCommand;
    }
    
}

void Navigation::setPath(int id, bool color, vector<double> newDestination){
    if(color){
        myObstacles[id+6]->setPath(newDestination);
    }
    else{
        myObstacles[id]->setPath(newDestination);
    }
} 

void Navigation::NavigationMain(){
    // clock_t c_start=clock();
    // cout<<"\nTime: " <<c_start<<endl;
    // cout<<myObstacles[3]->myRK.getDeltaX() << endl;
    // cout<<myObstacles[3]->myRK.getDeltaY() << endl;
    // cout<<myObstacles[3]->myRK.getDeltaTheta() << endl;
    // cout<<myObstacles[1]->myRK.getDeltaDistance()[0] << endl;
    // cout<<myObstacles[1]->myRK.getDeltaDistance()[1] << endl;
    cout<<"\n";
    myObstacles[0]->followPath();
    setCommand(0,0);
    myObstacles[1]->followPath();
    setCommand(1,0);
    myObstacles[2]->followPath();
    setCommand(2,0);
    myObstacles[3]->followPath();
    setCommand(3,0);
    myObstacles[4]->followPath();
    setCommand(4,0);
    myObstacles[5]->followPath();
    setCommand(5,0);
    YellowTeam[0].followPath();
    setCommand(0,1);
    YellowTeam[1].followPath();
    setCommand(1,1);
    YellowTeam[2].followPath();
    setCommand(2,1);
    YellowTeam[3].followPath();
    setCommand(3,1);
    YellowTeam[4].followPath();
    setCommand(4,1);
    YellowTeam[5].followPath();
    setCommand(5,1);

    // clock_t c_end=clock();
    // cout<<"Main Time: "<<c_end-c_start<<endl;

    
}