#include "RobotKinematics.h"

// #include <math.h>
// #include <cmath>
// #include <vector>
// #include <iostream>

// using namespace std;

RobotKinematics::RobotKinematics(){}
RobotKinematics::~RobotKinematics(){}

void RobotKinematics::MoveToPoint(double x, double y, double theta)
{
    vector<vector<double>> newWheelToRobot = {{(1 / this->r) * (cos(this->getTheta0()) * sin(this->a1) + cos(this->a1) * sin(this->getTheta0())), (1 / this->r) * (sin(this->getTheta0()) * sin(this->a1) - cos(this->a1) * cos(this->getTheta0())), -(1 / this->r) * this->R},
                                              {(1 / this->r) * (cos(this->getTheta0()) * sin(this->a2) + cos(this->a2) * sin(this->getTheta0())), (1 / this->r) * (sin(this->getTheta0()) * sin(this->a2) - cos(this->a2) * cos(this->getTheta0())), -(1 / this->r) * this->R},
                                              {(1 / this->r) * (cos(this->getTheta0()) * sin(this->a3) + cos(this->a3) * sin(this->getTheta0())), (1 / this->r) * (sin(this->getTheta0()) * sin(this->a3) - cos(this->a3) * cos(this->getTheta0())), -(1 / this->r) * this->R},
                                              {(1 / this->r) * (cos(this->getTheta0()) * sin(this->a4) + cos(this->a4) * sin(this->getTheta0())), (1 / this->r) * (sin(this->getTheta0()) * sin(this->a4) - cos(this->a4) * cos(this->getTheta0())), -(1 / this->r) * this->R}};
    this->setWheelToRobot(newWheelToRobot);
 
    // this->setDeltaX(x - this->getX0());
    // this->setDeltaY(y - this->getY0());
    // if(abs(this->getTheta0())>(abs(theta)+M_PI_2)){
    //     this->setDeltaTheta(this->getTheta0() - theta);
    // }
    // else{
    //     this->setDeltaTheta(theta - this->getTheta0());
    // }
    
    
    // printf("Y: %f\n",this->getDeltaY());
    // printf("Theta: %f\n",this->getDeltaTheta());
    // double distance = sqrt(pow(this->getDeltaX(), 2) + pow(this->getDeltaY(), 2));
    double x_dot = 0.0;
    double y_dot = 0.0;
    double theta_dot = 0.0;

    // cout<<this->getDeltaTheta()<<endl;
    
    
    // cout<<"RK: "<<this->getDeltaDistance()<<endl;
    x_dot=abs(this->pid_x.calculate(this->getDeltaDistance()[0]))*cos(this->getAngle());
    y_dot=abs(this->pid_y.calculate(this->getDeltaDistance()[1]))*sin(this->getAngle());
    if(this->getUsePID()){
        x_dot=x_dot;
        // cout << "X_P: " << this->pid_x.Kp*this->getDeltaDistance()[0] << endl;
        // cout << "X_I: " << this->pid_x.Ki*this->pid_x.integral << endl;
        // cout << "X_D: " << this->pid_x.Kd*this->pid_x.derivative << endl;
        // cout << "X_Output: " << this->pid_x.output << endl;
        y_dot=y_dot;
        // cout << "Y_P: " << this->pid_y.Kp*this->getDeltaDistance()[1] << endl;
        // cout << "Y_I: " << this->pid_y.Ki*pid_y.integral << endl;
        // cout << "Y_D: " << this->pid_y.Kd*this->pid_y.derivative << endl;
        // cout << "Y_Output: " << this->pid_y.output << endl;
    }
    else{
        x_dot=0.8*cos(this->getAngle());
        y_dot=0.8*sin(this->getAngle());
    }
    
    theta_dot=pid_theta.calculate(this->getDeltaTheta());
    // cout << "Theta_Output: " << pid_theta.output << endl;
    
    // printf("%f\t %f\t %f\n",this->getX0(),this->getY0(),this->getTheta0());
    // printf("Goal: %f\t %f\t %f\n",x,y,theta);
    // printf("Delta: %f\t %f\t %f\n",this->getDeltaX(),this->getDeltaY(),this->getDeltaTheta());
    // printf("Time: %f\n",dt);
    vector<double> velocities = {x_dot, y_dot, theta_dot};
    // printf("Velocities: %f %f %f\n",x_dot,y_dot,theta_dot);


    this->setVelocities(velocities);
    // printf("Speed: %f\t %f\t %f\n",velocities[0],velocities[1],velocities[2]);

    vector<double> newWheelSpeeds = {{this->getWheelToRobot()[0][0] * velocities[0] + this->getWheelToRobot()[0][1] * velocities[1] + this->getWheelToRobot()[0][2] * velocities[2]},
                                  {this->getWheelToRobot()[1][0] * velocities[0] + this->getWheelToRobot()[1][1] * velocities[1] + this->getWheelToRobot()[1][2] * velocities[2]},
                                  {this->getWheelToRobot()[2][0] * velocities[0] + this->getWheelToRobot()[2][1] * velocities[1] + this->getWheelToRobot()[2][2] * velocities[2]},
                                  {this->getWheelToRobot()[3][0] * velocities[0] + this->getWheelToRobot()[3][1] * velocities[1] + this->getWheelToRobot()[3][2] * velocities[2]}};

    // printf("%f\n",this->getTheta0());
    this->setWheelSpeeds(this->robotToGrsim(newWheelSpeeds));//this->robotToGrsim(newWheelSpeeds)
    // printf("Wheelspeed: %f\t %f\t %f\t %f\n",this->getWheelSpeeds()[0],this->getWheelSpeeds()[1],this->getWheelSpeeds()[2],this->getWheelSpeeds()[3]);
}

vector<double> RobotKinematics::robotToGrsim(vector<double> wheelSpeed){
    vector<double> grSim={-1*wheelSpeed[0],-1*wheelSpeed[1],-1*wheelSpeed[2],-1*wheelSpeed[3]};
    return grSim;   
}

// Setter x0 
void RobotKinematics::setX0(double newX0)
{
    this->x0 = newX0/1000;
}
// Getter x0
double RobotKinematics::getX0()
{
    return this->x0;
}

// Setter y0
void RobotKinematics::setY0(double newY0)
{
    y0 = newY0/1000;
}
// Getter y0
double RobotKinematics::getY0()
{
    return y0;
}

// Setter theta0
void RobotKinematics::setTheta0(double newTheta0)
{
    // if(newTheta0<0){
    //     newTheta0=newTheta0+2*M_PI;
    // }
    theta0 = newTheta0;
}
// Getter theta0
double RobotKinematics::getTheta0()
{
    return theta0;
}

// Setter dx
void RobotKinematics::setDeltaX(double newDeltaX)
{
    deltaX = newDeltaX;
}
// Getter dx
double RobotKinematics::getDeltaX()
{
    return deltaX;
}

// Setter dy
void RobotKinematics::setDeltaY(double newDeltaY)
{
    deltaY = newDeltaY;
}
// Getter dy
double RobotKinematics::getDeltaY()
{
    return deltaY;
}


void RobotKinematics::setUsePID(bool newbool){
    this->usePID=newbool;
}

bool RobotKinematics::getUsePID(){
    return this->usePID;
}


//Setter dDist
void RobotKinematics::setDeltaDistance(){
    if(getDeltaX()<0&&getDeltaY()<0){
        deltaDistance[0]=-1*sqrt(pow(getDeltaX(),2)+pow(getDeltaY(),2));
        deltaDistance[1]=-1*sqrt(pow(getDeltaX(),2)+pow(getDeltaY(),2));
    }
    else if(getDeltaX()<0&&getDeltaY()>=0){
        deltaDistance[0]=-1*sqrt(pow(getDeltaX(),2)+pow(getDeltaY(),2));
        deltaDistance[1]=sqrt(pow(getDeltaX(),2)+pow(getDeltaY(),2));
    }
    else if(getDeltaX()>=0&&getDeltaY()<0){
        deltaDistance[0]=sqrt(pow(getDeltaX(),2)+pow(getDeltaY(),2));
        deltaDistance[1]=-1*sqrt(pow(getDeltaX(),2)+pow(getDeltaY(),2));
    }
    else{
        deltaDistance[0]=sqrt(pow(getDeltaX(),2)+pow(getDeltaY(),2));
        deltaDistance[1]=sqrt(pow(getDeltaX(),2)+pow(getDeltaY(),2));
    }
    
}

//Getter dDist
vector<double> RobotKinematics::getDeltaDistance(){
    return deltaDistance;
}

//Setter dAng
void RobotKinematics::setAngle(){
    if(getDeltaX()!=0){
        angle=atan2(getDeltaY(),getDeltaX());
    }
    else{
        angle=atan2(getDeltaY(),0.000001);
    }
    
}

//Getter dAng
double RobotKinematics::getAngle(){
    return angle;
}

// Setter dtheta
void RobotKinematics::setDeltaTheta(double newDeltaTheta)
{
    deltaTheta = newDeltaTheta;
}
// Getter dtheta
double RobotKinematics::getDeltaTheta()
{
    return deltaTheta;
}

// Setter dt
void RobotKinematics::setDeltaTime(double newDeltaTime)
{
    deltaTime = newDeltaTime;
}
// Getter dt
double RobotKinematics::getDeltaTime()
{
    return deltaTime;
}

// Setter WheelToRobot
void RobotKinematics::setWheelToRobot(vector<vector<double>> newWheelToRobot)
{
    wheelToRobot = newWheelToRobot;
}
// Getter WheelToRobot
vector<vector<double>> RobotKinematics::getWheelToRobot()
{
    return wheelToRobot;
}

// Setter RobotToWheel
void RobotKinematics::setRobotToWheel(vector<vector<double>> newRobotToWheel)
{
    robotToWheel = newRobotToWheel;
}
// Getter RobotToWheel
vector<vector<double>> RobotKinematics::getRobotToWheel()
{
    return robotToWheel;
}

//Setter x0
void RobotKinematics::setVelocities(vector<double> newVelocities){
    velocities = newVelocities;
}

//Getter x0
vector<double> RobotKinematics::getVelocities(){
    return velocities;
}

void RobotKinematics::setWheelSpeeds(vector<double> newWheelSpeeds){
    this->wheelSpeeds=newWheelSpeeds;
}

vector<double> RobotKinematics::getWheelSpeeds(){
    return this->wheelSpeeds;
}

