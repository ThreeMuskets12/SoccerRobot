#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;


PID::PID( double dt, double max, double min, double Kp, double Ki, double Kd )
{
    this->dt=dt;
    this->max=max;
    this->min=min;
    this->Kp=Kp;
    this->Ki=Ki;
    this->Kd=Kd;
}

PID::~PID() 
{
    
}


/**
 * Implementation
 */


double PID::calculate( double newError )//goal, double actual
{
    double Pout = 0.0;
    double Iout = 0.0;
    double Dout = 0.0;
    
    // Calculate error
    double error = newError;
    // cout << "Error: " << error <<endl;
    // if(abs(error)<=0.001){
    //     integral=0.0;
    //     pre_error=0.0;
    //     // counter_PID++;
    //     // c_start=clock();
    // }
    // if(counter_PID==3 || counter_PID==6){
    //     c_end=clock();
    //     cout << c_end-c_start <<endl;
    // }

    // Proportional term
    Pout = Kp * error;
    // if ( abs(error) >= goal/2){
    //     output = 100 * (error/abs(error));
    //     return output;
    // }
    // if(abs(error) < goal/2){
        
    // }
    
    

    // Integral term
    
    integral += error * dt;
    // cout << "Integral:" << integral << endl;
    Iout = Ki * (integral);

    // Derivative term
    derivative = (error - pre_error) / dt;
    // cout<<"Derivative: "<<this->derivative<<endl;
    
    Dout = Kd * derivative;

    // cout<<"P: "<<Pout<<endl;
    // cout<<"I: "<<Iout<<endl;
    // cout<<"D: "<<Dout<<endl;

    // Calculate total output
    output = Pout + Iout + Dout;
    // cout<<"Out: "<<output<<endl;

    

    // Restrict to max/min
    if( output > max ){
        output = max;
    }
    else if( output < min ){
        output = min;
    }
    // Save error to previous error
    pre_error = error;

    return output;
}

void PID::reset(){
    this->Kp=0;
    this->Ki=0;
    this->Kd=0;
}


#endif