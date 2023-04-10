#ifndef _PID_H_
#define _PID_H_

class PID
{
    public:
        double Kp; //-  proportional gain
        double Ki; //-  Integral gain
        double Kd; //-  derivative gain
        double dt; //-  loop interval time
        double max; //- maximum value of manipulated variable
        double min; //- minimum value of manipulated variable
        double pre_error=0.0;
        double integral;
        double derivative=0.0;
        double output=0.0;
        int counter_PID=0;
        clock_t c_start;
        clock_t c_end;

        PID( double dt, double max, double min, double Kp, double Ki, double Kd );
        void reset();

        // Returns the manipulated variable given a setpoint and current process value
        double calculate(double error);//goal, double actual
        ~PID();
};

#endif