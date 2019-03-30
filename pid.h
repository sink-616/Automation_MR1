#ifndef PID_H
#define PID_H
#include <Arduino.h>

#define NU 0.0

class PID {
    
public:
    PID(double int_time);

    double limit(double result,double limit);
    
    void set(double P_, double I_, double D_);
    
    double con (double error);
    
    void reset();
    
private:
    
    double time;
    double max_L;
    double min_L;
    double p;
    double i;
    double d;
    double integ;
    double pre_err;
    
};

#endif
