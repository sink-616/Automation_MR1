#include "pid.h"

PID::PID(double int_time){
    time = int_time;
    reset();
}

double PID::limit(double result,double limit){
  max_L = limit;
  min_L = -limit;
  if(result >= max_L) result = max_L;
  if(result <= min_L) result = min_L;
  return result;
}

void PID::set(double P_, double I_, double D_){
    p = P_;
    i = I_;
    d = D_;
}

double PID::con (double error){
    double result = 0;
    
    result += error * p;
    
    integ += (error + pre_err) / 2.0 * time;
    if (error < 5.0 && error > -5.0) integ = 0.0;
    result += integ * i;
    
    result += (error - pre_err) / time * d;
    
    pre_err = error;
    
    return result;
}

void PID::reset(){
    p = 0.0;
    i = 0.0;
    d = 0.0;
    integ = 0.0;
    pre_err = 0.0;
}

