#ifndef BEZIER_H
#define BEZIER_H
#include <Arduino.h>

#define DEG 1
#define RAD 0

class Bezier {
    
public:
    Bezier();

    void setPoints(double xpoints[], double ypoints[] , int phase);
    
    double tPointx0(double t, int phase);
    double tPointx1(double t, int phase);
    double tPointx2(double t, int phase);
    double tPointx3(double t, int phase);
    double tPointy0(double t, int phase);
    double tPointy1(double t, int phase);
    double tPointy2(double t, int phase);
    double tPointy3(double t, int phase);
    
    double GetPointx(double t);
    double GetPointy(double t);
    
    double dPointx1(double t, int phase);
    double dPointx2(double t, int phase);
    double dPointx3(double t, int phase);
    double dPointy1(double t, int phase);
    double dPointy2(double t, int phase);
    double dPointy3(double t, int phase);
    
    double dBezierx(double t);
    double dBeziery(double t);
    
    double diff(double rad, double pre_rad);
    
    double GetAngle(double x, double y, int unit);
    
private:
    double conPx[4];
    double conPy[4];
    double start_x;
    double start_y;
    double P_;
    
};

#endif
