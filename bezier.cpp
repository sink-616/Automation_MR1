#include "bezier.h"

Bezier::Bezier(){
    for (int i = 0;i != 4;i++){
    conPx[i] = 0;
    conPy[i] = 0;
    }
    P_ = 0;
}

void Bezier::setPoints(double xpoints[], double ypoints[], int phase) {
    for (int i = 0;i != phase*4;i++){
    conPx[i] = xpoints[i];
    conPy[i] = ypoints[i];
    }
    start_x = conPx[0];
    start_y = conPy[0];
    P_ = phase;
}

double Bezier::tPointx0(double t, int phase) {return pow(1.0-t,3.0)*conPx[phase*4];}
double Bezier::tPointx1(double t, int phase) {return 3.0*pow(1.0-t,2.0)*t*conPx[phase*4+1];}
double Bezier::tPointx2(double t, int phase) {return 3.0*(1.0-t)*pow(t,2.0)*conPx[phase*4+2];}
double Bezier::tPointx3(double t, int phase) {return pow(t,3.0)*conPx[phase*4+3];}
double Bezier::tPointy0(double t, int phase) {return pow(1.0-t,3.0)*conPy[phase*4];}
double Bezier::tPointy1(double t, int phase) {return 3.0*pow(1.0-t,2.0)*t*conPy[phase*4+1];}
double Bezier::tPointy2(double t, int phase) {return 3.0*(1.0-t)*pow(t,2.0)*conPy[phase*4+2];}
double Bezier::tPointy3(double t, int phase) {return pow(t,3.0)*conPy[phase*4+3];}

double Bezier::GetPointx(double t){
    if(t >= P_) t = P_;
    int phase = (int)t;
    double gain = t - phase;
    return tPointx0(gain,phase) + tPointx1(gain,phase) + tPointx2(gain,phase) + tPointx3(gain,phase);
}
double Bezier::GetPointy(double t){
    if(t >= P_) t = P_;
    int phase = (int)t;
    double gain = t - phase;
    return tPointy0(gain,phase) + tPointy1(gain,phase) + tPointy2(gain,phase) + tPointy3(gain,phase);
}

double Bezier::dPointx1(double t, int phase){return 3.0*pow(1-t,2.0)*conPx[phase*4+1];}
double Bezier::dPointx2(double t, int phase){return 6.0*(1-t)*t*conPx[phase*4+2];}
double Bezier::dPointx3(double t, int phase){return 3.0*pow(t,2.0)*conPx[phase*4+3];}
double Bezier::dPointy1(double t, int phase){return 3.0*pow(1-t,2.0)*t*conPy[phase*4+1];}
double Bezier::dPointy2(double t, int phase){return 6.0*(1-t)*t*conPy[phase*4+2];}
double Bezier::dPointy3(double t, int phase){return 3.0*pow(t,2.0)*conPy[phase*4+3];}

double Bezier::dBezierx(double t){
    int phase = (int)t;
    double gain = t - phase;
    return tPointx1(gain,phase) + tPointx2(gain,phase) + tPointx3(gain,phase);
}
double Bezier::dBeziery(double t){
    int phase = (int)t;
    double gain = t - phase;
    return tPointy1(gain,phase) + tPointy2(gain,phase) + tPointy3(gain,phase);
}

double Bezier::diff(double rad, double pre_rad) {
    double diff = rad - pre_rad; // 差分を計算

    if (diff > PI) {  // マイナス方向にゼロ点回ったとき
        diff = -(PI + pre_rad) - (PI - rad);
    }
    else if (diff < -PI) { // プラス方向にゼロ点回ったとき
        diff = (PI - rad) + (PI + pre_rad);
    }

    return diff;
}

double Bezier::GetAngle(double x, double y, int unit){
    static double pre_x = start_x, pre_y = start_y, pre_rad = 0, rad = 0;
    double diff_rad = atan2((y - pre_y), (x - pre_x));
    rad += diff(diff_rad, pre_rad);
    pre_x = x;
    pre_y = y;
    pre_rad = diff_rad;
    if (unit) return (rad * 180 / PI);
    else return rad;
}
