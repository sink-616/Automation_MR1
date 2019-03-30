#include <Arduino.h>
#include <RoboClaw.h>
#include "Set_Up.h"
#include "MsTimerTPU3.h"
#include"iodefine_gcc63n.h"
#include "pid.h"
#include "bezier.h"

RoboClaw MD(&Serial2,10);

PID posix_pid(INT_TIME*0.001);
PID posiy_pid(INT_TIME*0.001);
PID deg_pid(INT_TIME*0.001);
PID spdx_pid(INT_TIME*0.001);
PID spdy_pid(INT_TIME*0.001);
PID srot_pid(INT_TIME*0.001);

//int encount1 = 0, encount2 = 0, encount3 = 0;
int Px = 127,Py = 127,Prot = 127,Pdire = 1;
double Ppx = 127,Ppy = 127,Pprot = 127,Ppdire = 1,Ppqpps = 0,Ppadd = 0,Ppspd = 0, Ppsrot = 0;
unsigned short int len = 0;
double global_x = 0,global_y = 0,deg = 0;
double spd_Lx = 0,spd_Ly = 0,spd_Lrot = 0;


double Ax[PHASE];
double Bx[PHASE];
double Cx[PHASE];
double Dx[PHASE];

double Ay[PHASE];
double By[PHASE];
double Cy[PHASE];
double Dy[PHASE];

double a_be[PHASE];
double b_be[PHASE];
double c_be[PHASE];
double d_be[PHASE];
double e_be[PHASE];
double f_be[PHASE];
double d_be_[PHASE];
double e_be_[PHASE];
double f_be_[PHASE];

double t_be = 0.0;
double pre_t_be = 0.1;
double epsilon = 1.0;

void setup() {
  MD.begin(115200);
  Serial.begin(115200);
  Serial1.begin(115200);
  MsTimerTPU3::set(INT_TIME, &serial_read);
  MsTimerTPU3::start();
  set_enc();
  posix_pid.set(P_P,P_I,P_D);
  posiy_pid.set(P_P,P_I,P_D);
  deg_pid.set(D_P,D_I,D_D);
  spdx_pid.set(XS_P,NU,XS_D);
  spdy_pid.set(YS_P,NU,YS_D);
  srot_pid.set(R_P,NU,R_D);
  for(int i = 0; i < PHASE; i++) {
    Ax[i] = Px[3*i+3] -3*Px[3*i+2] + 3*Px[3*i+1] - Px[3*i+0];
    Ay[i] = Py[3*i+3] -3*Py[3*i+2] + 3*Py[3*i+1] - Py[3*i+0];
    Bx[i] = Px[3*i+2] -2*Px[3*i+1] + Px[3*i+0];
    By[i] = Py[3*i+2] -2*Py[3*i+1] + Py[3*i+0];
    Cx[i] = Px[3*i+1] - Px[3*i+0];
    Cy[i] = Py[3*i+1] - Py[3*i+0];
    Dx[i] = Px[3*i+0];
    Dy[i] = Py[3*i+0];
  }
  for(int i = 0; i < PHASE; i++) {
    a_be[i] = pow(Ax[i], 2.0) + pow(Ay[i], 2.0);
    b_be[i] = 5*(Ax[i]*Bx[i] + Ay[i]*By[i]);
    c_be[i] = 2*((3*pow(Bx[i],2.0)+2*Ax[i]*Cx[i]) + (3*pow(By[i],2.0)+2*Ay[i]*Cy[i]));
    d_be_[i] = 9*Bx[i]*Cx[i] + 9*By[i]*Cy[i];
    e_be_[i] = 3*pow(Cx[i],2.0) + 3*pow(Cy[i],2.0);
    f_be_[i] = 0;
  }
}

// tを求めるための方程式
double func(int p, double t){
  return a_be[p] * pow(t, 5.0) + b_be[p] * pow(t,4.0) + c_be[p] * pow(t,3.0) + d_be[p] * pow(t,2.0) + e_be[p] * t + f_be[p];
}
// tを求めるための方程式の1階微分
double dfunc(int p, double t){
  return 5.0 * a_be[p] * pow(t, 4.0) +  4.0 * b_be[p] * pow(t,3.0) + 3.0 * c_be[p] * pow(t,2.0) + 2.0 * d_be[p] * t + e_be[p];
}

void serial_read(){
  len = Serial1.available();
  static char buff[10] = {};
  static unsigned char x = 127 ,y = 127 ,rot = 127 ;
  static char manu_direction = 0x01; 
  static int co = 0;
  
  for (int i = len;i;i--){
      
    buff[co] = Serial1.read();
    if(buff[co] == '\n') {
      manu_direction = buff[co-4];
      x = buff[co-3];
      y = buff[co-2];
      rot = buff[co-1];
      for (int j = co;j;j--){
        buff[j] = 0;
      }
      co=0;
    }else;
    
    co++;
    
    if (co > 9){
      for (int j = 0;j <= 9;j++){
        buff[j] = 0;
      }
      co=0;
    }else;
  }
  if (x == 1) x = 0;
  if (y == 1) y = 0;
  if (rot == 1) rot = 0;
  Px = x;
  Py = y;
  Prot = rot;
  Pdire = manu_direction;
  odometry();
  manual (Pdire,Px,Py,Prot);
}

void odometry(){
  unsigned short int rawcount1 = MTU1.TCNT;
  static unsigned short int pre_rawcount1 = 0;
  // 差分をインクリメントする
  double Diff1 = -diff(rawcount1,pre_rawcount1);
  //encount1 += Diff1;
  pre_rawcount1 = rawcount1;
  
  unsigned short int rawcount2 = MTU2.TCNT;
  static unsigned short int pre_rawcount2 = 0;
  // 差分をインクリメントする
  double Diff2 = (double)(diff(rawcount2,pre_rawcount2));
  //encount2 += Diff2;
  pre_rawcount2 = rawcount2;
  
  unsigned short int rawcount3 = TPU1.TCNT;
  static unsigned short int pre_rawcount3 = 0;
  // 差分をインクリメントする
  double Diff3 = -diff(rawcount3,pre_rawcount3);
  //encount3 += Diff3;
  pre_rawcount3 = rawcount3;

  double v_enc1 = (Diff1/RESOLUTION_X)*(DIAMETER*PI)/(INT_TIME*0.001);
  double v_enc2 = (Diff2/RESOLUTION_Y)*(DIAMETER*PI)/(INT_TIME*0.001);
  double v_enc3 = (Diff3/RESOLUTION_X)*(DIAMETER*PI)/(INT_TIME*0.001);
  
  //local speed (mm/s)
  double omega_e1 = v_enc1/W_DIS1;
  double omega_e3 = v_enc3/W_DIS1;
  spd_Lrot = (omega_e1/* + (v_enc2/W_DIS2) */+ omega_e3)*0.287;
  spd_Ly = (v_enc1 - v_enc3)/2;
  spd_Lx = (v_enc2 - spd_Lrot*W_DIS2);
  
  deg += spd_Lrot;
  double rad = deg/180*PI;
  
  //grobal speed (mm/s)
  double spd_Gx = spd_Ly*cos(rad) + spd_Lx*sin(rad);
  double spd_Gy = spd_Ly*sin(rad) + spd_Lx*cos(rad);
  double spd_Grot = spd_Lrot;
  
  double del_x = spd_Lx*(INT_TIME*0.001);
  double del_y = spd_Ly*(INT_TIME*0.001);
  
  global_x += del_y;
  global_y += del_x;
  
  Ppdire = deg;
  
}

void Mecanum_command(double duty[4]){
  MD.SpeedM1(ADR_MD1, (int)(duty[3])*M_CMD);
  MD.SpeedM2(ADR_MD1, (int)(duty[2])*M_CMD);
  MD.SpeedM1(ADR_MD2, (int)(duty[1])*M_CMD);
  MD.SpeedM2(ADR_MD2, (int)(duty[0])*M_CMD);
}

void Mecanum_move(double add_direction, double add_speed, double revolution){
  double duty[4];
  
  duty[0] = (cos(add_direction) - sin(add_direction))*add_speed - revolution;//右前
  duty[1] = (cos(add_direction) + sin(add_direction))*add_speed + revolution;//左前
  duty[2] = (cos(add_direction) + sin(add_direction))*add_speed - revolution;//右後
  duty[3] = (cos(add_direction) - sin(add_direction))*add_speed + revolution;//左後
  /*Ppx = duty[0]*M_CMD;
  Ppy = duty[1]*M_CMD;
  Pprot = duty[2]*M_CMD;
  Ppdire = duty[3]*M_CMD;*/
  Mecanum_command(duty);
}    

double limitAD(double crr, double prev, double tick){   //加減速制限
    if(crr > prev + LIMITER_ACCEL * tick) { return prev + LIMITER_ACCEL * tick; }
    if(crr < prev - LIMITER_DECCEL * tick) { return prev - LIMITER_DECCEL * tick; }
    return crr;
}

double limitR(double crr, double prev, double tick){   //加減速制限
    if(crr > prev + LIMITER_ROT * tick) { return prev + LIMITER_ROT * tick; }
    if(crr < prev - LIMITER_DEROT * tick) { return prev - LIMITER_DEROT * tick; }
    return crr;
}

void manual (int manu_direction,double x, double y, double rot){
  if (manu_direction == 1) {
    x = -x +127;
    y = -y +127;
  }
  
  else {
    x -=127;
    y -=127;
  }
  
  rot -= 127;
  //rot = -rot;
  
  static double add_x = 0.0, add_y = 0.0, add_srot = 0.0;
  
  static double prevX = 0, prevY = 0,preR = 0;
  
  x = limitAD(x*1000/127, prevX, INT_TIME*0.001);
  y = limitAD(y*1000/127*-1, prevY, INT_TIME*0.001);
  rot = limitR(rot*4, preR, INT_TIME*0.001);
  
  double qpps_x = spd_Lx/(100.0*PI)*(4.0*48.0)/M_CMD;
  double qpps_y = spd_Ly/(100.0*PI)*(4.0*48.0)/M_CMD;
  
  add_x += spdx_pid.con(x-qpps_x);
  add_y += spdy_pid.con(y-qpps_y);
  
  int L_rot = spd_Lrot*300;
  
  add_srot += srot_pid.con(rot-L_rot);
  
  double sam_limit;
  
  add_x = spdx_pid.limit(add_x,abs(add_x)+200);
  add_y = spdy_pid.limit(add_y,S_LIMIT);
  Ppsrot = srot_pid.limit(add_srot,R_LIMIT);
  
  if(x == 0 && qpps_x == 0.0) {
    add_x = 0;
  }
  if(y == 0 && qpps_y == 0) {
    add_y = 0;
  }
  if(rot == 0 && spd_Lrot == 0) {
    Ppsrot = 0;
    add_srot = 0;
  }
  
  double add_speed = hypot( add_x, add_y);
  
  double add_direction = atan2( add_x, add_y);
  
  Mecanum_move(add_direction,add_speed,Ppsrot);
  
  prevX = x;
  prevY = y;
  preR = rot;
  Ppx = x;
  Ppy = y;
  Ppadd = qpps_y;
  Ppspd = add_y;
}

void newton(){
  int count_newton = 0;
    do {
      t_be = pre_t_be - func(phase, pre_t_be)/dfunc(phase, pre_t_be);
      epsilon = abs((t_be - pre_t_be)/pre_t_be);
      
      //if(t_be < 0) t_be = 0.0;
      //else if(t_be > 1.0) t_be = 1.0;
      
      pre_t_be = t_be;
      count_newton++;
    }while(epsilon >= 1e-4 || count_newton <= 50);
}

void loop() {
  //Serial.print(Ppx);
  //Serial.print("\t");
  Serial.print(Ppy);
  Serial.print("\t");
  /*Serial.print(Pprot);
  Serial.print("\t");*/
  //Serial.print(Ppdire);
  //Serial.print("\t");
  //Serial.print(Ppqpps);
  //Serial.print("\t");
  Serial.print(Ppadd);
  Serial.print("\t");
  Serial.print(Ppspd);
  Serial.print("\n");
  delay(150);
}
