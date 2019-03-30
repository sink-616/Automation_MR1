#define PI_ 3.14159265359

#define PHASE 2

#define P_P 0.0
#define P_I 0.0
#define P_D 0.0

#define D_P 0.0
#define D_I 0.0
#define D_D 0.0

#define XS_P 0.085
#define XS_D 0.004

#define YS_P 0.08
#define YS_D 0.004

#define R_P 0.5
#define R_D 0.02

#define INT_TIME 15 //(ms)
#define S_LIMIT 1100
#define R_LIMIT 508

#define ENC_INIT_VAL 0x7FFF
#define RESOLUTION_X 800.0
#define RESOLUTION_Y 400.0
#define DIAMETER 38.0
#define W_DIS1 190.90
#define W_DIS2 249.50

#define LIMITER_ACCEL 1500
#define LIMITER_DECCEL 1500
#define LIMITER_ROT 1500
#define LIMITER_DEROT 1500
#define ADR_MD1 128
#define ADR_MD2 129
#define A_CMD 1.35
#define M_CMD 0.85
  
  void set_enc(){
  SYSTEM.MSTPCRA.BIT.MSTPA9 = 0; // MTU0~5モジュールストップ状態解除 
  SYSTEM.MSTPCRA.BIT.MSTPA13 = 0; // TPU0~5モジュールストップ状態解除
  PORT2.PMR.BIT.B4=1; //P24汎用入出力機能を選択
  PORT2.PMR.BIT.B5=1; //P25汎用入出力機能を選択
  PORT2.PMR.BIT.B2=1; //P22汎用入出力機能を選択
  PORT2.PMR.BIT.B3=1; //P23汎用入出力機能を選択
  PORTC.PMR.BIT.B2=1; //PC2汎用入出力機能を選択
  PORTC.PMR.BIT.B3=1; //PC3汎用入出力機能を選択

  MPC.PWPR.BIT.B0WI=0; //PFSWEビットへの書き込みを許可
  MPC.PWPR.BIT.PFSWE=1; //PFSレジスタへの書き込みを許可

  MPC.P24PFS.BIT.PSEL=0b00010;  //P24をMTCLKAピンとして使用
  MPC.P25PFS.BIT.PSEL=0b00010;  //P25をMTCLKBピンとして使用
  MPC.P22PFS.BIT.PSEL=0b00010;  //P22をMTCLKCピンとして使用
  MPC.P23PFS.BIT.PSEL=0b00010;  //P23をMTCLKDピンとして使用
  MPC.PC2PFS.BIT.PSEL=0b00011;  //PC2をTCLKAピンとして使用
  MPC.PC3PFS.BIT.PSEL=0b00011;  //PC3をTCLKBピンとして使用

  MPC.PWPR.BIT.PFSWE=0; //PFSレジスタへの書き込みを禁止
  MPC.PWPR.BIT.B0WI=1; //PFSWEビットへの書き込みを禁止

  MTU.TSTR.BIT.CST1=0; //MTU1.TCNTのカウント停止
  MTU1.TCR.BYTE=0;  //よくわからないけど，ここはゼロにしておけばOK?
  MTU1.TMDR.BIT.MD=0b0100; //位相計数モード1 4逓倍のカウント読み取り
  MTU1.TCNT=0; //カウントを初期化

  MTU1.TIOR.BIT.IOA=0b1010;  //両エッジでインプットキャプチャ
  MTU1.TIOR.BIT.IOB=0b1010;  //両エッジでインプットキャプチャ
  MTU.TSTR.BIT.CST1=1;  //MTU1.TCNTのカウント開始

  MTU.TSTR.BIT.CST2=0; //MTU2.TCNTのカウント停止
  MTU2.TCR.BYTE=0;  //よくわからないけど，ここはゼロにしておけばOK?
  MTU2.TMDR.BIT.MD=0b0100; //位相計数モード1 4逓倍のカウント読み取り
  MTU2.TCNT=0; //カウントを初期化

  MTU2.TIOR.BIT.IOA=0b1010;  //両エッジでインプットキャプチャ
  MTU2.TIOR.BIT.IOB=0b1010;  //両エッジでインプットキャプチャ
  MTU.TSTR.BIT.CST2=1;  //MTU2.TCNTのカウント開始

  TPUA.TSTR.BIT.CST1=0; //TPU1.TCNTのカウント停止
  TPU1.TCR.BYTE=0;  //よくわからないけど，ここはゼロにしておけばOK?
  TPU1.TMDR.BIT.MD=0b0100; //位相計数モード1 4逓倍のカウント読み取り
  TPU1.TCNT=0; //カウントを初期化

  TPU1.TIOR.BIT.IOA=0b1010;  //両エッジでインプットキャプチャ
  TPU1.TIOR.BIT.IOB=0b1010;  //両エッジでインプットキャプチャ
  TPUA.TSTR.BIT.CST1=1;  //TPU1.TCNTのカウント開始
  }
  
int diff(unsigned short int rawcount, unsigned short int pre_rawcount){
  double diff = (int)rawcount - (int)pre_rawcount; // 差分を計算

  if(diff > ENC_INIT_VAL){  // マイナス方向にゼロ点回ったとき
    diff = -(int)pre_rawcount - (0xFFFF - (int)rawcount);
  }
  else if(diff < -ENC_INIT_VAL){ // プラス方向にゼロ点回ったとき
    diff = (int)rawcount + (0xFFFF - (int)pre_rawcount);
  }
  return diff;
}
