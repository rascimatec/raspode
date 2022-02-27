#include <Arduino.h>
#include <Servo.h> //Servo library

#define TibiaLength 145.00
#define FemurLength 95.00
#define CoxaLength 36.00
#define BodySideLength 45.00


Servo servotibia;
Servo servofemur;
Servo servocoxa;

int beta,gama,alpha, beta1;
float x,y,z,H,L;

int PosX,PosY ,PosZ ,RotX ,RotY ,RotZ ;
//float BodyCenterOffsetX_1,BodyCenterOffsetX_2,BodyCenterOffsetX_3,BodyCenterOffsetX_4,BodyCenterOffsetX_5,BodyCenterOffsetX_6;
//float BodyCenterOffsetY_1,BodyCenterOffsetY_2,BodyCenterOffsetY_3,BodyCenterOffsetY_4,BodyCenterOffsetY_5,BodyCenterOffsetY_6;
//float FeetPosX_1,FeetPosY_1,FeetPosZ_1,FeetPosX_2,FeetPosY_2,FeetPosZ_2,FeetPosX_3,FeetPosY_3,FeetPosZ_3,FeetPosX_4,FeetPosY_4,FeetPosZ_4,FeetPosX_5,FeetPosY_5,FeetPosZ_5,FeetPosX_6,FeetPosY_6,FeetPosZ_6;

float BodyCenterOffset1 = BodySideLength/2;
float BodyCenterOffset2 = sqrt(pow(BodySideLength,2) - pow(BodyCenterOffset1,2));



//Body Center Offset X
//Leg 1
float BodyCenterOffsetX_1 = BodyCenterOffset1;
//Leg 2
float BodyCenterOffsetX_2=BodySideLength;
//Leg 3
float BodyCenterOffsetX_3	= BodyCenterOffset1;
//Leg 4
float BodyCenterOffsetX_4	= -BodyCenterOffset1;
//Leg 5
float BodyCenterOffsetX_5	= -BodySideLength;
//Leg 6
float BodyCenterOffsetX_6 =	-BodyCenterOffset1;

//Body Center Offset Y
//Leg 1
float BodyCenterOffsetY_1 =	BodyCenterOffset2;
//Leg 2
float BodyCenterOffsetY_2	 = 0;
//Leg 3
float BodyCenterOffsetY_3	= -BodyCenterOffset2;
//Leg 4
float BodyCenterOffsetY_4 =	-BodyCenterOffset2;
//Leg 5
float BodyCenterOffsetY_5	= 0;
//Leg 6
float BodyCenterOffsetY_6	= BodyCenterOffset2;

//initial feet position
//Leg 1
float FeetPosX_1	= cos(60/180*PI)*(CoxaLength + FemurLength);
float FeetPosZ_1	= TibiaLength;
float FeetPosY_1	= sin(60/180*PI)*(CoxaLength + FemurLength);
//Leg 2
float FeetPosX_2	= CoxaLength + FemurLength;
float FeetPosZ_2	= TibiaLength;
float FeetPosY_2	= 0;
//Leg 3
float FeetPosX_3	= cos(60/180*PI)*(CoxaLength + FemurLength);
float FeetPosZ_3	= TibiaLength;
float FeetPosY_3	= sin(-60/180*PI)*(CoxaLength + FemurLength);
//Leg 4
float FeetPosX_4	= -cos(60/180*PI)*(CoxaLength + FemurLength);
float FeetPosZ_4	= TibiaLength;
float FeetPosY_4	= sin(-60/180*PI)*(CoxaLength + FemurLength);
//Leg 5
float FeetPosX_5	= -(CoxaLength + FemurLength);
float FeetPosZ_5	= TibiaLength;
float FeetPosY_5	= 0;
//Leg 6
float FeetPosX_6	= -cos(60/180*PI)*(CoxaLength + FemurLength);
float FeetPosZ_6	= TibiaLength;
float FeetPosY_6	= sin(60/180*PI)*(CoxaLength + FemurLength);


void Body1_IK(float PosY, int PosX, float FeetPosY_1, float BodyCenterOffsetY_1, float BodyCenterOffsetX_1, float FeetPosX_1, float RotY , float RotZ, float RotX){

float TotalY_1,TotalX_1,DistBodyCenterFeet_1,AngleBodyCenterX_1,RollZ_1,PitchZ_1,BodyIKX_1,BodyIKY_1,BodyIKZ_1;

//Leg 1
TotalY_1	= FeetPosY_1 + BodyCenterOffsetY_1 + PosY;
TotalX_1	= FeetPosX_1 + BodyCenterOffsetX_1 + PosX;
DistBodyCenterFeet_1	= sqrt(pow(TotalY_1,2) + pow(TotalX_1,2));
AngleBodyCenterX_1	= PI/2 - atan2(TotalY_1, TotalX_1);
RollZ_1	= tan(RotZ * PI/180) * TotalX_1;
PitchZ_1	= tan(RotX * PI/180) * TotalY_1;
BodyIKX_1	= cos(AngleBodyCenterX_1 + (RotY * PI/180)) * DistBodyCenterFeet_1 - TotalX_1;
BodyIKY_1	= (sin(AngleBodyCenterX_1 + (RotY  * PI/180)) * DistBodyCenterFeet_1) - TotalY_1;
BodyIKZ_1	= RollZ_1 + PitchZ_1;
}

void Body2_IK(float PosY, int PosX, float FeetPosY_2, float BodyCenterOffsetY_2, float BodyCenterOffsetX_2, float FeetPosX_2, float RotY, float RotZ, float RotX ){

float TotalY_2,TotalX_2,DistBodyCenterFeet_2,AngleBodyCenterX_2,RollZ_2,PitchZ_2,BodyIKX_2,BodyIKY_2,BodyIKZ_2;

//Leg 2
TotalY_2	= FeetPosY_2 + BodyCenterOffsetY_2 + PosY;
TotalX_2	= FeetPosX_2 + PosX + BodyCenterOffsetX_2;
DistBodyCenterFeet_2	= sqrt(pow(TotalY_2,2) + pow(TotalX_2,2));
AngleBodyCenterX_2	= PI/2 - atan2(TotalY_2, TotalX_2);
RollZ_2	= tan(RotZ * PI/180) * TotalX_2;
PitchZ_2	= tan(RotX * PI/180) * TotalY_2;
BodyIKX_2	= cos(AngleBodyCenterX_2 + (RotY * PI/180)) * DistBodyCenterFeet_2 - TotalX_2;
BodyIKY_2	= (sin(AngleBodyCenterX_2 + (RotY  * PI/180)) * DistBodyCenterFeet_2) - TotalY_2;
BodyIKZ_2	= RollZ_2 + PitchZ_2;
}

void Body3_IK(float PosY, int PosX, float FeetPosY_3, float BodyCenterOffsetY_3, float BodyCenterOffsetX_3, float FeetPosX_3, float RotY, float RotZ, float RotX  ){


float TotalY_3,TotalX_3,DistBodyCenterFeet_3,AngleBodyCenterX_3,RollZ_3,PitchZ_3,BodyIKX_3,BodyIKY_3,BodyIKZ_3;

//Leg 3
TotalY_3	= FeetPosY_3 + BodyCenterOffsetY_3 + PosY;
TotalX_3	= FeetPosX_3 + BodyCenterOffsetX_3 + PosX;
DistBodyCenterFeet_3	= sqrt(pow(TotalY_3,2) + pow(TotalX_3,2));
AngleBodyCenterX_3	= PI/2 - atan2(TotalY_3, TotalX_3);
RollZ_3	= tan(RotZ * PI/180) * TotalX_3;
PitchZ_3	= tan(RotX * PI/180) * TotalY_3;
BodyIKX_3	= cos(AngleBodyCenterX_3 + (RotY * PI/180)) * DistBodyCenterFeet_3 - TotalX_3;
BodyIKY_3	= (sin(AngleBodyCenterX_3 + (RotY  * PI/180)) * DistBodyCenterFeet_3) - TotalY_3;
BodyIKZ_3	= RollZ_3 + PitchZ_3;
}

void Body4_IK(float PosY, int PosX, float FeetPosY_4, float BodyCenterOffsetY_4, float BodyCenterOffsetX_4, float FeetPosX_4, float RotY, float RotZ, float RotX ){


float TotalY_4,TotalX_4,DistBodyCenterFeet_4,AngleBodyCenterX_4,RollZ_4,PitchZ_4,BodyIKX_4,BodyIKY_4,BodyIKZ_4;

//Leg 4
TotalY_4	= FeetPosY_4 + BodyCenterOffsetY_4 + PosY;
TotalX_4	= FeetPosX_4 + BodyCenterOffsetX_4 + PosX;
DistBodyCenterFeet_4	= sqrt(pow(TotalY_4,2) + pow(TotalX_4,2));
AngleBodyCenterX_4	= PI/2 - atan2(TotalY_4, TotalX_4);
RollZ_4	= tan(RotZ * PI/180) * TotalX_4;
PitchZ_4	= tan(RotX * PI/180) * TotalY_4;
BodyIKX_4	= cos(AngleBodyCenterX_4 + (RotY * PI/180)) * DistBodyCenterFeet_4 - TotalX_4;
BodyIKY_4	= (sin(AngleBodyCenterX_4 + (RotY  * PI/180)) * DistBodyCenterFeet_4) - TotalY_4;
BodyIKZ_4	= RollZ_4 + PitchZ_4;
}

void Body5_IK(float PosY, int PosX, float FeetPosY_5, float BodyCenterOffsetY_5, float BodyCenterOffsetX_5, float FeetPosX_5, float RotY, float RotZ, float RotX  ){


float TotalY_5,TotalX_5,DistBodyCenterFeet_5,AngleBodyCenterX_5,RollZ_5,PitchZ_5,BodyIKX_5,BodyIKY_5,BodyIKZ_5;

//Leg 5
TotalY_5	= FeetPosY_5 + BodyCenterOffsetY_5 + PosY;
TotalX_5	= FeetPosX_5 + BodyCenterOffsetX_5 + PosX;
DistBodyCenterFeet_5	= sqrt(pow(TotalY_5,2) + pow(TotalX_5,2));
AngleBodyCenterX_5	= PI/2 - atan2(TotalY_5, TotalX_5);
RollZ_5	= tan(RotZ * PI/180) * TotalX_5;
PitchZ_5	= tan(RotX * PI/180) * TotalY_5;
BodyIKX_5	= cos(AngleBodyCenterX_5 + (RotY * PI/180)) * DistBodyCenterFeet_5 - TotalX_5;
BodyIKY_5	= (sin(AngleBodyCenterX_5 + (RotY  * PI/180)) * DistBodyCenterFeet_5) - TotalY_5;
BodyIKZ_5	= RollZ_5 + PitchZ_5;
}

void Body6_IK(float PosY, int PosX, float FeetPosY_6, float BodyCenterOffsetY_6, float BodyCenterOffsetX_6, float FeetPosX_6, float RotY, float RotZ, float RotX ){

float TotalY_6,TotalX_6,DistBodyCenterFeet_6,AngleBodyCenterX_6,RollZ_6,PitchZ_6,BodyIKX_6,BodyIKY_6,BodyIKZ_6;
//Leg 6
TotalY_6	= FeetPosY_6 + BodyCenterOffsetY_6 + PosY;
TotalX_6	= FeetPosX_6 + BodyCenterOffsetX_6 + PosX;
DistBodyCenterFeet_6	= sqrt(pow(TotalY_6,2) + pow(TotalX_6,2));
AngleBodyCenterX_6	= PI/2 - atan2(TotalY_6, TotalX_6);
RollZ_6	= tan(RotZ * PI/180) * TotalX_6;
PitchZ_6	= tan(RotX * PI/180) * TotalY_6;
BodyIKX_6	= cos(AngleBodyCenterX_6 + (RotY * PI/180)) * DistBodyCenterFeet_6 - TotalX_6;
BodyIKY_6	= (sin(AngleBodyCenterX_6 + (RotY  * PI/180)) * DistBodyCenterFeet_6) - TotalY_6;
BodyIKZ_6	= RollZ_6 + PitchZ_6;
}


void Leg_IK1 (float PosX, float PosY, float PosZ, float BodyIKX_1, float BodyIKZ_1, float BodyIKY_1, float FeetPosX_1, float FeetPosZ_1,float FeetPosY_1 ){

float NewPosX_1	,NewPosZ_1, NewPosY_1, CoxaFeetDist_1, IKSW_1, IKA1_1, IKA2_1, TAngle_1, IKTibiaAngle_1, IKFemurAngle_1, IKCoxaAngle_1;
//Leg 1
NewPosX_1	= FeetPosX_1 + PosX +  BodyIKX_1;
NewPosZ_1	= FeetPosZ_1 + PosZ + BodyIKZ_1;
NewPosY_1	= FeetPosY_1 + PosY + BodyIKY_1;

CoxaFeetDist_1	= sqrt(pow(NewPosX_1,2)   + pow(NewPosY_1,2));
IKSW_1	= sqrt(pow((CoxaFeetDist_1 - CoxaLength ) ,2) + pow(NewPosZ_1,2));
IKA1_1	= atan((CoxaFeetDist_1 - CoxaLength)/NewPosZ_1);
IKA2_1	= acos((pow(TibiaLength,2) - pow(FemurLength,2) - pow(IKSW_1,2))/(-2 * IKSW_1 *  FemurLength));
TAngle_1	= acos((pow(IKSW_1,2) - pow(TibiaLength,2) - pow(FemurLength,2))/(-2 * FemurLength * TibiaLength));
IKTibiaAngle_1	= 90 - TAngle_1 * 180/PI;
IKFemurAngle_1	= 90 - (IKA1_1 + IKA2_1) * 180/PI;
IKCoxaAngle_1	= 90 - atan2(NewPosY_1, NewPosX_1) * 180/PI;
}

void Leg_IK2 (float PosX, float PosY, float PosZ, float BodyIKX_2, float BodyIKZ_2, float BodyIKY_2, float FeetPosX_2, float FeetPosZ_2,float FeetPosY_2 ){


float NewPosX_2	,NewPosZ_2, NewPosY_2, CoxaFeetDist_2, IKSW_2, IKA1_2, IKA2_2, TAngle_2, IKTibiaAngle_2, IKFemurAngle_2, IKCoxaAngle_2;

//Leg 2
NewPosX_2	= FeetPosX_2 + PosX +  BodyIKX_2;
NewPosZ_2	= FeetPosZ_2 + PosZ + BodyIKZ_2;
NewPosY_2	= FeetPosY_2 + PosY + BodyIKY_2;

CoxaFeetDist_2	= sqrt(pow(NewPosX_2,2)   + pow(NewPosY_2,2));
IKSW_2	= sqrt(pow((CoxaFeetDist_2 - CoxaLength ) ,2) + pow(NewPosZ_2,2));
IKA1_2	= atan((CoxaFeetDist_2 - CoxaLength)/NewPosZ_2);
IKA2_2	= acos((pow(TibiaLength,2) - pow(FemurLength,2) - pow(IKSW_2,2))/(-2 * IKSW_2 *  FemurLength));
TAngle_2	= acos((pow(IKSW_2,2) - pow(TibiaLength,2) - pow(FemurLength,2))/(-2 * FemurLength * TibiaLength));
IKTibiaAngle_2	= 90 - TAngle_2 * 180/PI;
IKFemurAngle_2	= 90 - (IKA1_2 + IKA2_2) * 180/PI;
IKCoxaAngle_2	= 90 - atan2(NewPosY_2, NewPosX_2) * 180/PI;
}

void Leg_IK3 (float PosX, float PosY, float PosZ, float BodyIKX_3, float BodyIKZ_3, float BodyIKY_3, float FeetPosX_3, float FeetPosZ_3,float FeetPosY_3 ){

float NewPosX_3	,NewPosZ_3, NewPosY_3, CoxaFeetDist_3, IKSW_3, IKA1_3, IKA2_3, TAngle_3, IKTibiaAngle_3, IKFemurAngle_3, IKCoxaAngle_3;

//Leg 3
NewPosX_3	= FeetPosX_3 + PosX +  BodyIKX_3;
NewPosZ_3	= FeetPosZ_3 + PosZ + BodyIKZ_3;
NewPosY_3	= FeetPosY_3 + PosY + BodyIKY_3;

CoxaFeetDist_3	= sqrt(pow(NewPosX_3,2)   + pow(NewPosY_3,2));
IKSW_3	= sqrt(pow((CoxaFeetDist_3 - CoxaLength ) ,2) + pow(NewPosZ_3,2));
IKA1_3	= atan((CoxaFeetDist_3 - CoxaLength)/NewPosZ_3);
IKA2_3	= acos((pow(TibiaLength,2) - pow(FemurLength,2) - pow(IKSW_3,2))/(-2 * IKSW_3 *  FemurLength));
TAngle_3	= acos((pow(IKSW_3,2) - pow(TibiaLength,2) - pow(FemurLength,2))/(-2 * FemurLength * TibiaLength));
IKTibiaAngle_3	= 90 - TAngle_3 * 180/PI;
IKFemurAngle_3	= 90 - (IKA1_3 + IKA2_3) * 180/PI;
IKCoxaAngle_3	= 90 - atan2(NewPosY_3, NewPosX_3) * 180/PI;
}

void Leg_IK4 (float PosX, float PosY, float PosZ, float BodyIKX_4, float BodyIKZ_4, float BodyIKY_4, float FeetPosX_4, float FeetPosZ_4,float FeetPosY_4 ){

float NewPosX_4	,NewPosZ_4, NewPosY_4, CoxaFeetDist_4, IKSW_4, IKA1_4, IKA2_4, TAngle_4, IKTibiaAngle_4, IKFemurAngle_4, IKCoxaAngle_4;

//Leg 4
NewPosX_4	= FeetPosX_4 + PosX +  BodyIKX_4;
NewPosZ_4	= FeetPosZ_4 + PosZ + BodyIKZ_4;
NewPosY_4	= FeetPosY_4 + PosY + BodyIKY_4;

CoxaFeetDist_4	= sqrt(pow(NewPosX_4,2)   + pow(NewPosY_4,2));
IKSW_4	= sqrt(pow((CoxaFeetDist_4 - CoxaLength ) ,2) + pow(NewPosZ_4,2));
IKA1_4	= atan((CoxaFeetDist_4 - CoxaLength)/NewPosZ_4);
IKA2_4	= acos((pow(TibiaLength,2) - pow(FemurLength,2) - pow(IKSW_4,2))/(-2 * IKSW_4 *  FemurLength));
TAngle_4	= acos((pow(IKSW_4,2) - pow(TibiaLength,2) - pow(FemurLength,2))/(-2 * FemurLength * TibiaLength));
IKTibiaAngle_4	= 90 - TAngle_4 * 180/PI;
IKFemurAngle_4	= 90 - (IKA1_4 + IKA2_4) * 180/PI;
IKCoxaAngle_4	= 90 - atan2(NewPosY_4, NewPosX_4) * 180/PI;
}

void Leg_IK5 (float PosX, float PosY, float PosZ, float BodyIKX_5, float BodyIKZ_5, float BodyIKY_5, float FeetPosX_5, float FeetPosZ_5,float FeetPosY_5){

float NewPosX_5	,NewPosZ_5, NewPosY_5, CoxaFeetDist_5, IKSW_5, IKA1_5, IKA2_5, TAngle_5, IKTibiaAngle_5, IKFemurAngle_5, IKCoxaAngle_5;

//Leg 5
NewPosX_5	= FeetPosX_5 + PosX +  BodyIKX_5;
NewPosZ_5	= FeetPosZ_5 + PosZ + BodyIKZ_5;
NewPosY_5	= FeetPosY_5 + PosY + BodyIKY_5;

CoxaFeetDist_5	= sqrt(pow(NewPosX_5,2)   + pow(NewPosY_5,2));
IKSW_5	= sqrt(pow((CoxaFeetDist_5 - CoxaLength ) ,2) + pow(NewPosZ_5,2));
IKA1_5	= atan((CoxaFeetDist_5 - CoxaLength)/NewPosZ_5);
IKA2_5	= acos((pow(TibiaLength,2) - pow(FemurLength,2) - pow(IKSW_5,2))/(-2 * IKSW_5 *  FemurLength));
TAngle_5	= acos((pow(IKSW_5,2) - pow(TibiaLength,2) - pow(FemurLength,2))/(-2 * FemurLength * TibiaLength));
IKTibiaAngle_5	= 90 - TAngle_5 * 180/PI;
IKFemurAngle_5	= 90 - (IKA1_5 + IKA2_5) * 180/PI;
IKCoxaAngle_5	= 90 - atan2(NewPosY_5, NewPosX_5) * 180/PI;
}

void Leg_IK6 (float PosX, float PosY, float PosZ, float BodyIKX_6, float BodyIKZ_6, float BodyIKY_6, float FeetPosX_6, float FeetPosZ_6,float FeetPosY_6 ){

float NewPosX_6	,NewPosZ_6, NewPosY_6, CoxaFeetDist_6, IKSW_6, IKA1_6, IKA2_6, TAngle_6, IKTibiaAngle_6, IKFemurAngle_6, IKCoxaAngle_6;
//Leg 6
NewPosX_6	= FeetPosX_6 + PosX +  BodyIKX_6;
NewPosZ_6	= FeetPosZ_6 + PosZ + BodyIKZ_6;
NewPosY_6	= FeetPosY_6 + PosY + BodyIKY_6;

CoxaFeetDist_6	= sqrt(pow(NewPosX_6,2)   + pow(NewPosY_6,2));
IKSW_6	= sqrt(pow((CoxaFeetDist_6 - CoxaLength ) ,2) +pow( NewPosZ_6,2));
IKA1_6	= atan((CoxaFeetDist_6 - CoxaLength)/NewPosZ_6);
IKA2_6	= acos((pow(TibiaLength,2) - pow(FemurLength,2) - pow(IKSW_6,2))/(-2 * IKSW_6 *  FemurLength));
TAngle_6	= acos((pow(IKSW_6,2) - pow(TibiaLength,2) - pow(FemurLength,2))/(-2 * FemurLength * TibiaLength));
IKTibiaAngle_6	= 90 - TAngle_6 * 180/PI;
IKFemurAngle_6	= 90 - (IKA1_6 + IKA2_6) * 180/PI;
IKCoxaAngle_6	= 90 - atan2(NewPosY_6, NewPosX_6) * 180/PI;

}


 void Servo1_Angles (float IKCoxaAngle_1,float IKFemurAngle_1,float IKTibiaAngle_1){
   float CoxaAngle_1,FemurAngle_1,TibiaAngle_1;
   //Leg 1
   CoxaAngle_1	= IKCoxaAngle_1 - 60;
   FemurAngle_1	= IKFemurAngle_1;
   TibiaAngle_1	= IKTibiaAngle_1;
 }

 void Servo2_Angles (float IKCoxaAngle_2,float IKFemurAngle_2,float IKTibiaAngle_2){
   float CoxaAngle_2,FemurAngle_2,TibiaAngle_2;
   //Leg 2
   CoxaAngle_2	= IKCoxaAngle_2;
   FemurAngle_2	= IKFemurAngle_2;
   TibiaAngle_2	= IKTibiaAngle_2;
 }

void Servo3_Angles (float IKCoxaAngle_3,float IKFemurAngle_3,float IKTibiaAngle_3){
  float CoxaAngle_3,FemurAngle_3,TibiaAngle_3;
   //Leg 3
   CoxaAngle_3	= IKCoxaAngle_3 + 60;
   FemurAngle_3	= IKFemurAngle_3;
   TibiaAngle_3	= IKTibiaAngle_3;
}

void Servo4_Angles (float IKCoxaAngle_4,float IKFemurAngle_4,float IKTibiaAngle_4){
  float CoxaAngle_4,FemurAngle_4,TibiaAngle_4;
   //Leg 4
   CoxaAngle_4	= IKCoxaAngle_4 - 240;
   FemurAngle_4	= IKFemurAngle_4;
   TibiaAngle_4	= IKTibiaAngle_4;
}

void Servo5_Angles (float IKCoxaAngle_5,float IKFemurAngle_5,float IKTibiaAngle_5){
  float CoxaAngle_5,FemurAngle_5,TibiaAngle_5;
   //Leg 5
   CoxaAngle_5	= IKCoxaAngle_5 - 180;
   FemurAngle_5	= IKFemurAngle_5;
   TibiaAngle_5	= IKTibiaAngle_5;
}

void Servo6_Angles (float IKCoxaAngle_6,float IKFemurAngle_6,float IKTibiaAngle_6){
  float CoxaAngle_6,FemurAngle_6,TibiaAngle_6;
   //Leg 6
   CoxaAngle_6	= IKCoxaAngle_6 - 120;
   FemurAngle_6	= IKFemurAngle_6;
   TibiaAngle_6	= IKTibiaAngle_6;

}

/* coordinate_to_degrees(x, y): # function to convert coordinates to angles from the x-axis (0~360)
    x += 0.00001 # this is to avoid zero division error in case x == 0
 
    if x >= 0 and y >= 0:   # first quadrant
        angle = degrees(atan(y/x))
    elif x < 0 and y >= 0:  # second quadrant
        angle = 180 + degrees(atan(y/x))
    elif x < 0 and y < 0:   # third quadrant
        angle = 180 + degrees(atan(y/x))
    elif x >= 0 and y < 0:  # forth quadrant
        angle = 360 + degrees(atan(y/x))
    return round(angle,1)*/


/*cinematica_inversa(){
   // Coxa, Femur e Tibia são as medidas de comprimento
   // x, y e z são as coordenadas
   //alpha, beta, gama são os ângulos das juntas
   //L é o comprimento entre a junta 2 e a ponta da pata
   //H é a projeção do seguimento L no plano XY

   H=sqrt(pow(x,2) + pow(y,2))-coxa;
   L = sqrt(pow(H,2) + pow(z,2));

   alpha = (atan(x/y))*(180.00/PI); //angulo do servo.coxa
   //caso o valor da tangente seja positivo não se pode distinguir se o ângulo é do primeiro ou terceiro quadrante e se for negativo, se é do segundo ou quarto quadrante
   
   gama = ((acos((pow(femur,2) + pow(tibia,2) - pow(L,2))/(2*femur*tibia)))*(180.00/PI)); //angulo do servo.tibia
   beta1 = (acos((pow(femur,2) + pow(L,2) - pow(tibia,2))/(2*femur*L)))*(180.00/PI); 
   beta = (alpha + beta1);//angulo do servo.femur

}*/

/*cinematica_direta(){

//Coxa é o comprimento do elo que representa a coxa da perna do hexápode;
//Fémur é o comprimento do elo que representa o fémur da perna do hexápode;
//Tíbia é o comprimento do elo que representa a tíbia da perna do hexápode;
//Ac representa a altura do corpo do robô ao solo;
//γ é o valor angular da junta rotacional que representa a anca do hexápode;
//α é o valor angular da junta rotacional que representa o joelho do hexápode;
//β é o valor angular da junta rotacional que representa o tornozelo do hexápode.

  x= (coxa+femur*cos(alpha)+tibia*cos(alpha+beta))*cos(gama)
  y= (coxa+femur*cos(alpha)+tibia*cos(alpha+beta))*sen(gama)
  z= Ac+femur*sen(alpha)+tibia*sen(alpha+beta) 
  }*/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  servotibia.attach(9);  
  servofemur.attach(10);  
  servocoxa.attach(11);  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("30");// You can display on the serial the signal value
    servotibia.write(90); //Turn clockwise at high speed
  

}