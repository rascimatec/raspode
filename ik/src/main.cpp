#include <Arduino.h>
#include <Servo.h> //Servo library
#include <math.h>

#define tibia 145.00
#define femur 95.00
#define coxa 36.00

Servo servotibia;
Servo servofemur;
Servo servocoxa;
int beta,gama,alpha, beta1;
float x,y,z,H,L;

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


/*ik(){
   // Coxa, Femur e Tibia são as medidas de comprimento
   // x, y e z são as coordenadas
   //alpha, beta, gama são os ângulos das juntas
   //L é o comprimento entre a junta 2 e a ponta da pata
   //H é a projeção do seguimento L no plano XY

   H=sqrt(pow(x,2) + pow(y,2))-coxa;
   L = sqrt(pow(H,2) + pow(z,2));

   alpha = (atan(x/y))*(180.00/PI); //angulo do servo.coxa
   gama = ((acos((pow(femur,2) + pow(tibia,2) - pow(L,2))/(2*femur*tibia)))*(180.00/PI)); //angulo do servo.tibia
   beta1 = (acos((pow(femur,2) + pow(L,2) - pow(tibia,2))/(2*femur*L)))*(180.00/PI); 
   beta = (alpha + beta1);//angulo do servo.femur

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
  //for(pos=0;pos<=180;pos++){
    servotibia.write(1); //Turn clockwise at high speed
  //}
  //for(pos=180;pos>=0;pos--){
    //servotibia.write(pos); //Turn clockwise at high speed
  //}

}