#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include "speedController.h"

#include <ESP_Multi_Board.h>

//public
/**
 * Control variables
 */
int targetSpeedX;
int targetSpeedW;

/**
 * Output variable. To be readed
 */
  long distanceLeft;
  long encoderCount;
  float curSpeedX;
  float curSpeedW;
  float accX;
  float decX;
  float accW;
  float decW;

/**
 * Tune variables
 */
  float ir_weight;
  float kpX, kdX;
  float kpW, kdW; //used in straight
  float kpW0, kdW0; //used in straight
  float kpWir, kdWir;




//private
int encoderFeedbackX = 0;
int encoderFeedbackW = 0;
float pidInputX = 0;
float pidInputW = 0;
float posErrorX = 0;
float posErrorW = 0;
float posErrorWir = 0;
float oldPosErrorX = 0;
float oldPosErrorW = 0;
float oldPosErrorWir = 0;
int posPwmX = 0;
int posPwmW = 0;
int posPwmWir = 0;

#define TARGET_WALL_DIST 70




  
long encoderChange=0;
long leftEncoderChange=0;
long rightEncoderChange=0;
long leftEncoderOld = 0;
long rightEncoderOld = 0;
long leftEncoder = 0;
long  rightEncoder = 0;
long leftEncoderCount = 0;
long rightEncoderCount = 0;
    
int leftBaseSpeed = 0;
int rightBaseSpeed = 0;

float gyroFeedbackRatio = -223.5;//5900;


extern float **telemetria;
extern int p_telemetria;

void speedProfile(void *a){ 
    getEncoderStatus(); 
    updateCurrentSpeed(); 
    calculateMotorPwm(); 
}


void getEncoderStatus()
{
    long leftEncoder = robot.getEncLeftCount(); //leftEncoder = TIM2->CNT;//read current encoder ticks from register of 32 bit general purpose timer 2
    long rightEncoder = robot.getEncRightCount(); //rightEncoder = TIM5->CNT;//read current encoder ticks from register of 32 bit general purpose timer 5

    leftEncoderChange = leftEncoder - leftEncoderOld;
    rightEncoderChange = rightEncoder - rightEncoderOld;
    encoderChange = (leftEncoderChange + rightEncoderChange)/2;

    leftEncoderOld = leftEncoder;
    rightEncoderOld = rightEncoder;

    leftEncoderCount += leftEncoderChange;
    rightEncoderCount += rightEncoderChange;
    encoderCount =  (leftEncoderCount+rightEncoderCount)/2;

    distanceLeft -= encoderChange;// update distanceLeft
}



void updateCurrentSpeed(void)
{
    if(curSpeedX < targetSpeedX && curSpeedX >= 0){
        curSpeedX += (float)(speed_to_counts(accX*2)/100);
        if(curSpeedX > targetSpeedX)
            curSpeedX = targetSpeedX;
    }else if(curSpeedX > targetSpeedX && curSpeedX > 0){
        curSpeedX -= (float)speed_to_counts(decX*2)/100;
        if(curSpeedX < targetSpeedX)
            curSpeedX = targetSpeedX;
    }else if(curSpeedX < targetSpeedX && curSpeedX < 0){
        curSpeedX += (float)speed_to_counts(decX*2)/100;
        if(curSpeedX > targetSpeedX)
            curSpeedX = targetSpeedX;
    }else if(curSpeedX > targetSpeedX && targetSpeedX <= 0){
        curSpeedX -= (float)(speed_to_counts(accX*2)/100);
        if(curSpeedX < targetSpeedX)
            curSpeedX = targetSpeedX;
    }
    
    if(curSpeedW < targetSpeedW && curSpeedW >= 0){
        curSpeedW += accW;
        if(curSpeedW > targetSpeedW)
            curSpeedW = targetSpeedW;
    }else if(curSpeedW > targetSpeedW && curSpeedW > 0){
        curSpeedW -= decW;
        if(curSpeedW < targetSpeedW)
            curSpeedW = targetSpeedW;
    }else if(curSpeedW < targetSpeedW && curSpeedW < 0){
        curSpeedW += decW;
        if(curSpeedW > targetSpeedW)
            curSpeedW = targetSpeedW;
    }else if(curSpeedW > targetSpeedW && curSpeedW <= 0){
        curSpeedW -= accW;
        if(curSpeedW < targetSpeedW)
            curSpeedW = targetSpeedW;
    }
    
}

void calculateMotorPwm(void) // encoder PD controller
{
    int sensorFeedback;
    
    //Serial.println(IR_value[0]);
    /* simple PD loop to generate base speed for both motors */
    encoderFeedbackX = rightEncoderChange + leftEncoderChange;
    encoderFeedbackW = rightEncoderChange - leftEncoderChange;
    float gyro = robot.read_IMU_GyZ()*1.0;
    float gyroFeedback = gyro / gyroFeedbackRatio; //gyroFeedbackRatio mentioned in curve turn lecture
    //gyroIntegrate += gyroFeedback;
    //sensorFeedback = sensorError/a_scale;//have sensor error properly scale to fit the system

    //if(onlyUseGyroFeedback)
    //    rotationalFeedback = gyroFeedback;
    //else if(onlyUseEncoderFeedback)
          float rotationalFeedback = /*((float)(ENCODER_GYRO_WEIGHT*1.0f)*(encoderFeedbackW*1.0f)) + ((float)(1.0f - ENCODER_GYRO_WEIGHT*1.0)*(*/gyroFeedback;//));
    //else
    //    rotationalFeedback = encoderFeedbackW + gyroFeedback;
        //if you use IR sensor as well, the line above will be rotationalFeedback = encoderFeedbackW + gyroFeedback + sensorFeedback;
        //make sure to check the sign of sensor error.
  
    posErrorX += curSpeedX - encoderFeedbackX;
    posErrorW += curSpeedW - rotationalFeedback;
   


    getErrIRChino();
    Serial.print(posErrorWir);

    /*if(posErrorWir < 20){
      posErrorW = 0;
    }*/

    posPwmX = kpX * posErrorX + kdX * (posErrorX - oldPosErrorX);
    posPwmW = kpW * posErrorW + kdW * (posErrorW - oldPosErrorW);
    posPwmWir = kpWir * posErrorWir + kdWir * (posErrorWir - oldPosErrorWir); 

    Serial.print("-");
    Serial.print(posPwmWir);

    oldPosErrorX = posErrorX;
    oldPosErrorW = posErrorW;
    oldPosErrorWir = posErrorWir;

    leftBaseSpeed = posPwmX - ( (1-ir_weight)*posPwmW + ir_weight*posPwmWir);
    rightBaseSpeed = posPwmX + ( (1-ir_weight)*posPwmW + ir_weight*posPwmWir);


    Serial.print("-");
    Serial.println((rightBaseSpeed-leftBaseSpeed)/2);
    robot.setMotorLeftSpeed(leftBaseSpeed);
    robot.setMotorRightSpeed(rightBaseSpeed);
    /*if(p_telemetria<SIZE_TELEMETRIA){
      telemetria[p_telemetria][0]=micros();
      telemetria[p_telemetria][1]=curSpeedX;
      telemetria[p_telemetria][2]=encoderFeedbackX;
      //telemetria[p_telemetria][3]=curSpeedW;
      telemetria[p_telemetria][3]=curSpeedW;
      telemetria[p_telemetria][4]=rotationalFeedback;
      telemetria[p_telemetria][5]=posErrorWir;
      //telemetria[p_telemetria][6]=leftEncoderChange;
      p_telemetria++;
    }*/
    
}

void leerIRs(uint8_t *data){
  uint8_t IR_value1[3]={0};
  uint8_t IR_value2[3]={0};
  robot.analogScand(3, IR_value1); 
  robot.digitalWrite(0,HIGH);
  robot.analogScand(3, IR_value2);
  robot.digitalWrite(0,LOW);
  for(int i=0;i<3;i++)
    data[i] = (IR_value2[i] - IR_value1[i]);
  
}

float mapf(long x, long in_min, long in_max, long out_min, long out_max)
{
 return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

void leerDist(double *res){
  
  uint8_t IR_value1[3]={0};
  uint8_t IR_value2[3]={0};
  uint8_t cal[3][21]={{236,226,184,120,83,64,49,39,32,27,23  ,20,17,15,14,13,12,11,10,8,7},
                      {245,237,231,198,121,85,62,48,38,31,25, 21,18,15,13,12,11,10,9,8,7},
                      {236,233,218,177,129,99,77,61,50,42,35,30,26,23,20,18, 15, 14, 13,12,11}};
  
  robot.analogScand(3, IR_value1); 
  robot.digitalWrite(0,HIGH);
  robot.analogScand(3, IR_value2);
  robot.digitalWrite(0,LOW);
  for(int i=0;i<3;i++){
    uint8_t data = (IR_value2[i] - IR_value1[i]);
    for(int j=1;j<10;j++){
      if(data >= cal[i][j]){
        res[i]=mapf(data,cal[i][j],cal[i][j-1],j*10,(j-1)*10);
        break;
      }
      res[i]=mapf(data,cal[i][20],0,200,1000);
    }
  }
}

inline void getErrIRNew(){
    int DLMiddleValue=101;
    int DRMiddleValue=170;
    int DLMinValue=60;
    int DRMinValue=76;
    uint8_t IR_value[3]={0};
    leerIRs(IR_value);
    //telemetria[p_telemetria][6]=IR_value[0];
    //telemetria[p_telemetria][7]=IR_value[2];

    if( abs(DLMiddleValue - IR_value[0]) < abs(IR_value[2] - DRMiddleValue) ){ //pared izqueirda mas cercana que la derecha, la utilizo para ir por el centro
      if(IR_value[0] > DLMinValue) //pared detectada por encima del umbral minimo 
        posErrorWir = DLMiddleValue - IR_value[0];
      else //pared no detectada
        posErrorWir /= 3; 
    }else{
      if(IR_value[2] > DRMinValue )
        posErrorWir = IR_value[2] - DRMiddleValue;
      else
        posErrorWir /= 3;
    }
}

inline void getErrIRChino(){
    //int DLMiddleValue=101;
    //int DRMiddleValue=170;
    //int DLMinValue=60;
    //int DRMinValue=76;

    int DLMiddleValue=215;
    int DRMiddleValue=225;
    int DLMinValue=170;
    int DRMinValue=185;
    
    uint8_t IR_value[3]={0};
    leerIRs(IR_value);
    
    //telemetria[p_telemetria][6]=IR_value[0];
    //telemetria[p_telemetria][7]=IR_value[2];

    if(IR_value[0] > DLMinValue || IR_value[2] > DRMinValue){
      if(IR_value[0] > IR_value[2]){
        posErrorWir = DLMiddleValue - IR_value[0];
      }else{
        posErrorWir = IR_value[2] - DRMiddleValue;
      }
    }else{
      posErrorWir=0;
    }
    
}

inline void getErrIR(){
    //int DLMiddleValue=101;
    //int DRMiddleValue=170;
    //int DLMinValue=60;
    //int DRMinValue=76;

    int DLMiddleValue=215;
    int DRMiddleValue=225;
    int DLMinValue=170;
    int DRMinValue=185;
    
    uint8_t IR_value[3]={0};
    leerIRs(IR_value);
    
    //telemetria[p_telemetria][6]=IR_value[0];
    //telemetria[p_telemetria][7]=IR_value[2];
    
    if(IR_value[0] > DLMinValue && IR_value[2] < DRMinValue){
      posErrorWir = DLMiddleValue - IR_value[0];
      if(IR_value[0] < DLMiddleValue)
        posErrorWir /=3;
    }else if(IR_value[2] > DRMinValue && IR_value[0] < DLMinValue){
      posErrorWir = IR_value[2] - DRMiddleValue;
      if(IR_value[2] < DRMiddleValue)
        posErrorWir /=3;
    }else if(IR_value[0] > DLMinValue && IR_value[2] > DRMinValue){
      posErrorWir = 0;
      
      if(IR_value[0] < DLMiddleValue)
        posErrorWir += (DLMiddleValue - IR_value[0])/3;
      else 
        posErrorWir += (DLMiddleValue - IR_value[0]);
      
      if(IR_value[2] < DRMiddleValue)
        posErrorWir += (IR_value[2] - DRMiddleValue)/3;
      else
        posErrorWir += (IR_value[2] - DRMiddleValue);
        
      posErrorWir /= 2;
      
    }else
      posErrorWir /= 3;
}






void resetSpeedProfile(void)
{
    //reset Everything and reTune parameters;


    robot.setSpeed(0,0);//setLeftPwm(0);//setRightPwm(0);

  //Tune parameters
   ir_weight = 0.0;
   kpX = 0.95; kdX = 10;
   kpW = 0.8; kdW = 17;//used in straight
   kpW0 = kpW; kdW0 = kdW;//used in straight
   kpWir = 8.05; kdWir = 39;//used with IR errors

   accX = 45;//6m/s/s
   decX = 90;
   accW = 1; //cm/s^2
   decW = 1;


//reset variables
    pidInputX = 0;
    pidInputW = 0;
    curSpeedX = 0;
    curSpeedW = 0;
    targetSpeedX = 0;
    targetSpeedW = 0;
    posErrorX = 0;
    posErrorW = 0;
    oldPosErrorX = 0;
    oldPosErrorW = 0;
    leftEncoderOld = 0;
    rightEncoderOld = 0;
    leftEncoder = 0;
    rightEncoder = 0;
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    encoderCount = 0;
    leftBaseSpeed = 0;
    rightBaseSpeed = 0;

    robot.resetEncoders();//reset left encoder count //TIM5->CNT = 0;//reset right encoder count
}
