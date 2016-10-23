#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>
#include <pgmspace.h>
#include <ESP8266WiFi.h>

#include <ESP_Multi_Board.h>
#include "speedController.h"
#include "telemetry.h"

extern "C" {
#include "user_interface.h"
}


#define ir_weight_straight  0.5


/**
 * Control variables
 */
 extern int targetSpeedX;
 extern int targetSpeedW;

/**
 * Output variable. To be readed
 */
 extern long distanceLeft;
 extern long encoderCount;
 extern float curSpeedX;
 extern float curSpeedW;
 extern float accX;
 extern float decX;
 extern float accW;
 extern float decW;

/**
 * Tune variables
 */
 extern float ir_weight;
 extern float kpX, kdX;
 extern float kpW, kdW; //used in straight
 extern float kpW0, kdW0; //used in straight
// extern float kpWir, kdWir;//used with IR errors

//#define abs(x)  (x<0)?(-x):(x)

//speed to count or count to speed are the macro or function to make the unit conversion
// between encoder_counts/ms and mm/ms or any practical units you use.
int moveSpeed = speed_to_counts(0);
int turnSpeed = speed_to_counts(13);
int returnSpeed = speed_to_counts(500*2);
int stopSpeed = speed_to_counts(100*2);
int maxSpeed = speed_to_counts(15);



long oldEncoderCount=0;

ESP_Multi_Board robot;

extern float **telemetria;
extern int p_telemetria;

os_timer_t timerSpeedController;


float needToDecelerate(long dist, int curSpd, int endSpd)//speed are in encoder counts/ms, dist is in encoder counts
{
    if (curSpd<0) curSpd = -curSpd;
    if (endSpd<0) endSpd = -endSpd;
    if (dist<0) dist = 1;//-dist;
    if (dist == 0) dist = 1;  //prevent divide by 0

    return (abs(counts_to_speed((curSpd*curSpd - endSpd*endSpd)*100/(double)dist/4/2))); //dist_counts_to_mm(dist)/2);
    //calculate deceleration rate needed with input distance, input current speed and input targetspeed to determind if the deceleration is needed
    //use equation 2*a*S = Vt^2 - V0^2  ==>  a = (Vt^2-V0^2)/2/S
    //because the speed is the sum of left and right wheels(which means it's doubled), that's why there is a "/4" in equation since the square of 2 is 4
}

/*
sample code for straight movement
*/
void moveOneCell()
{
    targetSpeedW = 0;
    targetSpeedX = moveSpeed;
    kpW = kpW0;
    kdW = kdW0;
    ir_weight = ir_weight_straight; // usar los IR para alinearte con la pared
    do
    {
        /*you can call int needToDecelerate(int32_t dist, int16_t curSpd, int16_t endSpd)
        here with current speed and distanceLeft to decide if you should start to decelerate or not.*/
        /*sample*/
        float accMin = needToDecelerate(ONE_CELL_DISTANCE-(encoderCount-oldEncoderCount), curSpeedX, moveSpeed);

        if( accMin < decX)
            targetSpeedX = maxSpeed;
        else
            targetSpeedX = moveSpeed;

//        Serial.print(millis());Serial.print(" ");
//        Serial.print(ONE_CELL_DISTANCE-(encoderCount-oldEncoderCount));Serial.print(" ");
//        Serial.print(curSpeedX);Serial.print(" ");
//        Serial.print(moveSpeed);Serial.print(" ");
//        Serial.print(accMin);Serial.print(" ");
//        Serial.print(decX);Serial.print(" ");
//        Serial.print(targetSpeedX);Serial.println(" ");
        delay(5);
        //there is something else you can add here. Such as detecting falling edge of post to correct longitudinal position of mouse when running in a straight path
        //Serial.print(targetSpeedX);Serial.print(" ");Serial.print((encoderCount-oldEncoderCount));Serial.print(" ");Serial.print(ONE_CELL_DISTANCE);Serial.println("");
     }
    while( ( (encoderCount-oldEncoderCount) < ONE_CELL_DISTANCE )//&& LFSensor < LFvalue2 && RFSensor < RFvalue2)//use encoder to finish 180mm movement if no front walls
    //|| (LFSensor < LFvalues1 && LFSensor > LFvalue2)//if has front wall, make the mouse finish this 180mm with sensor threshold only
    //|| (RFSensor < RFvalue1 && RFSensor > RFvalve2)
    );
    //LFvalues1 and RFvalues1 are the front wall sensor threshold when the center of mouse between the boundary of the cells.
    //LFvalues2 and RFvalues2 are the front wall sensor threshold when the center of the mouse staying half cell farther than LFvalues1 and 2
    //and LF/RFvalues2 are usually the threshold to determine if there is a front wall or not. You should probably move this 10mm closer to front wall when collecting
    //these thresholds just in case the readings are too weak.

//    Serial.print("Final Encoder Count");Serial.println((encoderCount-oldEncoderCount));
    oldEncoderCount = encoderCount; //update here for next movement to minimized the counts loss between cells.
}


/**
 * Ejecuta un giro de 90º partiendo de la entrada de la celda y acabando en la entrada de la celda a der o izq.
 *  *******      *  ^  *
 *   >    *       >    *
 *  *  v  *      ******* 
 *  
 *  Para realzar el giro se configura el controlador de velocidad para generar las curvas de velocidades lineales 
 *  y angulares deseadas que se componen de 3 regiones:
 *    t1 -> region de acceleracion de velocidad angular 
 *    t2 -> region de velocidad constante angular 
 *    t3 -> region de deceleracion angular
 *   
 *   Explicacion detallada: http://micromouseusa.com/?page_id=1342  (Lecture 7)
 * 
 */
void turn90(int dir){
  float velW=7.6*dir;
  long t1 = (speed_to_counts(abs(velW))/accW)*25;
  long t2 = 510;
  long t3 = (speed_to_counts(abs(velW))/decW)*25;
  long tini = millis();
  ir_weight = 0; //no usar los IR para alinearte con la pared cuando estas girando

  
  targetSpeedX = turnSpeed;
  targetSpeedW = speed_to_counts(velW);
  kpW = kpW1;
  kdW = kdW1;
  while(millis()-tini < t1){delay(1);}
  tini = millis();
  kpW = kpW2;
  kdW = kdW2;
  while(millis()-tini < t2){delay(1);}
  tini = millis();
  kpW = kpW1;
  kdW = kdW1;
  targetSpeedW = 0;
  targetSpeedX = targetSpeedX/5;
  while(millis()-tini < t3){delay(1);}
  targetSpeedX = 0;
  kpW = kpW0;
  kdW = kdW0;
  oldEncoderCount = encoderCount;
  resetGyro();
}

void avanzarCorreccionTrasGiro(int dist){
   resetGyro();
   targetSpeedW = 0;
   targetSpeedX = speed_to_counts(15);
   kpW = kpW0;
   kdW = kdW0;
   ir_weight = 0.0; //solo ir recto con el gyro (casi)
   //while(encoderCount - oldEncoderCount > dist)
    delay(100);
   oldEncoderCount = encoderCount;
   targetSpeedX = 0;
   resetGyro();
}


/**
 * Ejecuta un giro de 180º y se coloca en una pocisión validad para inciar la marcha en la siguiente celda
 * 
 * TODO  (no se encuentra calibrada)
 * 
 */
void turn180(int dir){
  float velW=16*dir;
  long t1 = (speed_to_counts(abs(velW))/accW)*25;
  long t2 =310;
  long t3 = (speed_to_counts(abs(velW))/decW)*25;
  long tini = millis();
  ir_weight = 0; //no usar los IR para alinearte con la pared cuando estas girando
  targetSpeedX = 0;
  targetSpeedW = speed_to_counts(velW);
  kpW = kpW1;
  kdW = kdW1;
  while(millis()-tini < t1){delay(1);}
  tini = millis();
  kpW = kpW2;
  kdW = kdW2;
  while(millis()-tini < t2){delay(1);}
  tini = millis();
  kpW = kpW1;
  kdW = kdW1;
  targetSpeedW = 0;
  targetSpeedX = 0;
  while(millis()-tini < t3){delay(1);}
  targetSpeedX = 0;
  kpW = kpW0;
  kdW = kdW0;
  oldEncoderCount = encoderCount;
  resetGyro();
}




void setup() {
  // put your setup code here, to run once:
  delay(500);

  Serial.begin(115200);
  //init_telnet();

  robot.begin();
  resetSpeedProfile();
   os_timer_setfn(&timerSpeedController, speedProfile, NULL);
   os_timer_arm(&timerSpeedController, 25, true);
  // resetSpeedProfile();
}




/*
#define TIME_BT_MOVE 10

void loopChino() {
  moveOneCell();
  delay(TIME_BT_MOVE);
  moveOneCell();
  delay(TIME_BT_MOVE);
  moveOneCell();
  delay(TIME_BT_MOVE);
  moveOneCell();
  delay(TIME_BT_MOVE);
  turn90(1);
  delay(TIME_BT_MOVE);
  turn90(1);
  delay(TIME_BT_MOVE);

  
//check_send_telnet_telemetry();
  delay(2000);



//  for(int i=0;i<p_telemetria;i++){
//    for(int j=0;j<5;j++){
//      Serial.print(telemetria[i][j]);
//      Serial.print(" ");
//    }
//    Serial.println("");
//  }
//  p_telemetria=0;


}
*/


int leerPared_old(){
    uint8_t IR_value[3]={0};
    leerIRs(IR_value);

    if(IR_value[0]<70){
      return 1;
    }else if(IR_value[1]<37){
      return 0;
    }else if(IR_value[2]<70){
      return -1;
    }
    return 2;
}

int leerPared(){
    double IR_dist[3]={0};
    leerDist(IR_dist);

    if(IR_dist[0]>53){
      return 1;
    }else if(IR_dist[1]<55){
      return 2;
    }else if(IR_dist[2]>53){
      return -1;
    }
    return 0;
}
inline bool leerHuecoFrontal(){
  double IR_dist[3]={0};
  leerDist(IR_dist);
  return (IR_dist[1] > 125); //105
}
inline bool leerPilarLateral(int numSen){
  double IR_dist[3]={0};
  leerDist(IR_dist);
  return (IR_dist[numSen] <60);
}

void loop(){
  
   targetSpeedW = 0;
   targetSpeedX = speed_to_counts(15);
   kpW = kpW0;
   kdW = kdW0;
   ir_weight = ir_weight_straight; // usar los IR para alinearte con la pared
   
  int pared=leerPared();
  if( pared!=0){
    resetGyro();
    targetSpeedX = speed_to_counts(0);
    delay(300);
    if(pared==1){//giro a la izq
      turn90(1);
      avanzarCorreccionTrasGiro(10);
      delay(1000);
    }else if(pared==-1){ //Compruebo si hay hueco al frente o no
      if( !leerHuecoFrontal()){ //si no hay hueco frontal giro a la der
        turn90(-1);
        avanzarCorreccionTrasGiro(50);
      }else{
        //sigo de frente hasta encontrar el siguiente pilar en la pared derecha
           targetSpeedW = 0;
           targetSpeedX = speed_to_counts(15);
           kpW = kpW0;
           kdW = kdW0;
           ir_weight = ir_weight_straight; // usar los IR para alinearte con la pared
           while( !leerPilarLateral(2) )delay(50); //Mientras que no encuentre la pared de nuevo (un pilar) en la derecha -> avanzo
      }
     }else if(pared==2){ //punto ciego tengo girar 180º
      targetSpeedX = 0;
      delay(500);
      turn180(1);
     }
    targetSpeedX = 0;
    delay(300);
  }
}

