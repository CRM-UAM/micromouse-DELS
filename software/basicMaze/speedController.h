
//call speedProfile(void) in systick handle to make it execute every one milli second
//this controller sample code doesn't include

//here are the code you need to call periodically to make the mouse sample and excute and a constant rate
//in order to make sample and pwm updating at a constant rate, you'd better call the code in the handler of a timer periodically
//the period for intterupt handler is usually 1ms, which means you have to finish all code excuting within 1ms
//I simply call the controller code in systick handler that intterupts every 1ms, notice systick timer is also used to
//keep track the system time in millisecond
/*
//test the timing make sure everything finishes within 1ms and leave at least 30% extra time for other code to excute in main routing
void systickHandler(void)
{
    Millis++;  //keep track of the system time in millisecond
    if(bUseIRSensor)
        readIRSensor();
    if(bUseGyro)
        readGyro();
    if(bUseSpeedProfile)
        speedProfile
}
*/

//ps. if you want to change speed, you simply change the targetSpeedX and targetSpeedW in main routing(int main) at any time.


#ifndef _SPEED_CONTROLLER_H
#define _SPEED_CONTROLLER_H

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <ESP_Multi_Board.h>



#define R_WHEEL 15.65 //radius in mm
#define ONE_CELL_DISTANCE 27500/(2*3.14*R_WHEEL)
#define speed_to_counts(a) (120*(a)/(2*3.14*R_WHEEL))
#define counts_to_speed(a) ((a)*(2*3.14*R_WHEEL)/120)

#define kpW1  0.85 //used for T1 and T3 in curve turn
#define kdW1  16 //used for T1 and T3 in curve turn
#define kpW2  0.8//used for T2 in curve turn
#define kdW2  20//used for T2 in curve turn


extern ESP_Multi_Board robot;

/**
 * Control variables
 */
// extern int targetSpeedX;
// extern int targetSpeedW;

/**
 * Output variable. To be readed
 */
// extern long distanceLeft;
// extern long encoderCount;
// extern float curSpeedX;
// extern float curSpeedW;
// extern float accX;
// extern float decX;
// extern float accW;
// extern float decW;

/**
 * Tune variables
 */
// extern float ir_weight;
// extern float kpX, kdX;
// extern float kpW, kdW;//used in straight
// extern float kpW0, kdW0;//used in straight
// extern float kpWir, kdWir;//used with IR errors



//main function
void speedProfile(void *a);

//Principal functions
void getEncoderStatus();
void updateCurrentSpeed();
void calculateMotorPwm();

//Aux functions
void leerIRs(uint8_t *data);
void getErrIR();
void getErrIRChino();
void getErrIRNew();
float needToDecelerate(long dist, int curSpd, int endSpd);//speed are in encoder counts/ms, dist is in encoder counts


void resetSpeedProfile(void);

#endif
