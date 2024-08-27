/*************************************************************************/
/*  File name: 	hal.h
*
*  Purpose:	Header file for the Hardware Abstraction Layer.
*               Constants used for application are set here.
*
*  Owner:  	Srividya Prasad, Siddharth D Srinivas, Gaurav Sai Palasari
*
*  Department:   ECE, ECE, EEE
*
*  Version History:
*  V4.0  30 July, 2023        Final version created
*/
/******************************************************************************/

#ifndef _HAL_H
#define _HAL_H

#include "datatypes.h"
#include <stdio.h>
#include "io430.h"

#define FREQUENCY 1000000 //1 MHz
#define SAMPLE_PERIOD 3 //3 sec

void hal_setWheelDia(float wd);
void hal_ledInit();
void hal_hallInit();
void resetPulseCount();

UINT16 getPPS();
UINT16 getRPM();
FLOAT32 getSpeed_mps();
void hal_printRPM();
void hal_printPPS();
void hal_printSpeed_mps();

void hal_blinkLED();
void incPulseCount();
void hal_process_HallSensor();
typedef void(* p_fHallSensorCallback_t)();
void hal_setCB_HallSensor(p_fHallSensorCallback_t p_fHallSensorCallback_t_in);
void hal_isr_hallsensorCB();

#endif

