/*
 * thrusters.c
 *
 *  Created on: Feb 19, 2024
 *      Author: Viktoria
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "robot.h"
#include "messages.h"
#include "global.h"
#include "communication.h"
#include "checksum.h"

uint8_t thruster_init = 0;

void addMarchToSumm(float *velocity);
void addLagToSumm(float *velocity);
void addDepthToSumm(float *velocity);
void addYawToSumm(float *velocity);
void addRollToSumm(float *velocity);
void addPitchToSumm(float *velocity);

int8_t resizeFloatToInt8(float input);
// velocity[i] = (KVMA[i][0]*Ux + KVMA[i][1]*Uy + KVMA[i][2]*Uz
//+ KVMA[i][3]*Ugamma + KVMA[i][4]*Uteta + KVMA[i][5]*Upsi)*KVMA[i][6];
float KVMA[6][5] = {
    {1.0, 1.0, 0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0, 0.0, 1.0},
    {1.0, -1.0, 0.0, 1.0, 0.0},
    {1.0, -1.0, 0.0, -1.0, 0.0},
    {0.0, 0.0, 1.0, 0.0, -1.0},
    {1.0, 1.0, 0.0, -1.0, 0.0}
};

void thrustersInit()
{
  rThrusters[FDR].address = 1; //Forward Left
  rThrusters[FDL].address = 2; //Forward Down Left
  rThrusters[BDR].address = 3; //Back Down Right
  rThrusters[BDL].address = 4; //Back Down Left

  rThrusters[FUR].address = 5; //Forward Up Right
  rThrusters[FUL].address = 6; //Forward Up Left
//  rThrusters[BUR].address = 7; //Back Up Right
//  rThrusters[BUL].address = 8; //Back Up Left

  for(uint8_t i=0; i<THRUSTERS_NUMBER; i++) {
    rThrusters[i].desiredSpeed = 0;
    rThrusters[i].kForward = 0.7;
    rThrusters[i].kBackward = 0.7;
    rThrusters[i].sForward = 127;
    rThrusters[i].sBackward = 127;
  }

}

void resetThrusters()
{
  rJoySpeed.depth = 0;
  rJoySpeed.lag = 0;
  rJoySpeed.march = 0;
  rJoySpeed.pitch = 0;
  rJoySpeed.roll = 0;
  rJoySpeed.yaw = 0;

  for(uint8_t i=0; i<THRUSTERS_NUMBER; i++) {
    rThrusters[i].desiredSpeed = 0;
  }

}

void fillThrustersRequest(uint8_t *buf, uint8_t thruster)
{
    struct thrustersRequest_s res;

    res.AA = 0xAA;
    res.type = 0x01;
    res.address = 0xAF;
    for(int i=0; i<THRUSTERS_NUMBER;i++){
    	int16_t velocity = rThrusters[i].desiredSpeed;


    	// Inverting
    	if(rThrusters[i].inverse) {
//    		velocity *= -1;
    	}

    	// Multiplier constants
    	if(velocity > 0) {
    		velocity = (int16_t) ( (float) (velocity) * rThrusters[i].kForward);
    	}
    	else if(velocity < 0) {
    		velocity = (int16_t) ((float) (velocity) * rThrusters[i].kBackward);
    	}

    	res.velocity[i] = velocity;
    }
    res.pwm_servo = rDevice[GRAB].force;
    memcpy((void*)buf, (void*)&res, THRUSTERS_REQUEST_LENGTH);
    AddChecksumm8bVma(buf, THRUSTERS_REQUEST_LENGTH);
}

void fillThrustersResponse(uint8_t *buf, uint8_t thruster)
{
	//TODO errors parsing! and what is all this new stuff means
    if(IsChecksumm8bCorrectVma(buf, THRUSTERS_RESPONSE_LENGTH) && buf[0] != 0) {
    	struct thrustersResponse_s res;
    	memcpy((void*)&res, (void*)buf, THRUSTERS_RESPONSE_LENGTH);

        rThrusters[thruster].current = res.current;

        ++uartBus[THRUSTERS_UART].successRxCounter;
    }
    else {
    	++uartBus[THRUSTERS_UART].brokenRxCounter;
    }
}

void formThrustVectors()
{
  float velocity[THRUSTERS_NUMBER];
  for(uint8_t i=0; i<THRUSTERS_NUMBER; i++) {
    velocity[i] = 0;
  }

//  float Ux;
//  float Uy;
//  float Uz;
//  float Ugamma;
//  float Uteta;
//  float Upsi;

  float U[6];

//  Ux = rJoySpeed.march;
//  Uy = rJoySpeed.lag;
//  Uz = rStabState[STAB_DEPTH].outputSignal;
//  Upsi = rStabState[STAB_YAW].outputSignal;
//  Ugamma = rStabState[STAB_ROLL].outputSignal;
//  Uteta = rJoySpeed.pitch;

  U[STAB_MARCH] = rJoySpeed.march;
  U[STAB_LAG] = rJoySpeed.lag;
//  U[STAB_DEPTH] = rJoySpeed.depth;
  U[STAB_DEPTH] = input_value;
  U[STAB_YAW] = rJoySpeed.yaw;
  U[STAB_ROLL] = rJoySpeed.roll;
  U[STAB_PITCH] = rJoySpeed.pitch;

  for(uint8_t i = 0; i < 6; i++)
  {
	if(rStabConstants[i].enable)
		U[i] = rStabState[i].outputSignal;
  }

  for (uint8_t i = 0; i < THRUSTERS_NUMBER; ++i)
  {
    velocity[i] = KVMA[i][0]*U[STAB_MARCH] + KVMA[i][1]*U[STAB_LAG] + KVMA[i][2]*U[STAB_DEPTH]
         + KVMA[i][3]*U[STAB_YAW] + KVMA[i][4]*U[STAB_ROLL];
    if(velocity[i]<=12 && velocity[i]>=-12) //Thrusters dead zone asymmetric, +3 ... -11~-12
    	velocity[i] = 0;
    if(!thruster_init)
    {
    	rThrusters[i].desiredSpeed = resizeFloatToInt8(velocity[i]);
    }
    else
    	rThrusters[i].desiredSpeed = 0;
  	}

}

int8_t resizeFloatToInt8(float input)
{
  int32_t cast = (int32_t) input;
//  cast = cast / 0xFF;
  if (cast > 127) {
    cast = 127;
  }
  else if(cast < -127) {
    cast = -127;
  }
  return (int8_t) cast;
}

