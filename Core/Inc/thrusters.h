/*
 * thrusters.h
 *
 *  Created on: Feb 19, 2024
 *      Author: Viktoria
 */

#ifndef THRUSTERS_H
#define THRUSTERS_H

void thrustersInit();
void desiredSpeed();
void fillThrustersRequest(uint8_t *buf, uint8_t thruster);
void fillThrustersResponse(uint8_t *buf, uint8_t thruster);
void formThrustVectors();

#endif

