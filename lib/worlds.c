/**************************************************************************
  Jose Mastrangelo, Anton ter Vehn & Manish Prajapat
  Group 6
 
 * FILE NAME:  worlds.c
 *
 * EECS461 at the University of Michigan
 * Virtual Worlds library - Template
 *
 * Created Sept 2003 David Thorsley
 *
 * Revision History:
 *  7-27-06 lovelljc
 *
 * Revised and adapted to use at IMRT ETH Zurich
 * 151-0593-00 Embedded Control Systems (ECS)
 *
 * Revision History:
 *  2008-09-04      Marianne Schmid
 *  2011-02-10      Thomas Schneider
 *  2012-08-15      Patrick Müller
 **************************************************************************/

#include <worlds.h>
#include <mpc5553.h>

float virtualSpring(float angle) {
    /* Compute the torque for the virtual spring based
     on the current angle (lab 4) */
    uint16_t k_spring = 50;		//[Nmm/deg]
	float torque;
    torque = k_spring*angle;
    return torque;
}


float virtualWall(float angle) {
    /* Compute the torque for the virtual wall based
     on the current angle (lab 4)  */
    uint16_t k_wall = 1750;		//[Nmm/deg]
	float torque;
    if(angle < WALL_POSITION){
		torque = 0;
	}
	else{
		torque = k_wall*(angle-WALL_POSITION);
	}
    /*fill in here*/
    return torque;
}


float virtualSpringDamper(float angle, float velocity) {
    /* Compute the torque for the virtual spring-damper based
     on the current angle and velocity (lab 6)*/
	float b_spring = 1.23062;//1.23062;
	uint16_t k_spring = 50;		//[Nmm/deg]
	float torque = 0;
    torque = k_spring*angle + b_spring*velocity;

    return torque;
}


float virtualWallDamper(float angle, float velocity) {
    /* Compute the torque for the virtual wall-damper based
    on the current angle and velocity (lab 6)*/
    /* Compute the torque for the virtual wall based
     on the current angle (lab 4)  */
    uint16_t k_wall = 1750;		//[Nmm/deg]
	float b_wall = TIMESTEP*k_wall/2.0;
	float torque;
    if(angle < WALL_POSITION){
		torque = 0;
	}
	else{
		torque = k_wall*(angle-WALL_POSITION) + b_wall*velocity;
	}
    /*fill in here*/
    return torque;
}

float virtualSpringMass(float angle) {
    /* Compute the torque for the virtual spring-mass system based
     on the current angle (lab 6)
     Use TIMESTEP as defined in worlds.h*/
	float k = 17.78;
	float J = 0.45;
	float torque;
	static float x1 = 0;
	static float x2 = 0;
	static float x1_prev = 0;
	static float x2_prev = 0;
	
	x1 = x1_prev + TIMESTEP*x2_prev;
	x2 = x2_prev + TIMESTEP*(k/J)*(-x1_prev + angle);
	
    torque = k*(x1-angle);
	x1_prev = x1;
	x2_prev = x2;

    return torque;
}


float virtualSpringMassDamper(float angle, float velocity) {
    /* Compute the torque for the virtual spring-mass-damper system
     based on the current angle and velocity (lab 6)
     Use TIMESTEP as defined in worlds.h*/
	float k = 17.78;
	float J = 0.45;
	float b_osc  = k*TIMESTEP;
	float torque;
	static float x1 = 0;
	static float x2 = 0;
	static float x1_prev = 0;
	static float x2_prev = 0;
	
	x1 = x1_prev + TIMESTEP*x2_prev;
	x2 = - TIMESTEP*(k/J)*x1_prev +(1 -(b_osc*TIMESTEP)/J)*x2_prev + (k*TIMESTEP/J)*angle + (b_osc*TIMESTEP/J)*velocity;
	
    torque = k*(x1-angle) + b_osc*velocity;;
	x1_prev = x1;
	x2_prev = x2;

    return torque;
}

float virtualSpringMassDamperEx(float angle, float velocity, float K, float J) {
    /* Compute the torque for the virtual spring-mass-damper system
     based on the current angle and velocity (lab 6), adjust spring constant K and mass M
     Use TIMESTEP as defined in worlds.h*/
    float b_osc  = 5*K*TIMESTEP;
	float torque;
	static float x1 = 0;
	static float x2 = 0;
	static float x1_prev = 0;
	static float x2_prev = 0;
	
	x1 = x1_prev + TIMESTEP*x2_prev;
	x2 = -TIMESTEP*(K/J)*x1_prev +(1 -(b_osc*TIMESTEP)/J)*x2_prev + (K*TIMESTEP/J)*angle + (b_osc*TIMESTEP/J)*velocity;
	
    torque = -K*(x1-angle) - b_osc*(x2-velocity);
	x1_prev = x1;
	x2_prev = x2;

    return torque;
}

float virtualKnob(float angle, float velocity, float B){
    /* Compute the torque for the virtual knob system
     based on the current angle and velocity (lab 6)
     Use TIMESTEP as defined in worlds.h
    */
    float torque;
	
	torque = B*(velocity);
    return torque;
}



// EOF
