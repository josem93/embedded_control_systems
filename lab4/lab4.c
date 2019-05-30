/**************************************************************************
   Jose Mastrangelo, Anton ter Vehn & Manish Prajapat
   Group 6
 * FILE NAME:  lab4.c
 *
 * EECS461 at the University of Michigan
 * Lab 4 Template
 *
 * Created July 2005
 *
 * Revision History:
 *   8-3-06  lovelljc
 *
 * Revised and adapted to use at IMRT ETH Zurich
 * 151-0593-00 Embedded Control Systems (ECS)
 *
 * Revision History:
 *  2008-09-04      Marianne Schmid
 **************************************************************************/
#include <ecs.h>
#include <mios.h>
#include <fqd.h>
#include <worlds.h>
#include <motor.h>
#include <qadc.h>
#include <mpc5553.h>

#define SCF_MIOS_CHANNEL        12                      /* The haptic interface uses eMIOS[12]  for the sc-Filter*/
#define HAPTIC_MIOS_CHANNEL     0                       /* The ECS interface board sends eMIOS[0] to the haptic interface motor */
#define PWM_PERIOD_ns           25000//(500000)                 /* 25000 Initialize PWM at 40 kHz */
#define MIOS_CLOCK_SCALER       0b00000000 				/* Select a clock prescalar that allows the eMIOS to run at the maximum possible frequency */
#define SINGLE_SCAN_CHANNEL     3                       /* Use AN3 for input of the potentiometer */
#define NUM_CONTINUOUS_CHANNELS 0                       /* Can scan up to 8 input channels continuously */+

#define LAB 4
//#define LAB4_3
//#define LAB4_4
#define LAB4_9

#ifdef LAB4_3
int main()
{
    /* Variable declarations here */
    /* Initializations */
    init_ECS(LAB);                                      /* Set the system clock to 40 MHz, enable the motor */
    init_MIOS_clock(MIOS_CLOCK_SCALER);                 /* Set the clock prescaler */
    init_PWM(HAPTIC_MIOS_CHANNEL, PWM_PERIOD_ns); 		//This was filled in
    init_PWM(SCF_MIOS_CHANNEL,1000);                    /* The sc-filter is run with 1 MHz*/
    set_PWMDutyCycleLimits(HAPTIC_MIOS_CHANNEL, 0.24, 0.76);
    init_FQD();
	
	set_PWMDutyCycle(HAPTIC_MIOS_CHANNEL, 0.5);
    /* Fill in lab4.c code here */
	while(1){}
	
    return(0);

}
#endif

#ifdef LAB4_4
int main()
{
    /* Variable declarations here */
    /* Initializations */
	vuint16_t iAnalogQ1;
	float dutyCycle;
	qadcInit_single(SINGLE_SCAN_CHANNEL);
	
    init_ECS(LAB);                                      /* Set the system clock to 40 MHz, enable the motor */
    init_MIOS_clock(MIOS_CLOCK_SCALER);                 /* Set the clock prescaler */
    init_PWM(HAPTIC_MIOS_CHANNEL, PWM_PERIOD_ns); 		//This was filled in
    init_PWM(SCF_MIOS_CHANNEL,1000);                    /* The sc-filter is run with 1 MHz*/
    set_PWMDutyCycleLimits(HAPTIC_MIOS_CHANNEL, 0.24, 0.76);
    init_FQD();
	
	set_PWMDutyCycle(HAPTIC_MIOS_CHANNEL, 0.5);
    /* Fill in lab4.c code here */
	while(1){
		iAnalogQ1 = qadcReadQ1();
		dutyCycle = iAnalogQ1 / 16000.0;
		set_PWMDutyCycle(HAPTIC_MIOS_CHANNEL, dutyCycle);
	}
	
    return(0);

}
#endif

#ifdef LAB4_7
int main()
{
    /* Variable declarations here */
    /* Initializations */
	float dutyCycle;
	float torque = 200;
	qadcInit_single(SINGLE_SCAN_CHANNEL);
	
    init_ECS(LAB);                                      /* Set the system clock to 40 MHz, enable the motor */
    init_MIOS_clock(MIOS_CLOCK_SCALER);                 /* Set the clock prescaler */
    init_PWM(HAPTIC_MIOS_CHANNEL, PWM_PERIOD_ns); 		//This was filled in
    init_PWM(SCF_MIOS_CHANNEL,1000);                    /* The sc-filter is run with 1 MHz*/
    set_PWMDutyCycleLimits(HAPTIC_MIOS_CHANNEL, 0.24, 0.76);
    init_FQD();
	
	set_PWMDutyCycle(HAPTIC_MIOS_CHANNEL, 0.5);
    /* Fill in lab4.c code here */
	dutyCycle = outputTorque(torque);
	while(1){
		set_PWMDutyCycle(HAPTIC_MIOS_CHANNEL, dutyCycle);
	}
	
    return(0);

}
#endif

#ifdef LAB4_8
int main()
{
    /* Variable declarations here */
    /* Initializations */
	vuint8_t *siu_gpdo_ptr;
	float dutyCycle;
	float torque;
	float angle;
	char buff[32];
	float anglePrev=0;
	uint16_t counter;
	uint32_t result; 
	
    init_ECS(LAB);                                      /* Set the system clock to 40 MHz, enable the motor */
    init_MIOS_clock(MIOS_CLOCK_SCALER);                 /* Set the clock prescaler */
    init_PWM(HAPTIC_MIOS_CHANNEL, PWM_PERIOD_ns); 		//This was filled in
    init_PWM(SCF_MIOS_CHANNEL,1000);                    /* The sc-filter is run with 1 MHz*/
    set_PWMDutyCycleLimits(HAPTIC_MIOS_CHANNEL, 0.24, 0.76);
    init_FQD();
	
	set_PWMDutyCycle(HAPTIC_MIOS_CHANNEL, 0.5);
    /* Fill in lab4.c code here */
	init_COM(1,115200);
	
	siu_gpdo_ptr = (unsigned char*)(0Xc3f90600);
	SIU.PCR[28].B.PA = 0;  /* GPIO */ 
	SIU.PCR[28].B.OBE = 1; /* Output */ 
	SIU.PCR[28].B.WPE = 0; 
	
	while(1){
		siu_gpdo_ptr[28] = 1;
		//counter = ReadFQD_pc();
		//result = updateCounter();
		angle = updateAngle();
		
		torque = virtualSpring(angle);
		dutyCycle = outputTorque(torque);
		set_PWMDutyCycle(HAPTIC_MIOS_CHANNEL, dutyCycle);
		
		//if(angle != anglePrev){
		//	sprintf(buff, "\r\n Angle = %3.2f", angle);
		//	serial_puts(1,buff);
		//}
		//anglePrev = angle;
		//set_PWMDutyCycle(HAPTIC_MIOS_CHANNEL, 0.5);
		siu_gpdo_ptr[28] = 0;
	}
	
    return(0);

}
#endif

#ifdef LAB4_9
int main()
{
    /* Variable declarations here */
    /* Initializations */
	vuint8_t *siu_gpdo_ptr;
	float dutyCycle;
	float torque;
	float angle;
	char buff[32];
	float anglePrev=0;
	uint16_t counter;
	uint32_t result; 
	
    init_ECS(LAB);                                      /* Set the system clock to 40 MHz, enable the motor */
    init_MIOS_clock(MIOS_CLOCK_SCALER);                 /* Set the clock prescaler */
    init_PWM(HAPTIC_MIOS_CHANNEL, PWM_PERIOD_ns); 		//This was filled in
    init_PWM(SCF_MIOS_CHANNEL,1000);                    /* The sc-filter is run with 1 MHz*/
    set_PWMDutyCycleLimits(HAPTIC_MIOS_CHANNEL, 0.24, 0.76);
    init_FQD();
	
	set_PWMDutyCycle(HAPTIC_MIOS_CHANNEL, 0.5);
    /* Fill in lab4.c code here */
	init_COM(1,115200);
	
	siu_gpdo_ptr = (unsigned char*)(0Xc3f90600);
	SIU.PCR[28].B.PA = 0;  /* GPIO */ 
	SIU.PCR[28].B.OBE = 1; /* Output */ 
	SIU.PCR[28].B.WPE = 0; 
	
	while(1){
		siu_gpdo_ptr[28] = 1;
		//counter = ReadFQD_pc();
		//result = updateCounter();
		angle = updateAngle();
		
		torque = virtualWall(angle);
		dutyCycle = outputTorque(torque);
		set_PWMDutyCycle(HAPTIC_MIOS_CHANNEL, dutyCycle);
		
		//if(angle != anglePrev){
		//	sprintf(buff, "\r\n Angle = %3.2f", angle);
		//	serial_puts(1,buff);
		//}
		//anglePrev = angle;
		//set_PWMDutyCycle(HAPTIC_MIOS_CHANNEL, 0.5);
		siu_gpdo_ptr[28] = 0;
	}
	
    return(0);

}
#endif
/* EOF */
