//Manish Prajapat, Jose Mario Mastrangelo , Anton ter Vehn
// Group 06
/**************************************************************************
 * FILE NAME:  lab5.c
 *
 * EECS461 at the University of Michigan
 * Lab 5 - Template
 *
 * Created July 2005 Eric Williams
 *
 * Revision History:
 *   8-9-06  lovelljc
 *  10-4-06  ericjw
 *
 * Revised and adapted to use at IMRT ETH Zurich
 * 151-0593-00 Embedded Control Systems (ECS)
 *
 * Revision History:
 *	2008-09-04		Marianne Schmid
 *	2009-09-13		Marianne Schmid (different name for MiosChannel)
 *	2011-02-14		Thomas Schneider
 **************************************************************************/

#include <mpc5553.h>
#include <math.h>
#include <ecs.h>
#include <mios.h>
#include <qadc.h>
#include <isr.h>

#define SCF_MIOS_CHANNEL	12		/* The haptic interface uses eMIOS[12]  for the sc-Filter*/
#define SINUS_MIOS_CHANNEL   1       /* The haptic interface uses eMIOS[0]. Use eMIOS[1] for this lab */
#define PWM_PERIOD_ns       50000   /* 50000 Initialize PWM at 20 kHz */
#define MIOS_CLOCK_SCALER   0b0000  /* Divide by (SCALER + 1) = 1 */

#define SINGLE_SCAN_CHANNEL     0   /* Use AN0 for external input, AN1 for ECS interface board potentiometer */
#define NUM_CONTINUOUS_CHANNELS 0   /* Can scan up to 8 input channels continuously */

#define INTERRUPT_FREQUENCY_A 20000   /* Fixed-Interval Interrupts frequency (Hz) for IsrA*/
#define INTERRUPT_FREQUENCY_B  1000 //22220   /* Fixed-Interval Interrupts frequency (Hz) for IsrB*/
#define INTERRUPT_FREQUENCY_C  2222222   /* Fixed-Interval Interrupts frequency (Hz) for IsrC*/

#define M_PI 3.1415926535897
const int ptsPeriod = 10;
volatile static float SINE_LOOKUP[ptsPeriod];


//#define LAB5_A
//#define LAB5_B
#define LAB5_C

/* Use the appropriate interrupt frequency when calling an ISR */

float getDutyCycle(float in, float in_max, float in_min, float out_max, float out_min){
	float out = in*(out_max-out_min)/(in_max-in_min) + (out_min*in_max - out_max*in_min)/(in_max-in_min);
	return out;
}

void IsrA(void)
{
	uint16_t iAnalog;
	float in, dutyCycle;
	int8_t status;
	
	SIU.GPDO[29].B.PDO = 1;
	iAnalog = qadcReadQ1();
	iAnalog = iAnalog >> 2;
	in = (iAnalog/4095.0f)*5.12f;
	// calculate duty cycle
	dutyCycle = getDutyCycle(in, 4.0,1.0,0.9,0.1);
	
	//set PWM based on DIP switches
	if(SIU.GPDI[123].B.PDI == 1){
		status = set_PWMDutyCycleLimits(SINUS_MIOS_CHANNEL, 0.4,0.6);
	}
	else{
		status = set_PWMDutyCycleLimits(SINUS_MIOS_CHANNEL, 0.1,0.9);
	}
	
	if(SIU.GPDI[122].B.PDI == 1){
		set_PWMPeriod_ns(SINUS_MIOS_CHANNEL, 16667);
	}
	else{
	
		set_PWMPeriod_ns(SINUS_MIOS_CHANNEL, PWM_PERIOD_ns);
	}
	set_PWMDutyCycle(SINUS_MIOS_CHANNEL, dutyCycle);
	SIU.GPDO[29].B.PDO = 0;
}

void IsrB(void)
{	
	
	static uint32_t incB = 0;
	//double sine_data;
	float sine_data;
	float dutyCycle;
	SIU.GPDO[29].B.PDO = 1;
	//sine_data = sin(2*M_PI*incB/10.0);
	sine_data = sinf(2*M_PI*incB/10.0);
	dutyCycle = getDutyCycle(sine_data, 1.0,-1.0,0.9,0.1);
	set_PWMPeriod_ns(SINUS_MIOS_CHANNEL, 16667);
	set_PWMDutyCycle(SINUS_MIOS_CHANNEL, dutyCycle);
	incB++;
	SIU.GPDO[29].B.PDO = 0;
}

void IsrC(void)
{
	static uint8_t incC = 0;
	//double sine_data;
	//float dutyCycle;
	
	SIU.GPDO[29].B.PDO = 1;
	//check overflow
	//dutyCycle = SINE_LOOKUP[incC];
	EMIOS.CH[SINUS_MIOS_CHANNEL].CADR.R = EMIOS.CH[SINUS_MIOS_CHANNEL].CBDR.R * SINE_LOOKUP[incC];
	incC++;
	//incC = incC % ptsPeriod;
	if (incC ==  10) incC = 0;
	SIU.GPDO[29].B.PDO = 0;
}

#ifdef LAB5_A
int main() {
	int i;
	
	/* initialize processor */
	init_ECS(5);
	init_MIOS_clock(MIOS_CLOCK_SCALER);
	init_PWM(SINUS_MIOS_CHANNEL,PWM_PERIOD_ns);
	init_PWM(SCF_MIOS_CHANNEL,1000); /* 1 MHz */
	qadcInit_single(SINGLE_SCAN_CHANNEL);

	/* initialize GPIO */

	for(i=122; i<=123; i++) 
	{ 
		SIU.PCR[i].B.PA = 0;  /* GPIO */ 
		SIU.PCR[i].B.IBE = 1; /* Input */ 
		SIU.PCR[i].B.WPE = 0; /* Weak pull up disabled */ 
	} 
	SIU.PCR[29].B.PA = 0;  /* GPIO */ 
	SIU.PCR[29].B.OBE = 1; /* Output */ 
	SIU.PCR[29].B.WPE = 0; /* Weak pull up disabled */ 
	
	/* init interrupts */
	init_interrupts(IsrA, INTERRUPT_FREQUENCY_A);
	/* enable interrupts */
	enable_interrupts();

	while(1)
	{
		/* Loop forever */
	}
	return(0);
}
#endif

#ifdef LAB5_B
int main() {

	int i;

	/* initialize processor */
	init_ECS(5);
	init_MIOS_clock(MIOS_CLOCK_SCALER);
	init_PWM(SINUS_MIOS_CHANNEL,PWM_PERIOD_ns);
	init_PWM(SCF_MIOS_CHANNEL,1000); /* 1 MHz */
	qadcInit_single(SINGLE_SCAN_CHANNEL);

	/* initialize GPIO */
	for(i=122; i<=123; i++) 
	{ 
		SIU.PCR[i].B.PA = 0;  /* GPIO */ 
		SIU.PCR[i].B.IBE = 1; /* Input */ 
		SIU.PCR[i].B.WPE = 0; /* Weak pull up disabled */ 
	} 
	SIU.PCR[29].B.PA = 0;  /* GPIO */ 
	SIU.PCR[29].B.OBE = 1; /* Output */ 
	SIU.PCR[29].B.WPE = 0; /* Weak pull up disabled */ 
	
	/* init interrupts */
	init_interrupts(IsrB, INTERRUPT_FREQUENCY_B);
	/* enable interrupts */
	enable_interrupts();

	while(1)
	{
		/* Loop forever */
	}
	return(0);
}
#endif

#ifdef LAB5_C
int main() {
	
	int i;
	float sineFreq = INTERRUPT_FREQUENCY_C/ (float)ptsPeriod;
	float dutyCycle;

	/* initialize processor */
	init_ECS(5);
	init_MIOS_clock(MIOS_CLOCK_SCALER);
	init_PWM(SINUS_MIOS_CHANNEL,16667);
	init_PWM(SCF_MIOS_CHANNEL,1000); /* 1 MHz */
	qadcInit_single(SINGLE_SCAN_CHANNEL);
	
	// Pre-compute sine
	for(i=0; i < ptsPeriod ;i++){
		dutyCycle = getDutyCycle(sin(2*M_PI*i/10.0), 1.0,-1.0,0.9,0.1);
		SINE_LOOKUP[i] = dutyCycle;
	}
 
	for(i=122; i<=123; i++) 
	{ 
		SIU.PCR[i].B.PA = 0;  /* GPIO */ 
		SIU.PCR[i].B.IBE = 1; /* Input */ 
		SIU.PCR[i].B.WPE = 0; /* Weak pull up disabled */ 
	} 
	SIU.PCR[29].B.PA = 0;  /* GPIO */ 
	SIU.PCR[29].B.OBE = 1; /* Output */ 
	SIU.PCR[29].B.WPE = 0; /* Weak pull up disabled */ 
	
	/* init interrupts */
	init_interrupts(IsrC, INTERRUPT_FREQUENCY_C);
	/* enable interrupts */
	enable_interrupts();


	
	while(1)
	{
		/* Loop forever */
	}
	return(0);
}
#endif
/* EOF */
