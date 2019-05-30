//Jose Mastrangelo, Anton ter Vehn, Manish Prajapat
//Group 06

#include <mpc5553.h>
#include <ecs.h>
#include <mios.h>
#include <fqd.h>
#include <motor.h>
#include <serial.h>
#include <isr.h>
#include <worlds.h>
#include <serial.h>

#define SCF_MIOS_CHANNEL	12		/* The haptic interface uses eMIOS[12]  for the SC-Filter*/
#define HAPTIC_MIOS_CHANNEL	0		/* The haptic interface uses eMIOS[0]  for the PWM Signal*/
#define PWM_PERIOD_ns       (25000) /* 25000 Initialize PWM at 40 kHz */
#define MIOS_CLOCK_SCALER	0b000	/* Divide by (SCALER + 1) */

#define INTERRUPT_FREQUENCY	1000	/* Fixed-Interval Interrupts frequency (Hz) */
/* TIMESTEP defined in worlds.h - should be 1 / INTERRUPT_FREQUENCY */
//#define LAB6_2
//#define LAB6_3
//#define LAB6_5
//#define LAB6_6
//#define LAB6_7
#define LAB6_8

void sdIsr(void) {
	/* ISR for the spring-damper system */
	/* Calculate the wheel's velocity */
	/* Calculate and apply the correct torque to haptic wheel */
	float angle;
	static float prev_angle = 0;
	float velocity, torque, dutyCycle;
	angle = updateAngle();
	velocity = (angle - prev_angle)/(float)(1.0f/ INTERRUPT_FREQUENCY);
	
	torque = virtualSpringDamper(angle, velocity);
	//torque = virtualSpring(angle);
	dutyCycle = outputTorque(torque);
	set_PWMDutyCycle(HAPTIC_MIOS_CHANNEL, dutyCycle);
	
	prev_angle = angle;
	
}

void wdIsr(void) {
	/* ISR for the wall-damper system */
	/* Calculate the wheel's velocity */
	/* Calculate and apply the correct torque to haptic wheel */
	float angle;
	static float prev_angle = 0;
	float velocity, torque, dutyCycle;
	angle = updateAngle();
	velocity = (angle - prev_angle)/TIMESTEP;
		
	torque = virtualWallDamper(angle, velocity);
	dutyCycle = outputTorque(torque);
	set_PWMDutyCycle(HAPTIC_MIOS_CHANNEL, dutyCycle);
	
	prev_angle = angle;
  }

void smIsr(void) {
	/* ISR for the spring-mass system */
    /* Fill in here */
	
	float angle;
	float torque, dutyCycle;
	angle = updateAngle();
	
	torque = virtualSpringMass(angle);
	dutyCycle = outputTorque(torque);
	set_PWMDutyCycle(HAPTIC_MIOS_CHANNEL, dutyCycle);	
	
	
}

void smdIsr(void) {
	/* ISR for the spring-mass-damper system */
    float angle;
	static float prev_angle = 0;
	float velocity, torque, dutyCycle;
	angle = updateAngle();
	velocity = (angle - prev_angle)/(float)(1.0f/ INTERRUPT_FREQUENCY);
	
	torque = virtualSpringMassDamper(angle, velocity);
	dutyCycle = outputTorque(torque);
	set_PWMDutyCycle(HAPTIC_MIOS_CHANNEL, dutyCycle);
	prev_angle = angle;
}

#define ENTER_CRITICAL()  asm (" wrteei 0");
#define LEAVE_CRITICAL()  asm (" wrteei 1");

/*shared variables between main and interrupt service routine*/
	static float K;
	static float J;
	static float B;

void smdIsrEx(void) {


    float angle;
	static float prev_angle = 0;
	float velocity, torque, dutyCycle;
	angle = updateAngle();
	velocity = (angle - prev_angle)/(float)(1.0f/ INTERRUPT_FREQUENCY);
	
	torque = virtualSpringMassDamperEx(angle, velocity, K, J);
	dutyCycle = outputTorque(torque);
	set_PWMDutyCycle(HAPTIC_MIOS_CHANNEL, dutyCycle);
	prev_angle = angle;
}

void knobIsr(void) {
	/* ISR for the virtual knob system */
    float angle;
	static float prev_angle = 0;
	float velocity, torque, dutyCycle;
	int c, d;
	angle = updateAngle();
	velocity = (angle - prev_angle)/(float)(1.0f/ INTERRUPT_FREQUENCY);
	
	torque = virtualKnob(angle, velocity, B);
	dutyCycle = outputTorque(torque);
	set_PWMDutyCycle(HAPTIC_MIOS_CHANNEL, dutyCycle);
	if((int)angle%10 == 0){
		if(velocity>0){
			set_PWMDutyCycle(HAPTIC_MIOS_CHANNEL, 0.1); 
            for (c = 1; c <= 32; c++)
			for (d = 1; d <= 32; d++)
				{}			}
	
		if(velocity<0){
			set_PWMDutyCycle(HAPTIC_MIOS_CHANNEL, 0.9); 
			for (c = 1; c <= 32; c++)
			for (d = 1; d <= 32; d++)
				{}			}
	set_PWMDutyCycle(HAPTIC_MIOS_CHANNEL, 0.5);
	}
	prev_angle = angle;
	
}
#ifdef LAB6_2
int main()
{
	float *inc_var = &K;
	init_ECS(6);
	init_MIOS_clock(MIOS_CLOCK_SCALER);
	init_PWM(HAPTIC_MIOS_CHANNEL,PWM_PERIOD_ns);
	set_PWMDutyCycleLimits(HAPTIC_MIOS_CHANNEL, 0.24, 0.76);
	init_PWM(SCF_MIOS_CHANNEL,1000);  /* 1 MHz */

	init_FQD();
	init_COM(1,115200);

	/* used for smdIsrEx(void)
	  *serial_puts(1,"\r\nK selected, Enter k,j,+,-\r\n");
	/*

	/* Initialize GPIO */
	/* Fill in here */

	/* init interrupts */
	init_interrupts(wdIsr, INTERRUPT_FREQUENCY);
	/* Enable interrupts */
    /* Fill in here */
	enable_interrupts();

	while(1){
	}
	return(0);
}
#endif

#ifdef LAB6_3
int main()
{
	float *inc_var = &K;
	init_ECS(6);
	init_MIOS_clock(MIOS_CLOCK_SCALER);
	init_PWM(HAPTIC_MIOS_CHANNEL,PWM_PERIOD_ns);
	set_PWMDutyCycleLimits(HAPTIC_MIOS_CHANNEL, 0.24, 0.76);
	init_PWM(SCF_MIOS_CHANNEL,1000);  /* 1 MHz */

	init_FQD();
	init_COM(1,115200);

	/* used for smdIsrEx(void)
	  *serial_puts(1,"\r\nK selected, Enter k,j,+,-\r\n");
	/*

	/* Initialize GPIO */
	/* Fill in here */

	/* init interrupts */
	init_interrupts(sdIsr, INTERRUPT_FREQUENCY);
	/* Enable interrupts */
    /* Fill in here */
	enable_interrupts();

	while(1){
	}
	return(0);
}
#endif

#ifdef LAB6_5
int main()
{
	float *inc_var = &K;
	init_ECS(6);
	init_MIOS_clock(MIOS_CLOCK_SCALER);
	init_PWM(HAPTIC_MIOS_CHANNEL,PWM_PERIOD_ns);
	set_PWMDutyCycleLimits(HAPTIC_MIOS_CHANNEL, 0.24, 0.76);
	init_PWM(SCF_MIOS_CHANNEL,1000);  /* 1 MHz */

	init_FQD();
	init_COM(1,115200);

	/* init interrupts */
	init_interrupts(smIsr, INTERRUPT_FREQUENCY);
	/* Enable interrupts */
    /* Fill in here */
	enable_interrupts();

	while(1){
	}
	return(0);
}
#endif

#ifdef LAB6_6
int main()
{
	float *inc_var = &K;
	init_ECS(6);
	init_MIOS_clock(MIOS_CLOCK_SCALER);
	init_PWM(HAPTIC_MIOS_CHANNEL,PWM_PERIOD_ns);
	set_PWMDutyCycleLimits(HAPTIC_MIOS_CHANNEL, 0.24, 0.76);
	init_PWM(SCF_MIOS_CHANNEL,1000);  /* 1 MHz */

	init_FQD();
	init_COM(1,115200);

	/* used for smdIsrEx(void)
	  *serial_puts(1,"\r\nK selected, Enter k,j,+,-\r\n");
	/*

	/* Initialize GPIO */
	/* Fill in here */

	/* init interrupts */
	init_interrupts(smdIsr, INTERRUPT_FREQUENCY);
	/* Enable interrupts */
    /* Fill in here */
	enable_interrupts();

	while(1){
	}
	return(0);
}
#endif

#ifdef LAB6_7
int main()
{
	float *inc_var = &K;
	char input;
	char byte_in;
	float j_prev,k_prev;
	char buff[32];
	init_ECS(6);
	init_MIOS_clock(MIOS_CLOCK_SCALER);
	init_PWM(HAPTIC_MIOS_CHANNEL,PWM_PERIOD_ns);
	set_PWMDutyCycleLimits(HAPTIC_MIOS_CHANNEL, 0.24, 0.76);
	init_PWM(SCF_MIOS_CHANNEL,1000);  /* 1 MHz */

	init_FQD();
	init_COM(1,115200);
     
	/* Initialize GPIO */
	/* Fill in here */

	/* init interrupts */
	init_interrupts(smdIsrEx, INTERRUPT_FREQUENCY);
	/* Enable interrupts */
    /* Fill in here */
	K = 17.78;
	J = 0.45;
	enable_interrupts();

	while(1){
		if(serial_readyToReceive(1)){
			byte_in = serial_getchar(1);
			
			if(byte_in == 'k' || byte_in == 'K'){
				input = byte_in;
			}
			if(byte_in == 'j' || byte_in == 'J'){
				input = byte_in;
			}
			
			if(input == 'k' || input == 'K'){
				serial_puts(1,"\r\nK selected, Enter +,-\r\n");
				if(byte_in == 44){
					ENTER_CRITICAL();
					K=K*1.1;

					LEAVE_CRITICAL();
					sprintf(buff, "\r\n K = %3.2f, J = %3.2f ", K,J);
					serial_puts(1,buff);
					
				}
				if(byte_in == 27){
					ENTER_CRITICAL();
					K=K*(1/1.1);
					sprintf(buff, "\r\n K = %3.2f, J = %3.2f ", K,J);	
					serial_puts(1,buff);
					LEAVE_CRITICAL();
					
				}
			}
			if(input == 'j' || input == 'J'){
			serial_puts(1,"\r\nJ selected, Enter +,-\r\n");
				if(byte_in == 44){
					ENTER_CRITICAL();
					J=J*1.1;		
					sprintf(buff, "\r\n K = %3.2f, J = %3.2f ", K,J);
					serial_puts(1,buff);
					LEAVE_CRITICAL();
				}
				if(byte_in == 27){
					ENTER_CRITICAL();
					J=J*(1/1.1);	
					sprintf(buff, "\r\n K = %3.2f, J = %3.2f ", K,J);			
					serial_puts(1,buff);
					LEAVE_CRITICAL();
				}
			}
		}
	}
	return(0);
}
#endif

#ifdef LAB6_8
int main()
{
	float *inc_var = &K;
	char byte_in;
	char buff[32];
	init_ECS(6);
	init_MIOS_clock(MIOS_CLOCK_SCALER);
	init_PWM(HAPTIC_MIOS_CHANNEL,PWM_PERIOD_ns);
	set_PWMDutyCycleLimits(HAPTIC_MIOS_CHANNEL, 0.24, 0.76);
	init_PWM(SCF_MIOS_CHANNEL,1000);  /* 1 MHz */

	init_FQD();
	init_COM(1,115200);

	/* Initialize GPIO */
	/* Fill in here */

	/* init interrupts */
	init_interrupts(knobIsr, INTERRUPT_FREQUENCY);
	/* Enable interrupts */
    /* Fill in here */
	enable_interrupts();
	B=1.23;
	while(1){
		if(serial_readyToReceive(1)){
			byte_in = serial_getchar(1);
			serial_puts(1,"\r\nB selected, Enter +,-\r\n");
			if(byte_in == 44){
				ENTER_CRITICAL();
				B=B*1.1;
				LEAVE_CRITICAL();
				sprintf(buff, "\r\n B = %3.2f", B);
				serial_puts(1,buff);
			}
			if(byte_in == 27){
				ENTER_CRITICAL();
				B=B*(1/1.1);
				LEAVE_CRITICAL();
				sprintf(buff, "\r\n B = %3.2f ",B);
				serial_puts(1,buff);
			}
		}
	}
	return(0);
}
#endif
//EOF
