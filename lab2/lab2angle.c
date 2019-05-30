//Jose Mastrangelo, Anton ter Vehn, Manish Prajapat
//Group 06


#include <fqd.h>
#include <ecs.h>
#include <mpc5553.h>
#include <serial.h>

void main()
{
	uint16_t counter;
	int32_t result, result_prev, temp;
	int i;
	char buff[32];
	float angle;
	/* pointers to registers */
	volatile unsigned char *siu_gpdo_ptr;
	init_ECS(2);
	/* Initialize FQD*/
	init_FQD();

	counter = ReadFQD_pc();

	siu_gpdo_ptr = (unsigned char*)(0Xc3f90600);  /* SIU_BASE + 0X600 for gpdo */ 
	
	/* configure output leds */ 
	for(i=28; i<44; i++) 
	{ 
		SIU.PCR[i].B.PA = 0;  /* GPIO */ 
		SIU.PCR[i].B.OBE = 1; /* Output */ 
		SIU.PCR[i].B.WPE = 0; 
	}
	
	init_COM(1,115200);
	result_prev = 0;
	while (1)
	{
		counter = ReadFQD_pc();
		result = updateCounter();
		temp = result;
		//result = result >> 6;
		for(i=28; i<44; i++) { 
			siu_gpdo_ptr[i] = (result & 0x01);
			result = result >> 1;
		} 
		angle = updateAngle();
		if(temp != result_prev){
			sprintf(buff, "\r\n Count = %i, Angle = %3.2f", temp, angle);
			serial_puts(1,buff);
		}
		result_prev = temp;
	}
}