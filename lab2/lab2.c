//Jose Mastrangelo, Anton ter Vehn, Manish Prajapat
//Group 06

#include <fqd.h>
#include <ecs.h>
#include <mpc5553.h>

void main()
{
	uint16_t counter;
	int32_t result;
	int i;
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

	while (1)
	{
		counter = ReadFQD_pc();
		result = updateCounter();
		result = result >> 8;
		for(i=28; i<44; i++) { 
			siu_gpdo_ptr[i] = (result & 0x00000001);
			result = result >> 1;
		} 
	}

}