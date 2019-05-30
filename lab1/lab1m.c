//Jose Mastrangelo, Anton ter Vehn, Manish Prajapat
//Group 06

/* Typedefs and processor initialization */ 
#include <ecs.h> 
#include <mpc5553.h>
#include <math.h>

void main(){ 
	int i; 
    vuint8_t input_one, input_two, result;

	/* pointers to registers */ 
	volatile unsigned char *siu_gpdi_ptr; 
	volatile unsigned char *siu_gpdo_ptr; 
	siu_gpdo_ptr = (unsigned char*)(0Xc3f90600);  /* SIU_BASE + 0X600 for gpdo */ 
	siu_gpdi_ptr = (unsigned char*)(0Xc3f90800);  /* SIU_BASE + 0X800 for gpdi */ 
	
	/* configure input dipswitches */ 
	for(i=122; i<130; i++) 
	{ 
		SIU.PCR[i].B.PA = 0;  /* GPIO */ 
		SIU.PCR[i].B.IBE = 1; /* Input */ 
		SIU.PCR[i].B.WPE = 0; /* Weak pull up disabled */ 
	} 
	/* configure output leds */ 
	for(i=28; i<33; i++) 
	{ 
		SIU.PCR[i].B.PA = 0;  /* GPIO */ 
		SIU.PCR[i].B.OBE = 1; /* Output */ 
		SIU.PCR[i].B.WPE = 0; 
	} 
	
	while (1){
		input_one = 0;
		input_two = 0;
		result = 0;
		
		for (i=0;i<4;i++)
		{   input_one += *(siu_gpdi_ptr+122+i)*pow(2,i);
			input_two += *(siu_gpdi_ptr+126+i)*pow(2,i);
			}
		result = input_one + input_two;
		
		for (i=28;i<33;i++){
			siu_gpdo_ptr[i] = (result & 0x01);
			result = result >> 1;
		}
	}
	
	init_ECS(1); /* Call this function with lab number = 1 to init processor */ 
}