//Jose Mastrangelo, Anton ter Vehn, Manish Prajapat
//Group 06

/* Typedefs and processor initialization */ 
#include <ecs.h> 
#include <mpc5553.h>
#include <math.h>
#include <serial.h>

void main(){ 
	int i; 
    vuint8_t result, remainder;
	char byte_in;
	char byte_prev;
	int finished = 0;
	
	/* pointers to registers */ 
	volatile unsigned char *siu_gpdi_ptr; 
	volatile unsigned char *siu_gpdo_ptr; 
	siu_gpdo_ptr = (unsigned char*)(0Xc3f90600);  /* SIU_BASE + 0X600 for gpdo */ 
	siu_gpdi_ptr = (unsigned char*)(0Xc3f90800);  /* SIU_BASE + 0X800 for gpdi */ 
	
	/* configure output leds */ 
	for(i=28; i<33; i++) 
	{ 
		SIU.PCR[i].B.PA = 0;  /* GPIO */ 
		SIU.PCR[i].B.OBE = 1; /* Output */ 
		SIU.PCR[i].B.WPE = 0; 
	} 
	byte_prev = 0;
	init_COM(1,115200);
	serial_puts(1,"\n\n\n\n\n\n\n\n\rSerial Output Enabled.");
	while (!finished){
		if(serial_readyToReceive(1)){
			byte_in = serial_getchar(1);
			//The following two if statesments were created as the two buttons 
			// 0 and 8 did not give the expected ascII code. Instead, they corresponded
			// to 112 and 120.
			if(byte_in == 112)
				byte_in = 48;
			if(byte_in == 120)
				byte_in = 56;
			switch(byte_in){
			case '0':
				byte_in = byte_in - 48;
				result = byte_in + byte_prev;
				byte_prev = byte_in;
			break;
			case '1':
				byte_in = byte_in - 48;
				result = byte_in + byte_prev;
				byte_prev = byte_in;
			break;
			case '2':
				byte_in = byte_in - 48;
				result = byte_in + byte_prev;
				byte_prev = byte_in;
			break;
			case '3':
				byte_in = byte_in - 48;
				result = byte_in + byte_prev;
				byte_prev = byte_in;
			break;
			case '4':
				byte_in = byte_in - 48;
				result = byte_in + byte_prev;
				byte_prev = byte_in;
			break;
			case '5':
				byte_in = byte_in - 48;
				result = byte_in + byte_prev;
				byte_prev = byte_in;
			break;
			case '6':
				byte_in = byte_in - 48;
				result = byte_in + byte_prev;
				byte_prev = byte_in;
			break;
			case '7':
				byte_in = byte_in - 48;
				result = byte_in + byte_prev;
				byte_prev = byte_in;
			break;
			case '8':
				byte_in = byte_in - 48;
				result = byte_in + byte_prev;
				byte_prev = byte_in;
			break;
			case '9':
				byte_in = byte_in - 48;
				result = byte_in + byte_prev;
				byte_prev = byte_in;
			break;	
			
			}
		}	
		
		remainder = result; 
		for (i=28;i<33;i++){
			siu_gpdo_ptr[i] = (remainder & 0x01);
			remainder = remainder >> 1;
		}
	}
	
	init_ECS(1); /* Call this function with lab number = 1 to init processor */ 
}