typedef union  
{ 
	/* Pad Configuration Registers */ 
	volatile unsigned short REG; 
	
	struct { 
		volatile unsigned short :3; 
		volatile unsigned short PA:3; 
		volatile unsigned short OBE:1; 
		volatile unsigned short IBE:1; 
		volatile unsigned short DSC:2; 
		volatile unsigned short ODE:1; 
		volatile unsigned short HYS:1; 
		volatile unsigned short SRC:2; 
		volatile unsigned short WPE:1; 
		volatile unsigned short WPS:1; 
	} FIELDS3; 
} SIU_PCR;