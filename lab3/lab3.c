//Jose Mastrangelo, Anton ter Vehn, Manish Prajapat
//Group 06

#include <ecs.h>
#include <mpc5553.h>
#include <isr.h>
#include <qadc.h>
//#include <unistd.h>

//#define LAB_3_4_1
//#define LAB_3_4_2
//#define LAB_3_4_3
//#define LAB_3_4_4
#define LAB_3_4_5
#define LAB_3_4_6

#ifdef LAB_3_4_1
void main(){
	vuint16_t iAnalogQ1;
	vuint16_t ianalog_prev =0;
    char buff[32];
	qadcInit_single(1);
	init_ECS(3);
	init_COM(1,115200);

	while(1){
		iAnalogQ1 = qadcReadQ1();
	
		if( iAnalogQ1!= ianalog_prev){
			sprintf(buff, "\r\n Resistance = %i", iAnalogQ1);
			serial_puts(1,buff);
		}
		ianalog_prev = iAnalogQ1;
	}
	
}
#endif

#ifdef LAB_3_4_2
void main(){
	uint32_t inputs[8] = {0,1,2,3,4,5,6,7};
	vuint16_t iAnalogQ2[8];
	vuint16_t ianalog_prev =0;
	uint8_t channel;
	
	char buff[32];
	qadcInit_conti(8,inputs);
	init_ECS(3);
	init_COM(1,115200);
	
	while(1){
		for (channel=0;channel <8; channel++){
			iAnalogQ2[channel] = qadcReadQ2(channel);
		
			//if( iAnalogQ1!= ianalog_prev){
				sprintf(buff, "CH%i: %i \r\n", channel, iAnalogQ2[channel]);
				serial_puts(1,buff);
			//}
			//ianalog_prev = iAnalogQ1;
		}
		//sprintf("\r\n");
		//sleep(1);
	}
	
}
#endif

#ifdef LAB_3_4_3
void main(){
	vuint8_t *siu_gpdo_ptr;
	vuint16_t iAnalogQ1;
	vuint16_t ianalog_prev =0;
    char buff[32];
	qadcInit_single(1);
	init_ECS(3);
	init_COM(1,115200);
	
	siu_gpdo_ptr = (unsigned char*)(0Xc3f90600);
	
	SIU.PCR[28].B.PA = 0;  /* GPIO */ 
	SIU.PCR[28].B.OBE = 1; /* Output */ 
	SIU.PCR[28].B.WPE = 0; 

	while(1){
		siu_gpdo_ptr[28] = 1;
		iAnalogQ1 = qadcReadQ1();
		siu_gpdo_ptr[28] = 0;
/* 		if( iAnalogQ1!= ianalog_prev){
			sprintf(buff, "\r\n Resistance = %i", iAnalogQ1);
			serial_puts(1,buff);
		} */
		//
	}
	
}
#endif

#ifdef LAB_3_4_4
void main(){
	vuint8_t *siu_gpdo_ptr;
	vuint16_t iAnalogQ1;
	vuint16_t ianalog_prev =0;
	uint16_t debug;
    char buff[32];
	qadcInit_single(0);
	init_ECS(3);
	init_COM(1,115200);

	siu_gpdo_ptr = (unsigned char*)(0Xc3f90600);
	
	SIU.PCR[28].B.PA = 0;  /* GPIO */ 
	SIU.PCR[28].B.OBE = 1; /* Output */ 
	SIU.PCR[28].B.WPE = 0; 
	
	while(1){
	  
		iAnalogQ1 = qadcReadQ1();
		
		iAnalogQ1 = iAnalogQ1 >> 2;
 		if(iAnalogQ1 > 0x800){
			siu_gpdo_ptr[28] = 1;
			debug =1;
		}
		else{
			siu_gpdo_ptr[28] = 0;
			debug=0;
			
		} 
	}
	
}
#endif

#ifdef LAB_3_4_5
void main(){
	vuint8_t *siu_gpdo_ptr;
	vuint16_t iAnalogQ1;
	vuint16_t ianalog_prev =0;
	uint16_t debug;
    char buff[32];
	qadcInit_single(0);
	init_ECS(3);
	init_COM(1,115200);

	siu_gpdo_ptr = (unsigned char*)(0Xc3f90600);
	
	SIU.PCR[28].B.PA = 0;  /* GPIO */ 
	SIU.PCR[28].B.OBE = 1; /* Output */ 
	SIU.PCR[28].B.WPE = 0; 
	
	while(1){
	  
		iAnalogQ1 = qadcReadQ1();
		
		iAnalogQ1 = iAnalogQ1 >> 2;
 		if(iAnalogQ1 > 0x800){
			siu_gpdo_ptr[28] = 1;
			debug =1;
		}
		else{
			siu_gpdo_ptr[28] = 0;
			debug=0;
			
		} 
	}
	
}
#endif
