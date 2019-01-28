#include "balena.h"
#include "Firmata.h"
#include "Serial.h"

int main_test(void)
{
	BalenaClass Balena;
//	firmata::FirmataClass Firmata;

	SerialClass Serial(1,115200);

	unsigned char buf[32] = "12345678910111213141517181920\r\n";

	Serial.begin(115200);

	Serial.write(buf);

//	Firmata.begin(9600);

	while(1){
			for(volatile long i=0; i<300000; i++);
		        // Turn on the LED
	    		digitalWrite(3,1);

		        // Add some delay
//		        for(volatile long i=0; i<300000; i++)
//		            ;
		    	Serial.write(buf);

		    	delay(1000);

		        // Turn off the LED
		    	digitalWrite(3,0);

		        // Add some more delay

		    	delay(1000);

//		        for(volatile long i=0; i<300000; i++)
//		            ;
	};



  while (1) {
	  // Do Firmata things
  };

  return 0;
}
