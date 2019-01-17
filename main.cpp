#include "balena.h"
#include "Firmata.h"
#include "Serial.h"

int main(void)
{
	BalenaClass Balena;
	firmata::FirmataClass Firmata;

	SerialClass Serial(0,9600);

	Firmata.begin(9600);

  while (1) {
	  // Do Firmata things
  };
  return 0;
}
