#include "balena.h"
#include "Firmata.h"
#include "Serial.h"

int main(void)
{
	balena::BalenaClass Balena;
	firmata::FirmataClass Firmata;

	SerialClass Serial(0,9600);

	Balena.init();
	Firmata.begin(9600);

  while (1) {
	  // Do Firmata things
  };
  return 0;
}
