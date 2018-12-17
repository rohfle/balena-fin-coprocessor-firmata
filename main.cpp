#include "balena.h"
#include "Firmata.h"

int main(void)
{
	balena::BalenaClass Balena;
	firmata::FirmataClass Firmata;

	Balena.init();
	Firmata.begin(9600);

  while (1) {
	  // Do Firmata things
  };
  return 0;
}
