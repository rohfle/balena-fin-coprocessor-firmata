#include "balena.h"
#include "Firmata.h"

using namespace firmata;

int main(void)
{

initBalena();
Firmata.begin();

  while (1) {
	  // Do Firmata things
  };
  return 0;
}
