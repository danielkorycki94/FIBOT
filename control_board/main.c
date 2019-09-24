
#include "library.h"


void main(void)
{
    WDT_A_hold(WDT_A_BASE);
    configureCS();
    configureIO();
    cofigureTimers();
    conigureUART();
    configureI2C();
    configureADC();
    __delay_cycles(16000000);
    __bis_SR_register(GIE);


    for (;;)
    {
    }

}
