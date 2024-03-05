/*===========================================================================
 * DESCRIPTION:
 *
 * This example exhibits basic functionality of the STR71x and STR9x REva
 * daughterboards. It displays a counter on the REva LEDs.
 *
 * JUMPERS:
 * On the REva board, you must set the following jumpers:
 *  INPUT:  POT
 *  OUTPUT: D0->D7
 *
 * NOTE:
 *  Make sure that the boot mode selected on the board is the same as the mode
 *  selected in the target options. (RAM by default, i.e. Boot0=0 and Boot1=1)
 *
 *---------------------------------------------------------------------------
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *---------------------------------------------------------------------------
 * Copyright (c) IoTize S.A.S., 2006-2017
 *==========================================================================*/

#include "strx.h"

int delaycnt;
int k;

//this is the counter
volatile unsigned short i = 0x0550;

//uncomment this line if you want the main function to be located
// in the FLASH BLOCK 1 (by default, it goes in BLOCK 0)
//int main() __attribute__ ((section (".b1text")));

int main( void )
{
    //GPIO6 goes to the LEDs
    SCU->PRR1  |= 0x00100000;  // release P6 reset
    SCU->PCGR1 |= 0x00100000; // clock gating P6
    SCU->GPIOOUT[6] = 0x5555; // enable GPIO6 OUTPUT for all pins
    GPIO6->DIR = 0x0FF;       // Dir output on GPIO6 for all pins

    //GPIO3 goes to the buttons
    SCU->PRR1  |= 0x00020000;  // release P3 reset
    SCU->PCGR1 |= 0x00020000; // clock gating P3
    SCU->GPIOIN[3] |= 0x60; // enable GPIO3 INPUT for pins 5-6
    GPIO3->DIR &= 0x09F;       // Dir input on GPIO3 for pins 5-6

    //GPIO4 goes to the switches
    SCU->PRR1  |= 0x00040000;  // release P4 reset
    SCU->PCGR1 |= 0x00040000; // clock gating P4
    SCU->GPIOIN[3] |= 0x0F; // enable GPIO4 INPUT for pins 0-3
    GPIO4->DIR &= 0x0F0;       // Dir input on GPIO4 for pins 0-3

    while ( 1 )
    {
        //increment the counter
        i++;

        //read the value of the buttons
        k = ( GPIO_READDATA( GPIO3 ) & 0x60U ) | ( GPIO_READDATA( GPIO4 ) & 0x0FU ) | 0x90U;

        //print value of the counter and buttons on the LEDs
        GPIO_WRITEDATA( GPIO6, k );

        //delay
        for ( delaycnt = 0; delaycnt < 10000; delaycnt++ );
    }

    //this will never be executed. (and is probably optimized out)
    // but it avoids a warning when compiling
    return 0;
}
