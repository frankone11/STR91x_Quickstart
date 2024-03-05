/********************************************************************

UART.c

This file implements the io_putchar and io_getchar functions
 for all the STRx devices.

It is the source code for the IO_Putchar.a default library.

It has been written by using the ST libs and then inlining and
 simplifying the code for optimization.

You can duplicate it and include the copy in your project,
 then you'll be able to modify it for using different parameters:
 baudrate, stop bits, other UART, etc.

*********************************************************************/



#include "strx.h"

#if ( _STR9x_ )

#define __UART0 0x8
#define __GPIO3 0x20000
#define RESET 0

/*
    #define _Main_Crystal 25000

    #include "91x_map.h"
    #include "91x_scu.h"
    #include "91x_gpio.h"
    #include "91x_uart.h"
*/
#endif // _STR9x_

unsigned char __io_init_done = 0;

#if ( _STR7x_ )


#define UART0_Rx_Pin (0x0001<<8)   // TQFP 64: pin N° 63 , TQFP 144 pin N° 143
#define UART0_Tx_Pin (0x0001<<9)   // TQFP 64: pin N° 64 , TQFP 144 pin N° 144

#define UART_FIFOEnableBit      10
#define UART_TxFull             0x0200
#define UART_TxEmpty            0x0002
#define UART_RxHalfFull         0x0100
#define UART_RxBufFull          0x0001
#define UART_TimeOutIdle        0x0080

#if ( _STR73x_ )
unsigned int __io_Main_Osc = 8000000;
#endif // _STR73x_
#if ( _STR71x_ )
unsigned int __io_Main_Osc = 4000000;
#endif // _STR71x_

#if ( _STR9x_ )
unsigned int __io_Main_Osc = 25000000;
#endif // _STR71x_

//called by the user app to tell us what is the main osc frequency
void __io_SetMainOscFreq( unsigned int NewFreq )
{
    __io_Main_Osc = NewFreq; //save new freq value
    __io_init_done = 0; //force reinit at next putchar
}

#if ( _STR73x_ )
/*******************************************************************************
* Function Name  : PRCCU_GetFrequencyValue
* Description    : Retrieves the value of the clock passed as parameter.
* Input          : PRCCU_CLOCK : the Clock to get its value .
*                  This parameter can be one of the following values:
*                   - PRCCU_CLOCK_EXT
*                   - PRCCU_CLOCK_MCLK
* Output         : None
* Return         : The value of the clock passed as parameter in Hz.
*******************************************************************************/
#define CMU_CKSEL0_CKOSC  0x0001
#define RC_oscillator    2340000   /* Typical Reset Value of the Internal RC in Hz */
#define PRCCU_DIV2_Enable  0x8000
#define PRCCU_CK2_16_Disable  0x08
#define PRCCU_CSU_CKSEL_Enable  0x01
unsigned int myPRCCU_GetFrequencyValue()
{
    unsigned char MUL_Factor = 1, DIV_Factor = 1;
    unsigned int Tmp = 0, Tmp_CLOCK2 = 0, CLOCK1 = 0;

    if ( ( CMU->CTRL & CMU_CKSEL0_CKOSC ) == CMU_CKSEL0_CKOSC )
    {
        CLOCK1 = __io_Main_Osc ;
    }
    else
    {
        CLOCK1 = RC_oscillator;
    }

    /* Get the value of the fMCLK to CPU and peripherals */
    /* Depending on the status of the DIV2_EN bit get the CLOCK2 value  */
    if ( ( PRCCU->CFR & PRCCU_DIV2_Enable ) != 0 )
    {
        Tmp_CLOCK2 = CLOCK1 / 2;  /* Set CLOCK2 value to Half CLOCK1 */
    }
    else /* DIV2_EN bit is reset */
    {
        /* Set CLOCK2 value equal to CLOCK1 */
        Tmp_CLOCK2 =  CLOCK1;
    }

    if ( ( PRCCU->CFR & PRCCU_CK2_16_Disable ) != 0 )
    {
        if ( ( PRCCU->CFR & PRCCU_CSU_CKSEL_Enable ) != 0 )
        {
            /* Get the PLL Multiplication and the Division Factor */
            Tmp = PRCCU->PLLCR;
            MUL_Factor = ( ( Tmp & 0x30 ) >> 4 );
            switch ( MUL_Factor )
            {
            case 0: MUL_Factor = 20; break;
            case 1: MUL_Factor = 12; break;
            case 2: MUL_Factor = 28; break;
            case 3: MUL_Factor = 16; break;
            }

            DIV_Factor = ( Tmp & 0x07 ) + 1;

            /* return the value of the Main Clock MCLK*/
            return( Tmp_CLOCK2 * MUL_Factor / DIV_Factor ) ;
        }
        else /* CSU_CKSEL bit Reset */
        {
            return Tmp_CLOCK2; /* Set the MCLK value to CLOCK2  */
        }
    }
    else /* CK2_16 bit reset */
    {
        /* Set the MCLK value to CLOCK2 / 16  */
        return ( Tmp_CLOCK2 / 16 ) ;
    }
}
#endif // _STR73x_

#endif // _STR7x_

#if ( _STR9x_ )
unsigned int __io_Main_Osc = 25000000;

//called by the user app to tell us what is the main osc frequency
void __io_SetMainOscFreq( unsigned int NewFreq )
{
    __io_Main_Osc = NewFreq; //save new freq value
    __io_init_done = 0; //force reinit at next putchar
}

#endif // _STR9x_

void __io_init( void )
{
// Configure the GPIO pins

#if ( _STR9x_ ) // P3.0 and P3.1

    //GPIO_InitTypeDef gpio_init;
    //UART_InitTypeDef uart_init;

    /* Enable the UART0 Clock */
    //SCU_APBPeriphClockConfig(__UART0, ENABLE);
    SCU->PCGR1 |= __UART0;

    /* Enable the GPIO3 Clock */
    //SCU_APBPeriphClockConfig(__GPIO3, ENABLE);
    SCU->PCGRO |= __GPIO3;

    //GPIO_DeInit(GPIO3);
    SCU->PRR1 &= ~__GPIO3;
    SCU->PRR1 |= __GPIO3;
    SCU->GPIOTYPE[0x03] = 0x0000 ;
    SCU->GPIOOUT[0x03]  = 0x0000;
    SCU->GPIOIN[0x03]   = 0x0000;

    /*Configure UART0_Rx pin GPIO3.0*/
#if 0
    gpio_init.GPIO_Direction = GPIO_PinInput;
    gpio_init.GPIO_Pin = GPIO_Pin_0;
    gpio_init.GPIO_Type = GPIO_Type_PushPull ;
    gpio_init.GPIO_IPConnected = GPIO_IPConnected_Enable;
    gpio_init.GPIO_Alternate = GPIO_InputAlt1;
    GPIO_Init( GPIO3, &gpio_init );
#else
    /* Select pin direction */
    GPIO3->DIR &= ~1;

    /*Output ALternate 0*/
    SCU->GPIOOUT[3] &= ~0x3;

    /*Type configuration: PushPull or Open Collector*/
    SCU->GPIOTYPE[3] &= ~0x1;

    /*IP Connected disable*/
    SCU->GPIOIN[3] &= ~0x1;
    /*IP Connected enable*/
    SCU->GPIOIN[3] |= 0x1;
#endif

    /*Configure UART0_Tx pin GPIO3.1*/
#if 0
    gpio_init.GPIO_Direction = GPIO_PinOutput;
    gpio_init.GPIO_Pin = GPIO_Pin_1;
    gpio_init.GPIO_Alternate = GPIO_OutputAlt2;
    GPIO_Init( GPIO3, &gpio_init );
#else
    GPIO3->DIR |= 2;

    /*Output ALternate 0*/
    SCU->GPIOOUT[3] &= ~( 0xC );

    /*Output ALternate 2*/
    SCU->GPIOOUT[3] |= 0x8;

    /*Type configuration: PushPull or Open Collector*/
    SCU->GPIOTYPE[3] &= ~( 0x2 ) ;

    /*IP Connected disable*/
    SCU->GPIOIN[3] &= ~( 0x2 ) ;
    /*IP Connected enable*/
    SCU->GPIOIN[3] |= 0x2;
#endif

    //UART_DeInit(UART0);
    /* Reset the UARTx registers values */
    //SCU_APBPeriphReset(__UART0,ENABLE);
    SCU->PRR1 &= ~__UART0;                  /*APB peripheral held in Reset*/
    //SCU_APBPeriphReset(__UART0,DISABLE);
    SCU->PRR1 |= __UART0;

#if 0
    uart_init.UART_WordLength          = UART_WordLength_8D;
    uart_init.UART_StopBits            = UART_StopBits_1;
    uart_init.UART_Parity              = UART_Parity_No;
    uart_init.UART_BaudRate            = 9600;
    uart_init.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
    uart_init.UART_Mode                = UART_Mode_Tx_Rx;
    uart_init.UART_FIFO                = UART_FIFO_Disable;
    uart_init.UART_TxFIFOLevel         = UART_FIFOLevel_1_2;
    uart_init.UART_RxFIFOLevel         = UART_FIFOLevel_1_2;
    UART_Init( UART0, &uart_init );
#else

#define UART_IrDA_Disable_Mask          0xFFFD  /* IrDA Disable Mask */
#define UART_IrDA_Enable_Mask           0x0002  /* IrDA Enable Mask */
#define IrDA_LowPower_Enable_Mask       0x0004 /*IrDA lower power mode enable*/
#define IrDA_LowPower_Disable_Mask      0xFFFB /*IrDA lower power mode enable*/

    /* UART Mask */
#define UART_Enable_Mask            0x0001  /* UART Enable Mask */
#define UART_Disable_Mask           0xFFFE  /* UART Disable Mask */

    /* UART LoopBack */
#define UART_LoopBack_Disable_Mask      0xFF7F  /* LoopBack Disable Mask */
#define UART_LoopBack_Enable_Mask       0x0080  /* LoopBack Enable Mask */

#define UART_WordLength_Mask            0xFF9F  /* UART Word Length Mask */
#define UART_Parity_Mask                0xFF79  /* UART Parity Mask */
#define UART_HardwareFlowControl_Mask   0x3FFF  /* UART Hardware Flow Control Mask */
#define UART_TxRxFIFOLevel_Mask         0xFFC0  /* UART Tx Rx FIFO Level Mask */
#define UART_BreakChar_Mask             0x0001  /* UART Break Character send Mask*/
#define UART_FLAG_Mask                  0x1F    /* UART Flag Mask */
#define UART_Mode_Mask                  0xFCFF  /* UART Mode Mask */
#define UART_RTS_LowLevel_Mask          0x0800  /* RTS signal is low */
#define UART_RTS_HighLevel_Mask         0xF7FF  /* RTS signal is High */
#define UART_DTR_LowLevel_Mask          0x0400  /* DTR signal is low */
#define UART_DTR_HighLevel_Mask         0xFBFF  /* DTR signal is High */
#define UART_ClearFlag_Mask             0xAA    /* Clear Flag Mask */

    /* UART Data Length */
#define UART_WordLength_5D        0x0000  /* 5 bits Data */
#define UART_WordLength_6D        0x0020  /* 6 bits Data */
#define UART_WordLength_7D        0x0040  /* 7 bits Data */
#define UART_WordLength_8D        0x0060  /* 8 bits Data */

    /* UART Stop Bits */
#define UART_StopBits_1         0xFFF7  /* Disable two stop bit is transmitted
    at the end of frame */
#define UART_StopBits_2         0x0008  /* Enable Two stop bits are transmitted
    at the end of frame */
    /* UART Parity */
#define UART_Parity_No                0x0000  /* Parity Disable */
#define UART_Parity_Even          0x0006  /* Even Parity */
#define UART_Parity_Odd               0x0002  /* Odd Parity */
#define UART_Parity_OddStick          0x0082  /* 1 is transmitted as bit parity */
#define UART_Parity_EvenStick         0x0086  /* 0 is transmitted as bit parity */

    /* UART Hardware Flow Control */
#define UART_HardwareFlowControl_None      0x0000  /* HFC Disable */
#define UART_HardwareFlowControl_RTS       0x4000  /* RTS Enable */
#define UART_HardwareFlowControl_CTS       0x8000  /* CTS Enable */
#define UART_HardwareFlowControl_RTS_CTS   0xC000  /* CTS and RTS Enable */

    /* UART Mode */
#define UART_Mode_Rx                  0x0200  /* UART Rx Enabled */
#define UART_Mode_Tx                  0x0100  /* UART Tx Enbled */
#define UART_Mode_Tx_Rx               0x0300  /* UART Tx and Rx Enabled */

    /* UART FIFO */
#define UART_FIFO_Disable           0xFFEF  /* FIFOs Disable */
#define UART_FIFO_Enable            0x0010  /* FIFOs Enable */

    /* UART Interrupt definition */
#define UART_IT_OverrunError          0x0400  /* Overrun Error interrupt mask */
#define UART_IT_BreakError        0x0200  /* Break Error interrupt mask */
#define UART_IT_ParityError       0x0100  /* Parity Error interrupt mask */
#define UART_IT_FrameError        0x0080  /* Frame Error interrupt mask */
#define UART_IT_ReceiveTimeOut        0x0040  /* Receive Time Out interrupt mask */
#define UART_IT_Transmit              0x0020  /* Transmit interrupt mask */
#define UART_IT_Receive               0x0010  /* Receive interrupt mask */
#define UART_IT_DSR               0x0008  /* DSR interrupt mask */
#define UART_IT_DCD               0x0004  /* DCD interrupt mask */
#define UART_IT_CTS               0x0002  /* CTS interrupt mask */
#define UART_IT_RI                0x0001  /* RI interrupt mask */

    /* UART DMA On Error */
#define UART_DMAOnError_Enable        0xFFFB  /* DMA receive request enabled
    when the UART error interrupt
    is asserted. */
#define UART_DMAOnError_Disable       0x0004  /* DMA receive request disabled
    when the UART error interrupt
    is asserted. */
    /* UART DMA Request */
#define UART_DMAReq_Tx                0x02      /* Transmit DMA Enable */
#define UART_DMAReq_Rx                0x01      /* Receive DMA Enable */

    /* UART FLAG */
#define UART_FLAG_OverrunError        0x23    /* Overrun error flag */
#define UART_FLAG_Break               0x22    /* break error flag */
#define UART_FLAG_ParityError         0x21    /* parity error flag */
#define UART_FLAG_FrameError          0x20    /* frame error flag */
#define UART_FLAG_RI                  0x48    /* RI flag */
#define UART_FLAG_TxFIFOEmpty         0x47    /* Transmit FIFO Empty flag */
#define UART_FLAG_RxFIFOFull          0x46    /* Receive FIFO Full flag */
#define UART_FLAG_TxFIFOFull          0x45    /* Transmit FIFO Full flag */
#define UART_FLAG_RxFIFOEmpty         0x44    /* Receive FIFO Empty flag */
#define UART_FLAG_Busy                0x43    /* UART Busy flag */
#define UART_FLAG_DCD                 0x42    /* DCD flag */
#define UART_FLAG_DSR                 0x41    /* DSR flag */
#define UART_FLAG_CTS                 0x40    /* CTS flag */
#define UART_RawIT_OverrunError       0x6A    /* Overrun Error Raw IT flag */
#define UART_RawIT_BreakError         0x69    /* Break Error Raw IT flag */
#define UART_RawIT_ParityError        0x68    /* Parity Error Raw IT flag */
#define UART_RawIT_FrameError         0x67    /* Frame Error Raw IT flag */
#define UART_RawIT_ReceiveTimeOut     0x66    /* ReceiveTimeOut Raw IT flag */
#define UART_RawIT_Transmit       0x65    /* Transmit Raw IT flag */
#define UART_RawIT_Receive        0x64    /* Receive Raw IT flag */
#define UART_RawIT_DSR                0x63    /* DSR Raw IT flag */
#define UART_RawIT_DCD                0x62    /* DCD Raw IT flag */
#define UART_RawIT_CTS                0x61    /* CTS Raw IT flag */
#define UART_RawIT_RI                 0x60    /* RI Raw IT flag */

    /*IrDAx select*/
#define IrDA0 0x01                             /*IrDA0 select*/
#define IrDA1 0x02                             /*IrDA0 select*/
#define IrDA2 0x03                             /*IrDA0 select*/

//void UART_Init(UART_TypeDef* UARTx, UART_InitTypeDef* UART_InitStruct)
    {

        long long UART_MainClock = 0;
        unsigned int IntegerDivider = 0;
        unsigned int FractionalDivider = 0;

        /* Clear the LCR[6:5] bits */
        /* Choose Stop Bits - One Stop Bits */
        /* Configure the Parity - Clear the LCR[7]and LCR[2:1] bits */
        UART0->LCR &= ( UART_WordLength_Mask & UART_StopBits_1 & UART_Parity_Mask );

        /* Set the LCR[6:5] bits according to UART_WordLength value */
        /* Set the LCR[7]and LCR[2:1] bits according to UART_Parity value */
        UART0->LCR |= ( UART_WordLength_8D | UART_Parity_No );

        /* Configure the BaudRate */
        //UART_MainClock = (SCU_GetMCLKFreqValue())*1000;
        if ( ( SCU->CLKCNTR & 0x3 ) == 0x2 ) UART_MainClock = ( unsigned int )( __io_Main_Osc );
        else
    {
        if ( ( SCU->CLKCNTR & 0x3 ) == 0x1 ) UART_MainClock = ( unsigned int )( 32000 );
            else //UART_MainClock = (SCU_GetPLLFreqValue())*1000;
            {
                unsigned char PLL_M;
                unsigned char PLL_N;
                unsigned char PLL_P;

                PLL_M = SCU->PLLCONF & 0xFF;
                PLL_N = ( SCU->PLLCONF & 0xFF00 ) >> 8;
                PLL_P = ( SCU->PLLCONF & 0x70000 ) >> 16;

                if ( ( PLL_M > 0 ) && ( PLL_N > 0 ) )
                    UART_MainClock = ( unsigned int )( ( ( __io_Main_Osc * 2 ) * PLL_N ) / ( PLL_M << PLL_P ) );

                else UART_MainClock = 0;
            }
        }

    if ( ( SCU->CLKCNTR & 0x200 ) != 0x200 )
{
    UART_MainClock = UART_MainClock / 2;
}
/* Determine the integer part */
IntegerDivider = ( ( 100 ) * ( UART_MainClock ) / ( 16 * ( 9600 ) ) );
UART0->IBRD = IntegerDivider / 100;

/* Determine the fractional part */
FractionalDivider = IntegerDivider - ( 100 * ( UART0->IBRD ) );
UART0->FBRD = ( ( ( ( FractionalDivider * 64 ) + 50 ) / 100 ) );

/* Choose the Hardware Flow Control */
/* Clear the CR[15:14] bits */
/* Clear the CR[9:8] bits */
UART0->CR &= ( UART_HardwareFlowControl_Mask & UART_Mode_Mask );
/* Set the CR[15:14] bits according to UART_HardwareFlowControl value */
/* Configure the UART mode - Set the CR[9:8] bits according to UART_Mode value */
UART0->CR |= ( UART_HardwareFlowControl_None | UART_Mode_Tx_Rx );

/* Disable the FIFOs */
UART0->LCR &= UART_FIFO_Disable;
}



#endif


#if 0
    UART_LoopBackConfig( UART0, DISABLE );
#else
    /* Disable the LoopBack mode of the specified UART */
#define UART_LoopBack_Disable_Mask      0xFF7F  /* LoopBack Disable Mask */
    UART0->CR &= UART_LoopBack_Disable_Mask;
#endif

#if 0
    UART_Cmd( UART0, ENABLE );
#else
    /* Enable the selected UART by setting the UARTEN bit in the CR register */
#define UART_Enable_Mask            0x0001  /* UART Enable Mask */
    UART0->CR |= UART_Enable_Mask;
#endif

#endif // _STR9x_

#if ( _STR7x_ )

#if ( _STR73x_ )
    unsigned char delay = 0;
    /* UART0 Clock Enable */
    CFG->PCGR0 |= 0x0001 << 4;
    for ( delay = 8 ; delay != 0 ; delay-- ); /* Peripheral Stabilization */
    /* GPIO6 Clock Enable */
    CFG->PCGR0 |= 0x0001 << 24;
    for ( delay = 8 ; delay != 0 ; delay-- ); /* Peripheral Stabilization */

#define UARTGPIO GPIO6
#endif // _STR73x_

#if ( _STR71x_ )
#define UARTGPIO GPIO0
#endif // _STR71x_

    UARTGPIO->PC0 |= UART0_Tx_Pin;
    UARTGPIO->PC1 |= UART0_Tx_Pin;
    UARTGPIO->PC2 |= UART0_Tx_Pin;

    UARTGPIO->PC0 &= ~UART0_Rx_Pin;
    UARTGPIO->PC1 |= UART0_Rx_Pin;
    UARTGPIO->PC2 &= ~UART0_Rx_Pin;

    /* Configure the UART0 as following:
        - Baudrate = 9600 Bps
        - No parity
        - 8 data bits
        - 1 stop bit */

    UART0->CR = 0x00A9;

    // Reset the FIFOs
    UART0->RxRSTR = 0xFFFF;

    //UART_FifoReset  (UART0 , UART_TxFIFO); // Reset the UART_TxFIFO
    UART0->TxRSTR = 0xFFFF;

#if ( _STR73x_ )

    /* UART Configuration */

    {
        unsigned int tmpBaudRate = 0;
        /*Configure BaudRate*/
        tmpBaudRate = ( unsigned int )( ( myPRCCU_GetFrequencyValue( /*PRCCU_CLOCK_MCLK*/ ) * 10 ) / ( 16 * 9600 ) ); //BaudRate 9600

        /*Search the reload value (Integer)*/
        if ( tmpBaudRate - ( ( tmpBaudRate / 10 ) * 10 ) < 5 )
        {
            UART0->BR = tmpBaudRate / 10;
        }
        else
        {
            UART0->BR = ( tmpBaudRate / 10 ) + 1;
        }
    }

#endif // _STR73x_
#if ( _STR71x_ )

    // RTC Oscillator Frequency value = 32 768 Hz
#define RCCU_RTC_Osc  32768

#define RCCU_Div2_Mask  0x00008000
#define RCCU_CK2_16_Mask    0x00000008
#define RCCU_CSU_CKSEL_Mask 0x00000001
#define RCCU_MX_Mask   0x00000030
#define RCCU_MX_Index  0x04
#define RCCU_DX_Mask   0x00000007
    {
        unsigned int Tmp;
        unsigned char Div = 1;
        unsigned char Mul = 1;

        /* Configure the UARTX as following:
           - Baudrate = 9600 Bps
           - No parity
           - 8 data bits
           - 1 stop bit */
        Tmp = ( RCCU->CFR & RCCU_Div2_Mask ) ? ( __io_Main_Osc / 2 ) :  __io_Main_Osc;

        if ( ( RCCU->CCR & 0x04 ) == 0x04 )
        {
            Tmp = RCCU_RTC_Osc;
        }

        else if ( ( RCCU->CFR & RCCU_CK2_16_Mask ) == 0 )
        {
            Div = 16;
        }
        else if ( RCCU->CFR & RCCU_CSU_CKSEL_Mask )
        {
            Mul = ( RCCU->PLL1CR & RCCU_MX_Mask ) >> RCCU_MX_Index;
            switch ( Mul )
            {
            case 0: Mul = 20; break;
            case 1: Mul = 12; break;
            case 2: Mul = 24; break;
            case 3: Mul = 16; break;
            }
            Div = ( RCCU->PLL1CR & RCCU_DX_Mask ) + 1;
        }

        Div <<=  PCU->PDIVR & 0x3;

        //UART0->BR = (u16)(((Tmp * Mul) / Div)/(16*9600));
        UART0->BR = ( unsigned short )( ( ( Tmp * Mul ) / Div ) / ( 16 * 9600 ) );
    }

#endif // _STR71x_

    UART0->CR = 0x01A9; // Enable Rx

#endif // _STR7x_

    // IO init done!
    __io_init_done = 1;
}






/* send the character on UART0. */
void __io_putchar( int c )
{
    //init UART if needed
    if ( !__io_init_done )
    {
        __io_init();
    }

    /* \n is not enough. Need \r too! */
    if ( c == 0x0A )
    {
        __io_putchar( 0x0D );
    }
#if ( _STR7x_ )
    if ( UART0->CR & ( 0x0001 << UART_FIFOEnableBit ) ) // if FIFO ENABLED
    {
        while ( ( UART0->SR & UART_TxFull ) ); // while the UART_TxFIFO contain 16 characters.
    }
    else                  // if FIFO DISABLED
    {
        while ( !( UART0->SR & UART_TxEmpty ) ); // while the transmit shift register not empty
    }

    UART0->TxBUFR = c;

    while ( !( UART0->SR & UART_TxEmpty ) ); // wait until the data transmission is finished
#endif // _STR7x_
#if ( _STR9x_ )
    //while(UART_GetFlagStatus(UART0, UART_FLAG_Busy) == SET); // we wait while the UART transmitter is busy
    while ( ( UART0->FR & 8 ) != RESET ); // we wait while the UART transmitter is busy

    //UART_SendData(UART0, c);
    UART0->DR = c;

#endif // _STR9x_
}

int
putchar( c )
int c;
{
    __io_putchar( c );
    return c;
}

//printf from newlib-nano will call _write
__attribute__ ((weak)) int _write(
   int fd,
   const void *buffer,
   unsigned int count 
)
{
    unsigned int cnt = 0;
    char * buf = ((char *)(buffer));

    if ( ! buf )
        return 0;
    
    while ( cnt < count )
    {
        __io_putchar( buf [ cnt ] );
        cnt ++;
    }
    
    return cnt;
}


int __io_getchar()
{
    unsigned char  value;
    unsigned short wStatus;

    //init UART if needed
    if ( !__io_init_done )
    {
        __io_init();
    }
#if ( _STR7x_ )
    while ( !( UART0->SR & UART_RxBufFull ) ); // If data received
    UART0->TOR = 0xFF; // reload the Timeout counter
    while ( !( ( wStatus = UART0->SR ) & ( UART_TimeOutIdle | UART_RxHalfFull | UART_RxBufFull ) ) ); // while the UART_RxFIFO is empty and no Timeoutidle
    value = ( unsigned char )UART0->RxBUFR; // then read the Receive Buffer Register
#endif // _STR7x_
#if ( _STR9x_ )
    //while( UART_GetFlagStatus(UART0, UART_FLAG_RxFIFOFull) != SET );
    while ( ( UART0->FR & 0x40 ) == RESET );

    //value = UART_ReceiveData(UART0);
    value = ( ( unsigned char )UART0->DR );
#endif // _STR9x_

    return ( int )( value );
}


int getchar()
{
    return __io_getchar();
}
