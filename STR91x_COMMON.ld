/*
Common part of the linker scripts for STR91x devices
Copyright RAISONANCE 2006
You can use, modify and distribute thisfile freely, but without any waranty.
*/


/* Registers mapping */


EXTMEM_BASE     = 0x30000000 ;

FMI_BASE        = 0x54000000 ;
APB0_BASE       = 0x58000000 ;
APB1_BASE       = 0x5C000000 ;

USB_BASE        = 0x70000000 ;
EMI_BASE        = 0x74000000 ;
DMA_BASE        = 0x78000000 ;

VIC0_BASE       = 0xFFFFF000 ;
VIC1_BASE       = 0xFC000000 ;


PROVIDE( _DMA      = (DMA_BASE) );
PROVIDE( _EMI      = (EMI_BASE) );

PROVIDE( _VIC0     = (VIC0_BASE) );
PROVIDE( VIC0VECT  = (VIC0_BASE + 0x0030) );
PROVIDE( _VIC1     = (VIC1_BASE) );
PROVIDE( VIC1VECT  = (VIC1_BASE + 0x0030) );

PROVIDE( _USB      = (USB_BASE) );

PROVIDE( _AHBAPB0  = (APB0_BASE) );
PROVIDE( _WIU      = (APB0_BASE + 0x1000) );
PROVIDE( _TIM0     = (APB0_BASE + 0x2000) );
PROVIDE( _TIM1     = (APB0_BASE + 0x3000) );
PROVIDE( _TIM2     = (APB0_BASE + 0x4000) );
PROVIDE( _TIM3     = (APB0_BASE + 0x5000) );
PROVIDE( _GPIO0    = (APB0_BASE + 0x6000) );
PROVIDE( _GPIO1    = (APB0_BASE + 0x7000) );
PROVIDE( _GPIO2    = (APB0_BASE + 0x8000) );
PROVIDE( _GPIO3    = (APB0_BASE + 0x9000) );
PROVIDE( _GPIO4    = (APB0_BASE + 0xA000) );
PROVIDE( _GPIO5    = (APB0_BASE + 0xB000) );
PROVIDE( _GPIO6    = (APB0_BASE + 0xC000) );
PROVIDE( _GPIO7    = (APB0_BASE + 0xD000) );
PROVIDE( _GPIO8    = (APB0_BASE + 0xE000) );
PROVIDE( _GPIO9    = (APB0_BASE + 0xF000) );

PROVIDE( _AHBAPB1  = (APB1_BASE) );
PROVIDE( _RTC      = (APB1_BASE + 0x1000) );
PROVIDE( _SCU      = (APB1_BASE + 0x2000) );
PROVIDE( _MC       = (APB1_BASE + 0x3000) );
PROVIDE( _UART0    = (APB1_BASE + 0x4000) );
PROVIDE( _UART1    = (APB1_BASE + 0x5000) );
PROVIDE( _UART2    = (APB1_BASE + 0x6000) );
PROVIDE( _SPI0     = (APB1_BASE + 0x7000) );
PROVIDE( _SPI1     = (APB1_BASE + 0x8000) );
PROVIDE( _CAN      = (APB1_BASE + 0x9000) );
PROVIDE( _ADC      = (APB1_BASE + 0xA000) );
PROVIDE( _WDG      = (APB1_BASE + 0xB000) );
PROVIDE( _I2C0     = (APB1_BASE + 0xC000) );
PROVIDE( _I2C1     = (APB1_BASE + 0xD000) );




/* default stack sizes. 

These are used by the startup in order to allocate stacks for the different modes.
*/

__SVC_Stack_Size = 256 ;
__IRQ_Stack_Size = 1024 ;
__USR_Stack_Size = 1024 ;
__FIQ_Stack_Size = 256 ;
__ABT_Stack_Size = 256 ;
__UND_Stack_Size = 256 ;

__SVC_Stack_Init = _estack ;
__IRQ_Stack_Init = __SVC_Stack_Init - __SVC_Stack_Size ;
__USR_Stack_Init = __IRQ_Stack_Init - __IRQ_Stack_Size ;
__FIQ_Stack_Init = __USR_Stack_Init - __USR_Stack_Size ;
__ABT_Stack_Init = __FIQ_Stack_Init - __FIQ_Stack_Size ;
__UND_Stack_Init = __ABT_Stack_Init - __ABT_Stack_Size ;

/*"PROVIDE" allows to easily override these values from an object file or the commmand line.*/
PROVIDE ( _SVC_Stack_Init = __SVC_Stack_Init ) ;
PROVIDE ( _IRQ_Stack_Init = __IRQ_Stack_Init ) ;
PROVIDE ( _USR_Stack_Init = __USR_Stack_Init ) ;
PROVIDE ( _FIQ_Stack_Init = __FIQ_Stack_Init ) ;
PROVIDE ( _ABT_Stack_Init = __ABT_Stack_Init ) ;
PROVIDE ( _UND_Stack_Init = __UND_Stack_Init ) ;

/*
the user mode stack is an exception because we want it at the end of the RAM.
therefore, we just check against a minimum.
The value below is the minimum memory required for ALL THE STACKS.
There will be a link error if there is not this amount of RAM free at the end.
*/
_Minimum_Stack_Size = 0xC00 ;



/* default ISR addresses. 

The startup needs these addresses defined from another object file.
In case they are not, these PROVIDEs redirect them to the Reset.

_start must be defined, usually in the startup.
*/

PROVIDE( Undefined_Handler = _start );
PROVIDE( SWI_Handler = _start );
PROVIDE( Prefetch_Handler = _start );
PROVIDE( Abort_Handler = _start );
PROVIDE( FIQ_Handler = _start );
PROVIDE( T0TIMI_IRQHandler = _start );
PROVIDE( FLASH_IRQHandler = _start );
PROVIDE( RCCU_IRQHandler = _start );
PROVIDE( RTC_IRQHandler = _start );
PROVIDE( WDG_IRQHandler = _start );
PROVIDE( XTI_IRQHandler = _start );
PROVIDE( USBHP_IRQHandler = _start );
PROVIDE( I2C0ITERR_IRQHandler = _start );
PROVIDE( I2C1ITERR_IRQHandler = _start );
PROVIDE( UART0_IRQHandler = _start );
PROVIDE( UART1_IRQHandler = _start );
PROVIDE( UART2_IRQHandler = _start );
PROVIDE( UART3_IRQHandler = _start );
PROVIDE( BSPI0_IRQHandler = _start );
PROVIDE( BSPI1_IRQHandler = _start );
PROVIDE( I2C0_IRQHandler = _start );
PROVIDE( I2C1_IRQHandler = _start );
PROVIDE( CAN_IRQHandler = _start );
PROVIDE( ADC12_IRQHandler = _start );
PROVIDE( T1TIMI_IRQHandler = _start );
PROVIDE( T2TIMI_IRQHandler = _start );
PROVIDE( T3TIMI_IRQHandler = _start );
PROVIDE( HDLC_IRQHandler = _start );
PROVIDE( USBLP_IRQHandler = _start );
PROVIDE( T0TOI_IRQHandler = _start );
PROVIDE( T0OC1_IRQHandler = _start );
PROVIDE( T0OC2_IRQHandler = _start );



