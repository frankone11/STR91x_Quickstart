/* Support files for GNU libc.  Files in the system namespace go here.
   Files in the C namespace (ie those that do not start with an
   underscore) go in .c.  */

#include <unistd.h>

//__attribute__ ((weak)) caddr_t _sbrk       _PARAMS( ( int ) );


/* Register name faking - works in collusion with the linker.  */
register char* stack_ptr asm( "sp" );

__attribute__ ((weak)) 
caddr_t _sbrk( int incr )
{
    extern char end;       /* Defined by the linker */
    static char* heap_end;
    char* prev_heap_end;

    if ( heap_end == 0 )
    {
        heap_end = &end;
    }
    prev_heap_end = heap_end;
    if ( heap_end + incr > stack_ptr )
    {
        //stack and heap collision detected!!!
#if 1  //malloc should be usable without putchar
        //enter infinite loop
        while ( 1 );
        //you can replace this by a software reset, custom abort function, etc.
#else  //malloc should be usable without putchar
        _write( 1, "H&S col\n", 8 );
        abort();
#endif //malloc should be usable without putchar
    }

    heap_end += incr;
    return ( caddr_t ) prev_heap_end;
}

