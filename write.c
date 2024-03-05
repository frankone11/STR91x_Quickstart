/* Support files for GNU libc.  Files in the system namespace go here.
   Files in the C namespace (ie those that do not start with an
   underscore) go in .c.  */


#include <unistd.h>


//__attribute__ ((weak)) int     _write      _PARAMS( ( int, char*, int ) );

extern void __io_putchar( int c );

__attribute__ ((weak)) int _write( int    file,
            char* ptr,
            int    len )
{
    int todo;

    for ( todo = 0; todo < len; todo++ )
    {
        __io_putchar( *ptr++ );
    }

    return len;
}
