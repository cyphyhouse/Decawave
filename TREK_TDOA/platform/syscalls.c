/*! ----------------------------------------------------------------------------
 *  @file    syscalls.c
 *  @brief   Standard syscalls platform-specific implementation
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "port_deca.h"

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _sbrk()
 *
 * @brief  Increment heap pointer to allow dynamic memory allocation.
 *
 * @param  incr  size of the heap increment to perform, in bytes
 *
 * @return  pointer on the previous heap end or -1 if memory is full and increment cannot be performed.
 */
caddr_t _sbrk(int incr)
{
    extern char _ebss; /* Defined by the linker. */
    static char *heap_end;
    char *prev_heap_end;
    char *stack;

    if (heap_end == 0)
    {
        heap_end = &_ebss;
    }

    prev_heap_end = heap_end;

    stack = (char*) port_GET_stack_pointer();

    if (heap_end + incr > stack)
    {
        return (caddr_t) -1;
    }

    heap_end += incr;

    return (caddr_t) prev_heap_end;
}

int _read_r(int fd, char *ptr, size_t len)
{
    int i;
    size_t counter = 0;

    if(fd != STDIN_FILENO)
    {
        return -1;
    }

    for (i = 0; i < len; i++)
    {
        // Get characters from the UART
        while (port_USARTx_no_data())
        { };
        *ptr++ = (char)port_USARTx_receive_data();
        counter++;
    }

    return counter;
}

int _write(int fd, char *ptr, size_t len)
{
    size_t counter = len;

    if ((fd != STDOUT_FILENO) && (fd != STDERR_FILENO))
    {
        return -1;
    }

    while(counter-- > 0)
    {
        // Send the character from the buffer to UART
        while (port_USARTx_busy_sending())
        { };
        port_USARTx_send_data(*ptr);
        ptr++;
    }

    return len;
}

void _exit(int status)
{
    _write (STDERR_FILENO, "exit", 4);
    // Loop forever
    while (1)
    { };
}

int _close_r(int file)
{
    return 0;
}

int _fstat_r(int file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty_r(int file)
{
    switch (file)
    {
        case STDOUT_FILENO:
        case STDERR_FILENO:
        case STDIN_FILENO:
            return 1;
        default:
            return 0;
    }
}

int _getpid_r(void)
{
    return 1; // Return any number as PID.
}

int _lseek_r(int file, int ptr, int dir)
{
    return 0;
}

int _kill_r(int pid, int sig)
{
    return 0;
}
