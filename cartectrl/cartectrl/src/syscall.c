/**
  ******************************************************************************
  * @file    syscall.c
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version V1.0.0
  * @date    2013-04-05
  * @brief   Provides implementation of internal functions required to
  *          libc/libstdc++.
  ******************************************************************************
  */

#include <sys/stat.h>
#include <errno.h>
#undef errno

/** @addtogroup Internals
  * @{
  */


extern int errno;   ///< The global errno.

/**
  * @brief  Function called when @c exit is called
  * @param  status status code
  * @return None
  * @note   The status code is unused.
  */
void _exit(int status)
{
    // Infinite loop
    for (;;);
}

/**
  * @brief  Implementation of @c sbrk, required in order to have a working
  *         implementation of <tt>malloc/free (new/delete)</tt>.
  * @param  incr Minimum space to allocate in heap
  * @return Pointer to newly allocated space
  * @note   See @c sbrk in Linux man pages for more details.
  */
void *_sbrk(int incr)
{

    // Defined by the linker
    extern char __heap_start__;
    extern char __heap_end__;

    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0)
    {
        // Initialisation
        heap_end = &__heap_start__;
    }
    prev_heap_end = heap_end;
    if (heap_end + incr > &__heap_end__)
    {
        // Indicate that no memory is available.
        errno = ENOMEM;
        return (void *) -1;
    }

    heap_end += incr;
    return (void *) prev_heap_end;
}

/**
  * @brief  Implementation of kill.
  * @param  pid process id to send a signal
  * @param  sig process to send a signal
  * @return 0 if successful, -1 otherwise, and errno is set according to error
  * @note   It is a dummy implementation: it does nothing and always return a
  *         error.
  */
int _kill(int pid, int sig)
{
    errno = EINVAL;
    return -1;
}

/**
  * @brief  Implementation of getpid.
  * @return Process id of the current running process
  * @note   It is a dummy implementation: it assumes there's only one process
  *         running and its id is 1.
  */
int _getpid(void)
{
    return 1;
}

/**
  * @brief  Implementation of write.
  * @param  file file number
  * @param  ptr Pointer to data to write
  * @param  len Length of data in ptr to write
  * @note   Reminder:
  *         @arg @c STDOUT_FILENO is stdout;
  *         @arg @c STDERR_FILENO is stderr.
  *
  *         These macros are defined in <tt>unistd.h</tt>.
  * @return Number of bytes successfully written or (ssize_t) -1 on error
  * @note   See @c write in Linux man pages for more details.
  * @todo   Make write work on stdout and stderr, so using cout/cerr/printf etc.
  *         actually do the work.
  */
ssize_t _write(int file, const char *ptr, size_t len)
{
    /*
    int todo;

    for (todo = 0; todo < len; todo++) {
        outbyte (*ptr++);
    }
    */
    return len;
}

/**
  * @brief  Implementation of close.
  * @param  file file number
  * @return 0 if successful, -1 otherwise, and errno is set according to error
  * @note   It is a dummy implementation: it assumes it can't close any file.
  */
int _close(int file)
{
    errno = EBADF;
    return -1;
}

/**
  * @brief  Implementation of close.
  * @param  file file number
  * @return 0 if successful, -1 otherwise, and errno is set according to error
  * @note   It is a dummy implementation: it pretends the argument passed is a
  *         file descriptor that refer to a <em>character device</em>.
  */
int _fstat(int file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

/**
  * @brief  Implementation of isatty.
  * @param  file file number
  * @return 1 if the file descriptor represents a tty.
  * @note   It is a dummy implementation: it pretends the argument passed is a
  *         file descriptor that always refer to a tty.
  */
int _isatty(int file)
{
    return 1;
}

/**
  * @brief  Implementation of read.
  * @param  file file number
  * @param  ptr Pointer to data to write into
  * @param  len Length of data to read
  * @note   Reminder:
  *         @arg @c STDIN_FILENO is stdin.
  *
  *         These macros are defined in <tt>unistd.h</tt>.
  * @return Number of bytes successfully read or (ssize_t) -1 on error
  * @note   See @c read in Linux man pages for more details.
  * @todo   Make write work on stdin, so using cin/scanf etc. actually do the
  *         work.
  */
ssize_t _read(int file, char *ptr, size_t len)
{
    return 0;
}

/**
  * @brief  Implementation of lseek.
  * @param  file file number
  * @note   Reminder:
  *         @arg @c STDIN_FILENO is stdin.
  *         These macros are defined in <tt>unistd.h</tt>.
  * @param  offset offset relative to @c whence
  * @param  whence from where in the file:
  *         @arg SEEK_SET the offset is set to offset bytes;
  *         @arg SEEK_CUR the offset is set to its current location plus offset
  *              bytes;
  *         @arg SEEK_END the offset is set to the size of the file plus offset
  *             bytes.
  * @return the resulting offset location as measured in bytes from the
  *         beginning of the file or (off_t) -1 on error, and errno is set
  *         accordingly
  * @note   See @c lseek in Linux man pages for more details.
  * @note   It is a dummy implementation: it assume the file is non seekable.
  */
off_t _lseek(int fd, off_t offset, int whence)
{
    errno = EBADF;
    return (off_t) -1;
}


/**
  * @}
  */
