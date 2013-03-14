#include <sys/stat.h>
#include <errno.h>
#undef errno

extern int errno;

void _exit(int status)
{
    while(1)
    {}
}

void* _sbrk(int incr) {
/*
    extern char _end;		// Defined by the linker
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0) {
      heap_end = &_end;
    }
    prev_heap_end = heap_end;
    if (heap_end + incr > stack_ptr) {
      write (1, "Heap and stack collision\n", 25);
      abort ();
    }

    heap_end += incr;
    return (caddr_t) prev_heap_end;
    */
    return 0;
}

int _kill(int pid, int sig) {
    errno = EINVAL;
    return -1;
}

int _getpid(void) {
    return 1;
}

int _write(int file, char *ptr, int len) {
    /*
    int todo;

    for (todo = 0; todo < len; todo++) {
        outbyte (*ptr++);
    }
    */
    return len;
}

int _close(int file) {
    return -1;
}

int _fstat(int file, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file) {
    return 1;
}

int _read(int file, char *ptr, int len) {
    return 0;
}

int _lseek(int file, int ptr, int dir) {
    return 0;
}
