#include <stddef.h>
int _write (int file, const void * ptr, size_t len)
{
    int i;
    char ch;
    // for(i=0;i<len;++i)
    // {
    //     OLED_ScreenSwitchCheck();
    //     ch = *(char*)ptr;
    //     if(ch == '\n')
    //         OLED_Newline();
    //     else
    //     {
    //         OLED_Update_Pos();
    //         OLED_ShowChar(_ssd1306_pos_x,_ssd1306_pos_y,*((char*)ptr++));
    //     }
    // }

    // for(i =0;i<len;++i)
    // {
    //     HAL_UART_Transmit(&huart1, &(((uint8_t*)ptr)[i]), 1, 0);
    //     delay_ms(1);
    // }

    return len;
}

// _ssize_t
// _write_r (struct _reent *ptr,
//      int fd,
//      const void *buf,
//      size_t cnt);

int _read (int __fd, void *__buf, size_t __nbyte){return 0;}

// _ssize_t
// _read_r (struct _reent *ptr,
//      int fd,
//      void *buf,
//      size_t cnt){return 0;}

int     _close (int fd){return 0;}

// int _close_r(struct _reent *ptr,
//      int fd){return 0;}

int	_fstat (int __fd, struct stat *__sbuf ){return 0;}

// int
// _fstat_r (struct _reent *ptr,int fd,struct stat * pstat){return 0;}

extern const unsigned int _Min_Heap_Size;
extern const unsigned int  _end;

ptrdiff_t prog_brk_off = 0;

void *  _sbrk (ptrdiff_t __incr){
    void* old = (void*)((unsigned int )&_end + prog_brk_off);
    if(prog_brk_off + __incr > (unsigned int )&_Min_Heap_Size)
        return (void*)-1;
    prog_brk_off += __incr;
    return old;
}

// void *
// _sbrk_r (struct _reent *ptr,
//      ptrdiff_t incr){return NULL;}

int   _lseek (int __fildes, int __offset, int __whence){return 0;}

// _off_t
// _lseek_r (struct _reent *ptr,
//      int fd,
//      _off_t pos,
//      int whence){return 0;}

int
_kill (int pid,
     int sig){return 0;}

// int
// _kill_r (struct _reent *ptr,
//      int pid,
//      int sig){return 0;}

int
_getpid (void) {return 0;}

// int
// _getpid_r (struct _reent *ptr){return 0;};

// int
// _isatty_r (struct _reent *ptr,int fd){return 0;}

int	_isatty (int fd){return 0;}

void	_exit		(int sig){}


