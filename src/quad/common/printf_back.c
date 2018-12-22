
#include "printf.h"
#include "typeconversion.h"

#define REQUIRE_CC_ARM_PRINTF_SUPPORT
#define REQUIRE_PRINTF_LONG_SUPPORT

static serialPort_t *printfSerialPort;

typedef void (*putcf) (void *, char);
static putcf stdout_putf;
static void *stdout_putp;

void init_printf(void *putp, void (*putf) (void *, char))
{
    stdout_putf = putf;
    stdout_putp = putp;
}

static void _putc(void *p, char c)
{
    UNUSED(p);
    serialWrite(printfSerialPort, c);
}

void printfSupportInit(void)
{
	init_printf(NULL, _putc);
}

// print bf, padded from left to at least n characters.
// padding is zero ('0') if z!=0, space (' ') otherwise
static int putchw(void *putp, putcf putf, int n, char z, char *bf)
{
    int written = 0;
    char fc = z ? '0' : ' ';
    char ch;
    char *p = bf;
    while (*p++ && n > 0)
        n--;
    while (n-- > 0) {
        putf(putp, fc); written++;
    }
    while ((ch = *bf++)) {
        putf(putp, ch); written++;
    }
    return written;
}

// return number of bytes written
int tfp_format(void *putp, putcf putf, const char *fmt, va_list va)
{
    char bf[12];
    int written = 0;
    char ch;

    while ((ch = *(fmt++))) {
        if (ch != '%') {
            putf(putp, ch); written++;
        } else {
            char lz = 0;
#ifdef  REQUIRE_PRINTF_LONG_SUPPORT
            char lng = 0;
#endif
            int w = 0;
            ch = *(fmt++);
            if (ch == '0') {
                ch = *(fmt++);
                lz = 1;
            }
            if (ch >= '0' && ch <= '9') {
                ch = a2i(ch, &fmt, 10, &w);
            }
#ifdef  REQUIRE_PRINTF_LONG_SUPPORT
            if (ch == 'l') {
                ch = *(fmt++);
                lng = 1;
            }
#endif
            switch (ch) {
            case 0:
                goto abort;
            case 'u':{
#ifdef  REQUIRE_PRINTF_LONG_SUPPORT
                    if (lng)
                        uli2a(va_arg(va, unsigned long int), 10, 0, bf);
                    else
#endif
                        ui2a(va_arg(va, unsigned int), 10, 0, bf);
                    written += putchw(putp, putf, w, lz, bf);
                    break;
                }
            case 'd':{
#ifdef  REQUIRE_PRINTF_LONG_SUPPORT
                    if (lng)
                        li2a(va_arg(va, unsigned long int), bf);
                    else
#endif
                        i2a(va_arg(va, int), bf);
                    written += putchw(putp, putf, w, lz, bf);
                    break;
                }
            case 'x':
            case 'X':
#ifdef  REQUIRE_PRINTF_LONG_SUPPORT
                if (lng)
                    uli2a(va_arg(va, unsigned long int), 16, (ch == 'X'), bf);
                else
#endif
                    ui2a(va_arg(va, unsigned int), 16, (ch == 'X'), bf);
                written += putchw(putp, putf, w, lz, bf);
                break;
            case 'c':
                putf(putp, (char) (va_arg(va, int))); written++;
                break;
            case 's':
                written += putchw(putp, putf, w, 0, va_arg(va, char *));
                break;
            case '%':
                putf(putp, ch); written++;
                break;
            case 'n':
                *va_arg(va, int*) = written;
                break;
            default:
                break;
            }
        }
    }
abort:
    return written;
}

int tfp_printf(const char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    int written = tfp_format(stdout_putp, stdout_putf, fmt, va);
    va_end(va);
    while (!isSerialTransmitBufferEmpty(printfSerialPort));
    return written;
}

static void putcp(void *p, char c)
{
    *(*((char **) p))++ = c;
}

int tfp_sprintf(char *s, const char *fmt, ...)
{
    va_list va;

    va_start(va, fmt);
    int written = tfp_format(&s, putcp, fmt, va);
    putcp(&s, 0);
    va_end(va);
    return written;
}

void setPrintfSerialPort(serialPort_t *serialPort)
{
	printfSerialPort = serialPort;
}
