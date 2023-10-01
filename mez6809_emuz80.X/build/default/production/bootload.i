# 1 "bootload.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 288 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "C:/Users/tenda/.mchp_packs/Microchip/PIC18F-Q_DFP/1.16.368/xc8\\pic\\include\\language_support.h" 1 3
# 2 "<built-in>" 2
# 1 "bootload.c" 2






# 1 "./param.h" 1
# 17 "./param.h"
typedef unsigned short addr_t;
# 7 "bootload.c" 2

# 1 "./iopin.h" 1
# 8 "bootload.c" 2

# 1 "./xprintf.h" 1






# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\string.h" 1 3



# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\musl_xc8.h" 1 3
# 4 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\string.h" 2 3






# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\features.h" 1 3
# 10 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\string.h" 2 3
# 25 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\string.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 1 3
# 122 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned size_t;
# 168 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef __int24 int24_t;
# 204 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef __uint24 uint24_t;
# 411 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef struct __locale_struct * locale_t;
# 25 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\string.h" 2 3


void *memcpy (void *restrict, const void *restrict, size_t);
void *memmove (void *, const void *, size_t);
void *memset (void *, int, size_t);
int memcmp (const void *, const void *, size_t);
void *memchr (const void *, int, size_t);

char *strcpy (char *restrict, const char *restrict);
char *strncpy (char *restrict, const char *restrict, size_t);

char *strcat (char *restrict, const char *restrict);
char *strncat (char *restrict, const char *restrict, size_t);

int strcmp (const char *, const char *);
int strncmp (const char *, const char *, size_t);

int strcoll (const char *, const char *);
size_t strxfrm (char *restrict, const char *restrict, size_t);

char *strchr (const char *, int);
char *strrchr (const char *, int);

size_t strcspn (const char *, const char *);
size_t strspn (const char *, const char *);
char *strpbrk (const char *, const char *);
char *strstr (const char *, const char *);
char *strtok (char *restrict, const char *restrict);

size_t strlen (const char *);

char *strerror (int);
# 65 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\string.h" 3
char *strtok_r (char *restrict, const char *restrict, char **restrict);
int strerror_r (int, char *, size_t);
char *stpcpy(char *restrict, const char *restrict);
char *stpncpy(char *restrict, const char *restrict, size_t);
size_t strnlen (const char *, size_t);
char *strdup (const char *);
char *strndup (const char *, size_t);
char *strsignal(int);
char *strerror_l (int, locale_t);
int strcoll_l (const char *, const char *, locale_t);
size_t strxfrm_l (char *restrict, const char *restrict, size_t, locale_t);




void *memccpy (void *restrict, const void *restrict, int, size_t);
# 7 "./xprintf.h" 2

# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdarg.h" 1 3







# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 1 3





typedef void * va_list[1];
# 8 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdarg.h" 2 3


#pragma intrinsic(__va_start)
#pragma intrinsic(__va_arg)

extern void * __va_start(void);
extern void * __va_arg(void *, ...);
# 8 "./xprintf.h" 2

# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdint.h" 1 3
# 22 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdint.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 1 3
# 127 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned long uintptr_t;
# 142 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long intptr_t;
# 158 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef signed char int8_t;




typedef short int16_t;
# 173 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long int32_t;





typedef long long int64_t;
# 188 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long long intmax_t;





typedef unsigned char uint8_t;




typedef unsigned short uint16_t;
# 209 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned long uint32_t;





typedef unsigned long long uint64_t;
# 229 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned long long uintmax_t;
# 22 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdint.h" 2 3


typedef int8_t int_fast8_t;

typedef int64_t int_fast64_t;


typedef int8_t int_least8_t;
typedef int16_t int_least16_t;

typedef int24_t int_least24_t;
typedef int24_t int_fast24_t;

typedef int32_t int_least32_t;

typedef int64_t int_least64_t;


typedef uint8_t uint_fast8_t;

typedef uint64_t uint_fast64_t;


typedef uint8_t uint_least8_t;
typedef uint16_t uint_least16_t;

typedef uint24_t uint_least24_t;
typedef uint24_t uint_fast24_t;

typedef uint32_t uint_least32_t;

typedef uint64_t uint_least64_t;
# 144 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdint.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/stdint.h" 1 3
typedef int16_t int_fast16_t;
typedef int32_t int_fast32_t;
typedef uint16_t uint_fast16_t;
typedef uint32_t uint_fast32_t;
# 144 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdint.h" 2 3
# 9 "./xprintf.h" 2
# 30 "./xprintf.h"
extern void (*xfunc_output)(int);
void xputc (int chr);
void xfputc (void (*func)(int), int chr);
void xputs (const char* str);
void xfputs (void (*func)(int), const char* str);
void xprintf (const char* fmt, ...);
void xsnprintf (char* buff, size_t len, const char* fmt, ...);
void xvfprintf (void (*func)(int), const char* fmt, va_list arp);
void put_dump (const void* buff, unsigned long addr, int len, size_t width);




extern int (*xfunc_input)(void);
int xgets (char* buff, int len, int noecho);
int xatoi (const char** str, int32_t* res);
int xatof (char** str, double* res);
# 9 "bootload.c" 2


# 1 "./machdep.h" 1
# 11 "./machdep.h"
extern void putchx(int c);
extern int getchx(void);
extern int kbhit(void);

extern char fetch_ram(addr_t addr);
extern void deposit_ram(addr_t addr, char c);
extern char peek_ram(addr_t addr);
extern void poke_ram(addr_t addr, char c);

extern void init_boot(void);
extern void end_boot(void);

extern unsigned char get_databus(void);
extern void put_databus(unsigned char c);
extern addr_t get_addr(void);
extern int get_rwflag(void);

extern int ss_flag;
extern addr_t break_address;
# 11 "bootload.c" 2

# 1 "./bootload.h" 1
# 14 "./bootload.h"
void manualboot(void);
void monitor(int monitor_mode);
# 12 "bootload.c" 2


static int uc = -1;

int getchr(void)
{
    static int count = 0;
    int c;
    if (uc >= 0) {
        c = uc;
        uc = -1;
        return c;
    }
    while ((c = getchx()) == '.' && count++ < 2);
    if (c == '.') {
        count = 0;
        return -1;
    }
    count = 0;

    return c;
}

void ungetchr(int c)
{
    uc = c;
}

int is_hex(char c)
{
    if ('0' <= c && c <= '9')
        return !0;
    c &= ~0x20;
    return ('A' <= c && c <= 'F');
}

int to_hex(char c)
{

    if ('0' <= c && c <= '9')
        return c - '0';
    c &= ~0x20;
    if ('A' <= c && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

void clear_all(void)
{
    addr_t p = 0;
    int i = 0;
    do {
        if ((p & 0xfff) == 0) {
            xprintf("%X", i++);
        }
        poke_ram(p, 0);
    } while (p++ != 0xffff);
}

void manualboot(void)
{
    int c, cc, d, n, count;
    addr_t addr = 0, max = 0, min = ((addr_t)0xF000) + ((addr_t)0x1000) - 1;
    int addr_flag = 0;

    while (1) {
        while ((c = getchr()) == ' ' || c == '\t' || c == '\n' || c == '\r')
            ;
        if (c == -1)
            break;
        if (c == '!' && min < max) {

            addr_t start, end;
            start = min & 0xfff0;
            end = max;


            while (start < end) {
                if ((start & 0xf) == 0) {
                    xprintf("%04X ", start);
                }
                d = ((unsigned short)peek_ram(start))<<8;
                d |= peek_ram(start + 1);
                xprintf("%04X ", d);
                if ((start & 0xf) == 0xe) {
                    xprintf("\n");
                }
                start += 2;
            }
            if (ss_flag)
                xprintf("ss ");
            if (break_address)
                xprintf("%%%04X ", break_address);
            continue;
        }
        if (c == 's') {
            ss_flag = 1;
            break;
        }
        if (c == 'g') {
            ss_flag = 0;
            break;
        }
        if (c == ',') {
            clear_all();
            continue;
        }
        addr_flag = ((c == '=') || (c == '%'));
        cc = c;

        if (!addr_flag)
            ungetchr(c);

        n = 0;
        while ((d = to_hex((unsigned char)(c = getchr()))) >= 0) {
            n *= 16; n += d;

        }

        if (c < 0)
            break;
        if (d < 0) {
            if (addr_flag) {
                if (cc == '=')
                    addr = (addr_t)n;
                else if (cc == '%')
                    break_address = (addr_t)n;
            } else {
                if (((addr_t)0xF000) <= addr && addr <= (((addr_t)0xF000) + ((addr_t)0x1000) - 1)) {

                    poke_ram(addr++, ((n>>8) & 0xff));
                    poke_ram(addr++, (n & 0xff));
                    if (max < addr)
                        max = addr;
                    if (addr - 2 < min)
                        min = addr - 2;
                }
            }
            continue;
        }
    }
}







void monitor(int monitor_mode)
{
    static int count = 0;
    static char buf[8];
    int c, d;
    unsigned long addr = get_addr();

    xprintf("|%05lX %02X %c ", addr, get_databus(), (get_rwflag() ? 'R' : 'W'));

    if (monitor_mode == 2) {
        xprintf(" IN>");
        xgets(buf, 7, 0);
        int i = 0, n = 0;
        while (i < 8 && (c = buf[i++]) && (d = to_hex((unsigned char)c)) >= 0) {
            n *= 16; n += d;

        }
        put_databus((unsigned char)n);
    } else {
        if (monitor_mode == 1) {
            xprintf(" OUT: %02x", (int)get_databus());
        }






        xprintf("\n");
    }
}

void march_test(void)
{
    addr_t p, last = 0xffff;
    int n = 0;
    p = 0;
    do {
        poke_ram(p, 0);
    } while (p++ != last);
    n++;
    p = 0;
    do {
        if (peek_ram(p) == 0)
            poke_ram(p, 1);
        else {
            xprintf("%d: fail at %04X\n", n, p);
            return;
        }
    } while (p++ != last);
    n++;
    p = last;
    do {
        if (peek_ram(p) == 0)
            poke_ram(p, 1);
        else {
            xprintf("%d: fail at %04X\n", n, p);
            return;
        }
    } while (p-- != 0);
    n++;
    p = last;
    do {
        if (peek_ram(p) == 1)
            poke_ram(p, 0);
        else {
            goto error;
        }
    } while (p-- != 0);
    xprintf("all ok\n");
    return;
error:
    xprintf("%d: fail at %04X\n", n, p);
    return;
}
