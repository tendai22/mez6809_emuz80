/*
 * bootload.c ... kuma's X format loader
 * September 17, 2023 (C) Norihiro Kumagai
 * 
 */

#include "param.h"
#include "iopin.h"
#include "xprintf.h"

#include "machdep.h"
#include "bootload.h"

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
    //xprintf("'%02x'", c);
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
    c &= ~0x20;     // capitalize
    return ('A' <= c && c <= 'F');
}

int to_hex(char c)
{
    //xprintf("{%c}", c);
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
    addr_t addr = 0, max = 0, min = RAM_TOP + RAM_SIZE - 1;
    int addr_flag = 0;
    
    while (1) {
        while ((c = getchr()) == ' ' || c == '\t' || c == '\n' || c == '\r')
            ;   // skip white spaces
        if (c == -1)
            break;
        if (c == '!' && min < max) {
            // dump memory
            addr_t start, end;
            start = min & 0xfff0;
            end = max;
            //if (end > 0x40)
            //    end = 0x40;
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
        if (c == 's') { // start with single_step
            ss_flag = 1;
            break;   // start processor
        }
        if (c == 'g') { // start with no-single_step
            ss_flag = 0;
            break;      // start prosessor
        }
        if (c == ',') { // clear ram
            clear_all();
            continue;
        }
        addr_flag = ((c == '=') || (c == '%'));
        cc = c;
        //xprintf("[%c]", c);
        if (!addr_flag)
            ungetchr(c);
        // read one hex value
        n = 0;
        while ((d = to_hex((unsigned char)(c = getchr()))) >= 0) {
            n *= 16; n += d;
            //xprintf("(%x,%x)", n, d);
        }
        //xprintf("=%04X ", n);
        if (c < 0)
            break;
        if (d < 0) {
            if (addr_flag) {  // set address
                if (cc == '=')
                    addr = (addr_t)n;
                else if (cc == '%')
                    break_address = (addr_t)n;
            } else {
                if (RAM_TOP <= addr && addr <= (RAM_TOP + RAM_SIZE - 1)) {
                    //xprintf("[%04X] = %02X%02X\n", addr, ((n>>8)&0xff), (n & 0xff));
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

//
// monitor
// monitor_mode: 1 ... DBG_PORT write
//               2 ... DBG_PORT read
//               0 ... other(usually single step mode)
//
void monitor(int monitor_mode)
{
    static int count = 0;
    static char buf[8];
    int c, d;
    unsigned long addr = get_addr();
    
    xprintf("|%05lX %02X %c ", addr, get_databus(), (get_rwflag() ? 'R' : 'W'));
    
    if (monitor_mode == 2) {    // DBG_PORT read
        xprintf(" IN>");
        xgets(buf, 7, 0);
        int i = 0, n = 0;
        while (i < 8 && (c = buf[i++]) && (d = to_hex((unsigned char)c)) >= 0) {
            n *= 16; n += d;
            //xprintf("(%x,%x)", n, d);
        }
        put_databus((unsigned char)n);
    } else {
        if (monitor_mode == 1) { // DBG_PORT write
            xprintf(" OUT: %02x", (int)get_databus());
        }
#if 0
        if ((c = getchr()) == '.')
            ss_flag = 0;
        else if (c == 's' || c == ' ')
            ss_flag = 1;
#endif
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



