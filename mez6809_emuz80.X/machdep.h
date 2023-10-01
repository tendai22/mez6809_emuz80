/*
 * machdep.h : extern functions provided in main.c
 * September 17, 2023 (C) Norihiro Kumagai
 */

#ifndef __MACHDEP_H
#define __MACHDEP_H
#ifdef	__cplusplus
extern "C" {
#endif
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

#ifdef	__cplusplus
}
#endif

#endif //__MACHDEP_H
