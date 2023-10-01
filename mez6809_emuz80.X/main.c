/*
  PIC18F57Q43 ROM RAM and UART emulation firmware
  This single source file contains all code

  Target: mez6809_emuz80 - Mezzanine 6809 board on EMUZ80 board
  Compiler: MPLAB XC8 v2.36
  Copyright (C) by Norihiro Kumagai, 2023
  Original Written by Tetsuya Suzuki
  Special thanks https://github.com/satoshiokue/
*/

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator Selection (Oscillator not enabled)
#pragma config RSTOSC = HFINTOSC_64MHZ// Reset Oscillator Selection (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)

// CONFIG2
#pragma config CLKOUTEN = OFF   // Clock out Enable bit (CLKOUT function is disabled)
#pragma config PR1WAY = ON      // PRLOCKED One-Way Set Enable bit (PRLOCKED bit can be cleared and set only once)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

// CONFIG3
#pragma config MCLRE = EXTMCLR  // MCLR Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#pragma config PWRTS = PWRT_OFF // Power-up timer selection bits (PWRT is disabled)
#pragma config MVECEN = ON      // Multi-vector enable bit (Multi-vector enabled, Vector table used for interrupts)
#pragma config IVT1WAY = ON     // IVTLOCK bit One-way set enable bit (IVTLOCKED bit can be cleared and set only once)
#pragma config LPBOREN = OFF    // Low Power BOR Enable bit (Low-Power BOR disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

// CONFIG4
#pragma config BORV = VBOR_1P9  // Brown-out Reset Voltage Selection bits (Brown-out Reset Voltage (VBOR) set to 1.9V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD module is disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = OFF    // PPSLOCK bit One-Way Set Enable bit (PPSLOCKED bit can be set and cleared repeatedly (subject to the unlock sequence))
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG5
#pragma config WDTCPS = WDTCPS_31// WDT Period selection bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled; SWDTEN is ignored)

// CONFIG6
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG7
#pragma config BBSIZE = BBSIZE_512// Boot Block Size selection bits (Boot Block size is 512 words)
#pragma config BBEN = OFF       // Boot Block enable bit (Boot block disabled)
#pragma config SAFEN = OFF      // Storage Area Flash enable bit (SAF disabled)
#pragma config DEBUG = OFF      // Background Debugger (Background Debugger disabled)

// CONFIG8
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block not Write protected)
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers not Write protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not Write protected)
#pragma config WRTSAF = OFF     // SAF Write protection bit (SAF not Write Protected)
#pragma config WRTAPP = OFF     // Application Block write protection bit (Application Block not write protected)

// CONFIG10
#pragma config CP = OFF         // PFM and Data EEPROM Code Protection bit (PFM and Data EEPROM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include "param.h"
#include "iopin.h"
#include "xprintf.h"
#include "machdep.h"
#include "bootload.h"

#define MEZ6809_CLK 8000000UL // Z80 clock frequency(Max 20MHz)

#define _XTAL_FREQ 64000000UL

#define TOGGLE do { LATE2 ^= 1; } while(0)

union {
    unsigned int w; // 16 bits Address
    struct {
        unsigned char l; // Address low 8 bits
        unsigned char h; // Address high 8 bits
    };
} ab;

#define nop asm("  nop")

addr_t break_address = 0; // break_address is zero, it is invalid
int ss_flag = 0;

#define db_setin() (TRISC = 0xff)
#define db_setout() (TRISC = 0x00)

static void reset_DFF(void);



// UART3 Transmit
void putchx(int c) {
    while(!U3TXBE); // Wait for Tx buffer empty
    U3TXB = (unsigned char)c; // Write data
}

// UART3 Recive
int getchx(void) {
    while(U3RXBE); // Wait while Rx buffer is empty
    return U3RXB; // Read data
}

int kbhit(void)
{
    return !U3RXBE;
}

void HALT_on(void)
{
    LATE0 = 0;
}

void HALT_off(void)
{
    LATE0 = 1;
}

void RES_on(void)
{
    LATE1 = 0;
}

void RES_off(void)
{
    LATE1 = 1;
}

unsigned char get_databus(void)
{
    return PORTC;
}

addr_t get_addr(void)
{
    return ((((addr_t)PORTD)<<8) | PORTB);
}

int get_rwflag(void)
{
    return RD5;
}

void put_databus(unsigned char c)
{
    LATC = c;
}

static unsigned char ram[RAM_SIZE]; // Equivalent to RAM

char peek_ram(addr_t addr)
{
    return ram[addr & 0xfff];
}

void poke_ram(addr_t addr, char c)
{
    //xprintf("[%04X,%02X]", addr, c);
    ram[addr&0xfff] = c;
}

void init_boot(void)
{
    // RESET && HALT
}

void end_boot(void)
{
    
}

static void set_DFF(void)
{
    //TOGGLE;
    CLCSELECT = 0;      // CLC1 select
    CLCnGLS3 = 0x40;    // 1 for D-FF SET
    CLCnGLS3 = 0x80;    // 0 for D-FF SET
    //TOGGLE;
}

#define db_setin() (TRISC = 0xff)
#define db_setout() (TRISC = 0x00)

// toggle E gate
static void toggle_Egate(void)
{
    CLCSELECT = 2;
    CLCnCON &= ~0x80;   // DISABLE it
    //CLCnGLS2 = 0x04;    // Gate2 <- D2T <- E
    CLCnGLS2 ^= 0x0C;   // toggle D2T/N
    CLCnCON |= 0x80;
}




// main routine
void main(void) {
    int monitor_mode = 0;
    int count;
    unsigned char cc;
    unsigned long addr;

    // PIC/System initialize for A6,A7 being stable
    
    // RE2: TEST Pin output
    ANSELE2 = 0;
    TRISE2 = 0;
    LATE2 = 0;
    TOGGLE; TOGGLE;
    
    // System initialize
    OSCFRQ = 0x08; // 64MHz internal OSC

    // RE1: RESET output pin
    ANSELE1 = 0; // Disable analog function
    LATE1 = 0; // RESET assert
    TRISE1 = 0; // Set as output
    
    // RA0: R/W input pin with Weak pull up
    ANSELA0 = 0;
    WPUA0 = 1;
    TRISA0 = 1;
    
    
    // RA3: CLK out, hand clock
    ANSELA3 = 0;
    LATA3 = 0;
    TRISA3 = 0; // set as output
    RA3PPS = 0; // need it for handmade clock starts
    // handmade clock, settle HD6809 down in RESET&&HALT
    for (int i = 0; i < 38; ++i) {
        LATA3 ^= 1; // toggle it
    }

    // CPU test
    // DB out 0x12
    ANSELC = 0;
    TRISC = 0;      // output
    LATC = 0x12;    // NOP instruction
    
    LATE1 = 1;
    for (int i = 0; i < 256; ++i) {
        LATA3 ^= 1; // toggle it
    }
    


    __delay_ms(5);      // 5ms wait, expecting 
    TOGGLE; TOGGLE;

    // initialize for monitor/downloader
    
    // xprintf initialize
    xdev_out(putchx);
    xdev_in(getchx);


    // CLC disable
    CLCSELECT = 0;
    CLCnCON &= ~0x80;
    CLCSELECT = 1;
    CLCnCON &= ~0x80;
    CLCSELECT = 2;
    CLCnCON &= ~0x80;

    // Address bus A15-A8 pin
    ANSELD = 0x00; // Disable analog function
    WPUD = 0xff; // Week pull up
    TRISD |= 0xff; // Set as input

    // Address bus A7-A0 pin
    ANSELB = 0x00; // Disable analog function
    WPUB = 0xff; // Week pull up
    TRISB = 0xff; // Set as input

    // Data bus D7-D0 pin
    ANSELC = 0x00; // Disable analog function
    WPUC = 0xff; // Week pull up
    TRISC = 0xff; // Set as input(default)

    // IO pin assignment
    // RA1: E input
    ANSELA1 = 0;
    TRISA1 = 1;

    // If E == H, one more EXTAL clock is added.
    while (RA1) {
        LATA3 ^= 1;
        LATA3 ^= 1;
   
    }
    TOGGLE; TOGGLE;
    
    // RA0: R/W input
    ANSELA0 = 0;
    TRISA0 = 1;
    
    // RA4: NMI output pin
    ANSELA4 = 0;
    LATA4 = 1;
    TRISA4 = 0; // set as output
    
    // RA2: Q input pin
    ANSELA2 = 0;
    TRISA2 = 1; // set as input

    // Z80 clock(RA3) by NCO FDC mode
    //RA3PPS = 0x3f; // RA3 assign NCO1
    ANSELA3 = 0; // Disable analog function
    TRISA3 = 0; // NCO output pin
    NCO1INC = MEZ6809_CLK * 2 / 61;
    NCO1INC = 0x04000;
    NCO1CLK = 0x00; // Clock source Fosc
    NCO1PFM = 0;  // FDC mode
    NCO1OUT = 1;  // NCO output enable
    NCO1EN = 1;   // NCO enable
#if defined(EXTERNAL_CLOCK)
    // If we use an external clock generator, we need to
    // make RA3 pin Hi-Z.
    RA3PPS = 0;     // internal clock disable
    TRISA3 = 1;
#endif

    // test LED blink
#if 0
    while (1) {
        TOGGLE;
        __delay_us(50);
    }
#endif
    // UART3 initialize
//    U3BRG = 416; // 9600bps @ 64MHz
    U3CON0 |= (1<<7);   // BRGS = 0, 4 baud clocks per bit
    U3BRG = 138;    // 115200bps @ 64MHz, BRG=0, 99%

    U3CON0 &= 0xf0; // clear U3MODE 0000 -> 8bit
    U3TXBE = U3RXBE = 0;    // clear tx/rx/buffer
    U3RXEN = 1; // Receiver enable
    U3TXEN = 1; // Transmitter enable

    // UART3 Receiver
    ANSELA7 = 0; // Disable analog function
    TRISA7 = 1; // RX set as input
    U3RXPPS = 0x07; //RA7->UART3:RX3;
    
    // UART3 Transmitter
    ANSELA6 = 0; // Disable analog function
    LATA6 = 1; // Default level
    TRISA6 = 0; // TX set as output
    RA6PPS = 0x26;  //RA6->UART3:TX3;
    U3ON = 1; // Serial port enable

    xprintf(";");
#if 0    
    // 1, 2, 5, 6: Port A, C
    // 3, 4, 7, 8: Port B, D
    RA4PPS = 0x0;  // LATA4 -> RA4 -> /OE
    RA2PPS = 0x0;  // LATA2 -> RA2 -> /WE
    RD6PPS = 0x0;  // LATD6 -> RD6 -> WAIT
#endif

#if 0
    // L-chika
    while (1) {
        LATE1 ^= 1;
        LATE0 ^= 1;
        for (int i = 0; i < 100; ++i) {
            __delay_ms(10);
        }
    }
#endif
    // reconfigurate CLC devices
    // CLC pin assign
    // 0, 1, 4, 5: Port A, C
    // 2, 3, 6, 7: Port B, D
    CLCIN0PPS = 0x00;   // RA0 <- /IORQ <- R/W
    CLCIN1PPS = 0x01;   // RA1 <- /MREQ <- E
    CLCIN2PPS = 0x1C;   // RD4 <- A12
    CLCIN3PPS = 0x1D;   // RD5 <- A13
    CLCIN6PPS = 0x1E;   // RD6 <- A14
    CLCIN7PPS = 0x1F;   // RD7 <- A15
    
    // ====== CLC1 Address decoder
    CLCSELECT = 0;      // CLC1 select
    CLCnCON &= ~0x80;
    
    CLCnSEL0 = 2;       // CLCIN2 <- RD4 <- A12
    CLCnSEL1 = 3;       // CLCIN3 <- RD5 <- A13
    CLCnSEL2 = 6;       // CLCIN6 <- RD6 <- A14
    CLCnSEL3 = 7;       // CLCIN7 <- RD7 <- A15
    
    CLCnGLS0 = 0x02;    // Gate1 <- D1T
    CLCnGLS1 = 0x08;    // Gate2 <- D2T
    CLCnGLS2 = 0x20;    // Gate3 <- D3T
    CLCnGLS3 = 0x80;    // Gate4 <- D4T
    
    CLCnPOL = 0x00;     // non inverted
    CLCnCON = 0x82;     // 4 input AND, ENABLE it
    
    // ====== CLC3 AND-OR
    CLCSELECT = 2;      // CLC3 select
    CLCnCON &= ~0x80;   // DISABLE it
    
    CLCnSEL0 = 0x33;    // CLC1 ... address decoder
    CLCnSEL1 = 1;       // CLCIN1 <- RA1 <- E or /E
    CLCnSEL2 = 127;     // NC
    CLCnSEL3 = 0x2a;    // NCO1CLK
    
//    CLCnGLS0 = 0x02;    // Gate1 <- D1T <- CLC1
//    CLCnGLS0 = 0x40;    // "1" (NC inverted)
    CLCnGLS0 = 0x20;    // "0" (NC inverted)
    CLCnGLS1 = 0x04;    // Gate2 <- D2T <- E
//    CLCnGLS1 = 0x04;    // Gate2 <- D2N <- /E
    CLCnGLS2 = 0x20;    // Gate3 <- D3T
//    CLCnGLS2 = 0x10;    // "1" (NC inverted) (If SELn is NC, DnT is 0, DnN is 1)
//    CLCnGLS2 = 0x04;    // Gate2 <- D2T <- E
//    CLCnGLS3 = 0x80;    // Gate4 <- D4T
    CLCnGLS3 = 0x80;    // Gate4 <- D4T
    
    CLCnPOL = 0x00;     // non inverted the CLC2 output
    CLCnCON = 0x80;     // AND-OR (no interrupt)
    
    // ====== CLC2  1 input D-FF
    CLCSELECT = 1;      // choose CLC2
    
    CLCnSEL0 = 0x35;    // D-FF CLK <- CLC3OUT
    CLCnSEL1 = 0x34;    // D-FF D <- /CLC2OUT
    CLCnSEL2 = 127;     // D-FF SET NC
    CLCnSEL3 = 127;     // D-FF RESET NC
    
    CLCnGLS0 = 0x02;    // D-FF CLK <- D1T
    CLCnGLS1 = 0x04;    // D-FF D   <- D2N <- /CLC2OUT (inverted)
    CLCnGLS2 = 0x20;    // 0 for D-FF RESET
    CLCnGLS3 = 0x20;    // 0 for D-FF SET
    
    CLCnPOL = 0x00;     // non inverted the CLC3 output
    CLCnCON = 0x84;     // Select D-FF (no interrupt)

#if 0
    // ============== CLC3 /WAIT
    CLCSELECT = 2;      // CLC3 select
    CLCnCON &= ~0x80;
    
    CLCnSEL0 = 1;       // D-FF CLK <- CLCIN1PPS <- /IORQ
    CLCnSEL1 = 127;     // D-FF D NC
    CLCnSEL2 = 127;     // D-FF SET NC
    CLCnSEL3 = 127;     // D-FF RESET NC
    
    CLCnGLS0 = 0x01;    // /IORQ ~|_  (inverted)
    CLCnGLS1 = 0x40;    // D-FF D NC (1 for D-FF D)
    CLCnGLS2 = 0x80;    // D-FF SET (soft reset)
    CLCnGLS3 = 0x00;    // 0 for D-FF RESET

    // reset D-FF
    CLCnGLS2 = 0x40;    // 1 for D-FF RESET
    CLCnGLS2 = 0x80;    // 0 for D-FF RESET

    CLCnPOL = 0x80;     // non inverted the CLC3 output
    CLCnCON = 0x84;     // Select D-FF (no interrupt)
        
    // Here, we have CLC's be intialized.
    // Now we change GPIO pins to be switched
    // 1, 2, 5, 6: Port A, C
    // 3, 4, 7, 8: Port B, D
    RD6PPS = 0x03;      // CLC3 -> RD6 -> /WAIT
#endif

    
    init_boot();
    manualboot();
    end_boot();
    xprintf("->\n");

    // Clock output connect
    RA3PPS = 0x01;      // RA3 <- CLC2OUT
    
    // RESET again
    LATE1 = 0;
    LATE0 = 0;  // /BUSRQ deassert
    db_setin();
    TOGGLE;
    TOGGLE;
#if 0
    // Re-initialze for CPU running
    // RA5: /RD, SRAM /OE pin
    ANSELA5 = 0;
    LATA5 = 1;  // /RD negate
    TRISA5 = 1; // set as input
    
    // RA2: /WR, SRAM /WE pin
    ANSELA2 = 0;
    LATA2 = 1;  // CS2 negate
    TRISA2 = 1; // set as input

    // RA1: MREQ input pin
    ANSELA1 = 0; // Disable analog function
    TRISA1 = 1; // Set as input
#endif

#if 0
    // Address bus A15-A8 pin
    ANSELD = 0x00; // Disable analog function
    WPUD = 0x3f; // Week pull up
    TRISD |= 0x3f; // Set as input

    // Address bus A7-A0 pin
    ANSELB = 0x00; // Disable analog function
    WPUB = 0xff; // Week pull up
    TRISB = 0xff; // Set as input

    // Data bus D7-D0 pin
    ANSELC = 0x00; // Disable analog function
    WPUC = 0xff; // Week pull up
    TRISC = 0xff; // Set as input(default)
#endif
    // 6809 start
    //CLCDATA = 0x7;
    db_setin();
    while(!RA1);     // wait for E == L
    toggle_Egate();
    while(RA1);
    toggle_Egate();
    while(!RA1);     // wait for E == L
    toggle_Egate();
    while(RA1);
    toggle_Egate();
    while(!RA1);     // wait for E == L
    toggle_Egate();
    while(RA1);
    toggle_Egate();
    while(!RA1);     // wait for E == L
    toggle_Egate();
    while(RA1);
    toggle_Egate();
    while(!RA1);     // wait for E == L
    toggle_Egate();
    while(RA1);
    toggle_Egate();
    while(!RA1);     // wait for E == L
    toggle_Egate();
    while(RA1);
    toggle_Egate();
    TOGGLE;
    TOGGLE;
    TOGGLE;
    TOGGLE;
    count = 10;
    ss_flag = 1;
    while(!RA1);     // wait for E == L
    toggle_Egate();
    while(RA1);
    toggle_Egate();
    RES_off();
    while(1){
        int rw_flag;
        while(!RA1);    // Wait for E == H
        rw_flag = RA0;  // R/W
        if(RA0) { // R/W == 1, 6809 Read cycle (RW = 1)
            TOGGLE;
            addr = get_addr();
            LATC = 0x12;    // nop
            db_setout();
            //__delay_us(10);
#if 0
            if (addr == UART_CREG){ // UART control register
                // PIR9 pin assign
                // U3TXIF ... bit 1, 0x02
                // U3RXIF ... bit 0, 0x01
                // U3FIFO bit assign
                // TXBE   ... bit 5, 0x20
                // TXBF   ... bit 4, 0x10
                // RXBE   ... bit 1, 0x02
                // RXBF   ... bit 0, 0x01
                //cc = (U3TXBE ? 0x20 : 0) | (U3RXBE ? 2 : 0);
                cc = PIR9;
                db_setout();
                TOGGLE;
                LATC = cc; // U3 flag
                TOGGLE;
                //for (int i = 6; i-- > 0; ) nop;
            } else if(addr == UART_DREG) { // UART data register
                cc = U3RXB; // U3 RX buffer
                while(!(PIR9 & 2));
                db_setout();
                TOGGLE;
                LATC = cc;
                TOGGLE;
            } else { // invalid address
                db_setout();
                LATC = peek_ram(addr);
                //xprintf("%05lX: %02X %c bad\n", addr, PORTC, (RA5 ? 'R' : 'W'));
                //monitor_mode = 0;
            }
            if (ss_flag || monitor_mode) {
                xprintf("%05lX: %02X %c m%d\n", addr, PORTC, (RA5 ? 'R' : 'W'), monitor_mode);
                monitor(monitor_mode);
                monitor_mode = 0;
            }
#endif
            TOGGLE;
        } else { // 6809 write cycle (RW = 0)
            TOGGLE;
            TOGGLE;
            TOGGLE;
            addr = get_addr();
            // A19 == 1, I/O address area
            if(addr == UART_DREG) { // UART data register
                // this cycle actually starts at /MREQ down edge,
                // so too early PORTC reading will get garbage.
                // We need to wait for stable Z80 data bus.
                //for (int i = 5; i-- > 0; ) nop;
                TOGGLE;
                U3TXB = PORTC; // Write into U3 TX buffer
                TOGGLE;
                //xprintf("(%02X)", PORTC);
                //while(!(PIR9 & 2));
            //while(PIR9 & 2);
            //} else if((addr & 0xfff00) == DBG_PORT) {
            //    monitor_mode = 1;   // DBG_PORT write
            } else {
                TOGGLE;
                poke_ram(addr, PORTC);
                TOGGLE;
                // ignore write C000 or upper
                //xprintf("%05lX: %02X %c\n", addr, PORTC, (RA5 ? 'R' : 'W'));
                //monitor_mode = 1;
            }
            TOGGLE;
        }
#if 0
        if (ss_flag || monitor_mode) {
            xprintf("%05lX: %02X %c M%d\n", addr, PORTC, (rw_flag ? 'R' : 'W'), monitor_mode);
            //monitor(monitor_mode);
            monitor_mode = 0;
        }
#endif
    end_of_cycle:
        toggle_Egate();
        while(RA1); // Wait for IORQ == L or MREQ == L;
        TOGGLE;
        db_setin(); // Set data bus as input
        toggle_Egate();
        TOGGLE;
    }
}

