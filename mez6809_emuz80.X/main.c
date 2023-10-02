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
    // For 6809E, no EXTAL pin, so E/Q clocks should be provided early
#undef  MC6809E
#define MC6809E
#if defined(MC6809E)
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
    
    // ====== CLC2 ... 2div presclar/gate
    CLCSELECT = 1;
    CLCnCON = 0;        // CLC2 temporary disabled
    CLCnPOL = 0;        // not inverted output/g1g2g3g4
    
    CLCnSEL0 = 0x34;    // g1 <- CLC2
    CLCnSEL1 = 0x01;    // g2 <- CLCIN1 <- RA1(E)
    CLCnSEL2 = 0x2a;    // g3 <- NCO1
    CLCnSEL3 = 127;     // g4 <- no connect
    
    CLCnGLS0 = 0x20;    // D-FF CLK <- D3T
    CLCnGLS1 = 0x09;    // D1N (temporary) 0x05;    // D-FF D   <- D1N && D2N
    CLCnGLS2 = 0x80;    // 0 for D-FF RESET
    CLCnGLS3 = 0x80;    // 0 for D-FF SET

    CLCnPOL = 0x00;     // non inverted the CLC3 output
    CLCnCON = 0x84;     // Select D-FF (no interrupt)
#define TEST_CLC
#if defined(TEST_CLC)
    // E(34)-MREQ(19)-RA1
    ANSELA1 = 0;
    LATA1 = 0;
    TRISA1 = 0;
    // Q(35)-RFSH(28)-RA2
    ANSELA2 = 0;
    LATA2 = 0;
    TRISA2 = 0;
    // RD-RA5 to EXTAL 2div output for debug
    ANSELA5 = 0;
    LATA5 = 0;
    TRISA5 = 0;         // output
    
    LATA5 ^= 1;
    LATA5 ^= 1;
    LATA5 ^= 1;
    LATA5 ^= 1;
#endif
    // CLC4 ... 2bit sync counter bit 0
    CLCSELECT = 3;
    CLCnCON = 0;        // disable CLC3
    
    CLCnSEL0 = 0x36;    // g1 <- CLC4
    CLCnSEL1 = 127;     // 0x02;    // g2 <- CLCIN1 <- RA1(E)
    CLCnSEL2 = 0x34;    // g3 <- CLC2
    CLCnSEL3 = 127;     // g4 <- no connect
    
    CLCnGLS0 = 0x20;    // D-FF CLK <- D3T
    CLCnGLS1 = 0x01;    // D-FF D   <- D1N (or if gating is needed, add D2T/D2N
    CLCnGLS2 = 0x80;    // 0 for D-FF RESET
    CLCnGLS3 = 0x80;    // 0 for D-FF SET

    CLCnPOL = 0x00;     // non inverted the CLC3 output
    CLCnCON = 0x84;     // Select D-FF (no interrupt)

    // CLC6 ... 2bit sync couter bit 1
    CLCSELECT = 5;
    CLCnCON = 0;        // disable CLC4
    
    CLCnSEL0 = 0x37;    // g1 <- CLC5
    CLCnSEL1 = 127;     // NC (or gating input) 
    CLCnSEL2 = 0x34;    // g3 <- CLC2
    CLCnSEL3 = 127;     // NC 
    
    CLCnGLS0 = 0x20;    // D-FF CLK <- D3T
    CLCnGLS1 = 0x01;    // D-FF D   <- D1N
    CLCnGLS2 = 0x80;    // 0 for D-FF RESET
    CLCnGLS3 = 0x80;    // 0 for D-FF SET

    CLCnPOL = 0x00;     // non inverted output
    CLCnCON = 0x84;     // Select D-FF (no interrupt)

    // CLC5 ... XOR gate for 2bit sync couunter
    CLCSELECT = 4;
    CLCnCON = 0;        // disable CLC2
    
    CLCnSEL0 = 0x36;    // g1 <- CLC4
    CLCnSEL1 = 127;// 0x02;    // g2 <- CLCIN1 <- RA1(E)
    CLCnSEL2 = 0x38;    // g3 <- CLC6
    CLCnSEL3 = 127;     // g4 <- no connect
    
    CLCnGLS0 = 0x02;    // gate1 <- D1T
    CLCnGLS1 = 0x80;    // gate2 <- 0
    CLCnGLS2 = 0x20;    // gate3 <- D3T
    CLCnGLS3 = 0x80;    // gate4 <- 0

    CLCnPOL = 0x00;     // non inverted the CLC3 output
    CLCnCON = 0x81;     // OR-XOR 

    // Z80 clock(RA3) by NCO FDC mode
    NCO1INC = MEZ6809_CLK * 2 / 61;
    NCO1INC = 0x10000;
    NCO1CLK = 0x00; // Clock source Fosc
    NCO1PFM = 0;  // FDC mode
    NCO1OUT = 1;  // NCO output enable
    NCO1EN = 1;   // NCO enable

//    RE0PPS = 0x02;      // RE0 <- CLC2OUT (EXTAL)
    // CLC output selections
    ANSELD0 = 0;
    TRISD0 = 0;
    RD0PPS = 0x04;  // RC0 <- CLC4OUT
    RA1PPS = 0x05;  // 0x05: CLC5OUT
    RA2PPS = 0x06;  // 0x06: CLC6OUT
    RA5PPS = 0x02;  // 02: CLC2OUT // 0x3f:NCO1

    TOGGLE;TOGGLE;
    // sample loop for clock stretch
    while(1) {
        while(!RA1);    // wait for E upper edge
        TOGGLE;
        for (int i = 0; i < 3; ++i) {
            nop;    // dummy time eater
        }
        //
        CLCSELECT = 1;  // select CLC2
        CLCnGLS1 ^= 0x0c;   // toggle D2T,D2N
        TOGGLE;
        while(RA1);     // wait for E down edge
        TOGGLE;
        db_setin();
        CLCSELECT = 1;  // select CLC2
        CLCnGLS1 ^= 0x0c;   // toggle D2T,D2N
        TOGGLE;
    }
#else //!MC6809E
    ANSELA3 = 0;
    LATA3 = 0;
    TRISA3 = 0; // set as output
    RA3PPS = 0; // need it for handmade clock starts
    // handmade clock, settle HD6809 down in RESET&&HALT
    for (int i = 0; i < 38; ++i) {
        LATA3 ^= 1; // toggle it
    }
#endif

    // CPU test
    // DB out 0x12
    ANSELC = 0;
    TRISC = 0;      // output
    LATC = 0x12;    // NOP instruction
    
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
    
    init_boot();
    manualboot();
    end_boot();
    xprintf("->\n");

    // Clock output connect
#if defined(MC6809E)
    RA3PPS = 0;         // RA3 <- default GPIO
    
#else
    RA3PPS = 0x01;      // RA3 <- CLC2OUT
#endif
    

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

