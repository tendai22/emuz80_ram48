/*
  PIC18F57Q43 ROM RAM and UART emulation firmware
  This single source file contains all code

  Target: emuz80_ram48 - The computer with only Z80 and PIC18F57Q43
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

#define Z80_CLK 2000000UL // Z80 clock frequency(Max 20MHz)

#define _XTAL_FREQ 64000000UL

#define TOGGLE do { LATD5 ^= 1; } while(0)

unsigned char ram[RAM_SIZE]; // Equivalent to RAM
union {
    unsigned int w; // 16 bits Address
    struct {
        unsigned char l; // Address low 8 bits
        unsigned char h; // Address high 8 bits
    };
} ab;

unsigned int break_address = 0; // break_address is zero, it is invalid
int ss_flag = 0;

#define db_setin() (TRISC = 0xff)
#define db_setout() (TRISC = 0x00)

// UART3 Transmit
void putch(int c) {
    while(!U3TXIF); // Wait or Tx interrupt flag set
    U3TXB = (unsigned char)c; // Write data
}

// UART3 Recive
int getch(void) {
    while(!U3RXIF); // Wait for Rx interrupt flag set
    return U3RXB; // Read data
}

// peek, poke
char peek_ram(addr_t addr)
{
    char c;
    TRISD &= ~0x1f;
    TRISB = 0;  // A0-12 output
    LATD = ((LATD & 0xe0) | (unsigned char)((addr >> 8) & 0x1f));
    LATB = (unsigned char)(addr & 0xff);
    db_setin();
    SRAM_OE = 0;  // OE = 0;
    c = PORTC;
    SRAM_OE = 1;
    TRISD |= 0x1f;
    TRISB = 0xff;   // A0-12 input
    return c;
}

void poke_ram(addr_t addr, char c)
{
    //xprintf("(%04x,%02x)", addr, c);
    TRISD &= ~0x1f;
    TRISB = 0;  // AA0-15 output
    LATD = ((LATD & 0xe0) | (unsigned char)((addr >> 8) & 0x1f));
    LATB = (unsigned char)(addr & 0xff);
    LATA2 = 0;  // WE = 0;
    db_setout();
    LATC = c;
    LATA2 = 1;
    db_setin();
    TRISD |= 0x1f;
    TRISB = 0xff;   // A0-15 input
}
    
void HALT_on(void)
{
    TRISE1 = 0;
    LATE1 = 0;
}

void HALT_off(void)
{
    //TRISE1 = 1; // RESET is Open-Drain and pulled-up, so
                // only do it input-mode is necessary
    LATE1 = 1;
}

void RESET_on(void)
{
    TRISE2 = 0; // output
    LATE2 = 0;
}

void RESET_off(void)
{
    //WPUE2 = 1;
    //TRISE2 = 1; // set as input
    LATE2 = 1;
}

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
    while ((c = getch()) == '.' && count++ < 2);
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

void manualboot(void)
{
    int c, cc, d, n, count;
    addr_t addr = 0, max = 0, min = RAM_TOP + RAM_SIZE;
    int addr_flag = 0;
    
    while (1) {
        while ((c = getchr()) == ' ' || c == '\t' || c == '\n' || c == '\r')
            ;   // skip white spaces
        if (c == -1)
            break;
        if (c == '!' && min < max) {
            xprintf("\n");
            // dump memory
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
        if (c == 's') { // start with single_step
            ss_flag = 1;
            break;   // start processor
        }
        if (c == 'g') { // start with no-single_step
            ss_flag = 0;
            break;      // start prosessor
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
        if (c < 0)
            break;
        if (d < 0) {
            if (addr_flag) {  // set address
                if (cc == '=')
                    addr = (addr_t)n;
                else if (cc == '%')
                    break_address = (addr_t)n;
            } else {
                if (RAM_TOP <= addr && addr < (RAM_TOP + RAM_SIZE)) {
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

#define GET_ADDR() ((RD6 ? 0x80000L : 0L) | ((unsigned long)(PORTD&0x1f)<<8) | PORTB)

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
    unsigned long addr = GET_ADDR();
    
    xprintf("|%05lX %02X %c ", addr, PORTC, ((RA5) ? 'R' : 'W'));
    
    if (monitor_mode == 2) {    // DBG_PORT read
        xprintf(" IN>");
        xgets(buf, 7, 0);
        int i = 0, n = 0;
        while (i < 8 && (c = buf[i++]) && (d = to_hex((unsigned char)c)) >= 0) {
            n *= 16; n += d;
            //xprintf("(%x,%x)", n, d);
        }
        LATC = (unsigned char)n;
    } else {
        if (monitor_mode == 1) { // DBG_PORT write
            xprintf(" OUT: %02x", (int)PORTC);
        }
#if 0
        if ((c = getch()) == '.')
            ss_flag = 0;
        else if (c == 's' || c == ' ')
            ss_flag = 1;
#endif
        xprintf("\n");
    }
}

void toggle_DTACK(void)
{
    CLCSELECT = 2;      // CLC3 select
    //CLCnPOL = 0x82;     // inverted the CLC3 output
    CLCnPOL ^= 0x80;    // toggle POL on CLC3
}

void reset_DFF(void)
{
    TOGGLE;
    CLCSELECT = 2;      // CLC3 select
    CLCnGLS3 = 0x40;    // 1 for D-FF RESET
    CLCnGLS3 = 0x80;    // 0 for D-FF RESET
    TOGGLE;
}


#define db_setin() (TRISC = 0xff)
#define db_setout() (TRISC = 0x00)

// main routine
void main(void) {
    int monitor_mode = 0;
    int count;
    unsigned long addr;
    // System initialize
    OSCFRQ = 0x08; // 64MHz internal OSC

    // xprintf initialize
    xdev_out(putch);
    xdev_in(getch);
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
    TRISD |= 0x1f; // Set as input

    // Address bus A7-A0 pin
    ANSELB = 0x00; // Disable analog function
    WPUB = 0xff; // Week pull up
    TRISB = 0xff; // Set as input

    // Data bus D7-D0 pin
    ANSELC = 0x00; // Disable analog function
    WPUC = 0xff; // Week pull up
    TRISC = 0xff; // Set as input(default)

    // IO pin assignment
    
    // RESET output pin
    ANSEL(RESET) = 0; // Disable analog function
    LAT(RESET) = 0; // RESET assert
    TRIS(RESET) = 0; // Set as output

    // BUSRQ output pin
    ANSEL(BUSRQ) = 0; // Disable analog function
    LAT(BUSRQ) = 0; // HALT assert
    TRIS(BUSRQ) = 0; // Set as output

    // INT output pin
    ANSEL(INT) = 0; // Disable analog function
    LAT(INT) = 1; // No interrupt request
    TRIS(INT) = 0; // Set as output

    // WAIT output pin
    ANSEL(WAIT) = 0; // Disable analog function
    LAT(WAIT) = 1; // DTACK negate
    TRIS(WAIT) = 0; // Set as output

    // M1 input pin
    ANSEL(M1) = 0; // Disable analog function
    TRIS(M1) = 1; // Set as input

    // RFSH input pin
    ANSEL(RFSH) = 0;
    TRIS(RFSH) = 1; // set as input
    
    // RD input pin
    ANSEL(RD) = 0; // Disable analog function
    WPU(RD) = 1; // Week pull up
    TRIS(RD) = 1; // Set as input

    // SRAM OE,WE
    // SRAM OE pin
    ANSEL(SRAM_OE) = 0;
    LAT(SRAM_OE) = 1;  // WE negate
    TRIS(SRAM_OE) = 0; // set as output
    
    // SRAM WE pin
    ANSEL(SRAM_WE) = 0;
    LAT(SRAM_WE) = 1;  // OE negate
    TRIS(SRAM_WE) = 0; // set as output

    // MREQ input pin
    ANSEL(MREQ) = 0; // Disable analog function
    WPU(MREQ) = 1; // Week pull up
    TRIS(MRFEQ) = 1; // Set as input

    // IORQ input pin
    ANSEL(IORQ) = 0; // Disable analog function
    WPU(IORQ) = 1; // Week pull up
    TRIS(IORQ) = 1; // Set as input

    // Z80 clock(RA3) by NCO FDC mode
    RA3PPS = 0x3f; // RA3 asign NCO1
    ANSELA3 = 0; // Disable analog function
    TRISA3 = 0; // NCO output pin
    NCO1INC = Z80_CLK * 2 / 61;
    NCO1CLK = 0x00; // Clock source Fosc
    NCO1PFM = 0;  // FDC mode
    NCO1OUT = 1;  // NCO output enable
    NCO1EN = 1;   // NCO enable

    // UART3 initialize
//    U3BRG = 416; // 9600bps @ 64MHz
    U3CON0 |= (1<<7);   // BRGS = 0, 4 baud clocks per bit
    U3BRG = 138;    // 115200bps @ 64MHz, BRG=0, 99%

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

    // TEST Pin RD7
    ANSEL(TEST) = 0;
    TRIS(TEST) = 0;
    LAT(TEST) = 0;
    
    // 1, 2, 5, 6: Port A, C
    // 3, 4, 7, 8: Port B, D
    RxyPPS(SRAM_OE) = 0x0;  // LATA4 -> RA4 -> /OE
    RxyPPS(SRAM_WE) = 0x0;  // LATA2 -> RA2 -> /WE
    RxyPPS(WAIT) = 0x0;  // LATB5 -> RB5 -> WAIT

    U3ON = 1; // Serial port enable
    xprintf(";");
    manualboot();

    TOGGLE;
    TOGGLE;
    // Re-initialze for CPU running
#if 1
    // Address bus A15-A8 pin
    ANSELD = 0x00; // Disable analog function
    //WPUD = 0x1f; // Week pull up
    TRISD |= 0x1f; // Set as input

    // Address bus A7-A0 pin
    ANSELB = 0x00; // Disable analog function
    //WPUB = 0xff; // Week pull up
    TRISB = 0xff; // Set as input

    // Data bus D7-D0 pin
    ANSELC = 0x00; // Disable analog function
    //WPUC = 0xff; // Week pull up
    TRISC = 0xff; // Set as input(default)
#endif
    // reconfigurate CLC devices
    // CLC pin assign
    // 0, 1, 4, 5: Port A, C
    // 2, 3, 6, 7: Port B, D
    CLCIN0PPS = 0x01;   // RA1 <- /MREQ
    CLCIN1PPS = 0x00;   // RA0 <- /IORQ
    CLCIN2PPS = 0x14;   // RB4 <- /RFSH
    CLCIN3PPS = 0x15;   // RB5 <- /WAIT
    CLCIN4PPS = 0x05;   // RA5 <- /RD

    // 1, 2, 5, 6: Port A, C
    // 3, 4, 7, 8: Port B, D
    RxyPPS(SRAM_OE) = 0x01;  // CLC1 -> RA4 -> /OE
    RxyPPS(SRAM_WE) = 0x02;  // CLC2 -> RA2 -> /WE
    RxyPPS(WAIT) = 0x03;     // CLC3 -> RD7 -> /WAIT
    
    // ============ CLC1 /OE
    // /OE = (/MREQ == 0 && /RD == 0 && /RFSH == 1)
    CLCSELECT = 0;  // CLC1 select
    CLCnCON &= ~0x80;
    
    CLCnSEL0 = 0;       // CLCIN0PPS <- /MREQ
    CLCnSEL1 = 2;       // CLCIN2PPS <- /RFSH
    CLCnSEL2 = 4;       // CLCIN4PPS <- /RD
    CLCnSEL3 = 127;     // NC
    
    CLCnGLS0 = 0x01;    // /MREQ == 0 (inverted)
    CLCnGLS1 = 0x08;    // /RFSH == 1 (non-inverted)
    CLCnGLS2 = 0x10;    // /RD == 0 (inverted)
    CLCnGLS3 = 0x40;    // 1 for AND gate
    
    CLCnPOL = 0x80;     // inverted the CLC1 output
    CLCnCON = 0x82;     // 4 input AMD
            
    // ============ CLC2 /WE
    // /OE = (/MREQ == 0 && /RD == 1 && /RFSH == 1)
    CLCSELECT = 1;  // CLC1 select
    CLCnCON &= ~0x80;
    
    CLCnSEL0 = 0;       // CLCIN0PPS <- /MREQ
    CLCnSEL1 = 2;       // CLCIN2PPS <- /RFSH
    CLCnSEL2 = 4;       // CLCIN4PPS <- /RD
    CLCnSEL3 = 127;     // NC
    
    CLCnGLS0 = 0x01;    // /DS == 0 (inverted)
    CLCnGLS1 = 0x08;    // /RD == 1 (not inverted)
    CLCnGLS2 = 0x20;    // /RFSH == 1 (not inverted)
    CLCnGLS3 = 0x40;    // 1 for AND gate
    
    CLCnPOL = 0x80;     // inverted the CLC1 output
    CLCnCON = 0x82;     // 4 input AMD
            
    // ============== CLC3 /WAIT
    CLCSELECT = 2;      // CLC3 select
    CLCnCON &= ~0x80;
    
    CLCnSEL0 = 1;       // D-FF CLK <- CLCIN1PPS <- /IORQ
    CLCnSEL1 = 127;     // D-FF D NC
    CLCnSEL2 = 127;     // D-FF SET NC
    CLCnSEL3 = 127;     // D-FF RESET NC
    
    CLCnGLS0 = 0x01;    // /IORQ ~|_  (inverted)
    CLCnGLS1 = 0x00;    // /RD (non inverted)
    CLCnGLS2 = 0x00;    // D-FF RESET (soft reset)
    CLCnGLS3 = 0x00;    // 0 for D-FF RESET
    
    CLCnPOL = 0x82;     // inverted the CLC3 output
    CLCnCON = 0x84;     // Select D-FF (no interrupt)
        
  
    
    xprintf("start ss = %d, bp = %04X\n", ss_flag, break_address);
    // Z80 start
    //CLCDATA = 0x7;
    HALT_off();
    TOGGLE;
    RESET_off();    // RESET negate
    db_setin();
    TOGGLE;
    TOGGLE;
    TOGGLE;
    reset_DFF();
    while(1){
        while(!RD7);    // Wait for /DTACK == 1
//        while(RA0);     // Wait for /AS == 0
//        while(!RD6);
        // /DTACK == 1, later we need to set /DTACK == 0
        TOGGLE;
        TOGGLE;
        if (!RD6) 
            goto end_of_cycle;
        if(RA5) { // MC68008 read cycle (RW = 1)
            db_setout();
            addr = GET_ADDR();
            // A19 == 1, IO address
            if (addr == UART_CREG){ // UART control register
                // PIR9 pin assign
                // U3TXIF ... bit 1, 0x02
                // U3RXIF ... bit 0, 0x01
                LATC = PIR9; // U3 flag
            } else if(addr == UART_DREG) { // UART data register
                LATC = U3RXB; // U3 RX buffer
            } else if((addr & 0xfff00) == DBG_PORT) {
                monitor_mode = 2;   // DBG_PORT read
            } else { // invalid address
                xprintf("%05lX: %02X %c bad\n", addr, PORTC, (RA5 ? 'R' : 'W'));
                monitor_mode = 0;
            }
            if (ss_flag || monitor_mode) {
                xprintf("%05lX: %02X %c m%d\n", addr, PORTC, (RA5 ? 'R' : 'W'), monitor_mode);
                monitor(monitor_mode);
                monitor_mode = 0;
            }
        } else { // MC68008 memory write cycle (RW = 0)
            addr = GET_ADDR();
            // A19 == 1, I/O address area
            if(addr == UART_DREG) { // UART data register
                U3TXB = PORTC; // Write into U3 TX buffer
            } else if((addr & 0xfff00) == DBG_PORT) {
                monitor_mode = 1;   // DBG_PORT write
            } else {
                xprintf("%05lX: %02X %c\n", addr, PORTC, (RA5 ? 'R' : 'W'));
                monitor_mode = 1;
            }
        }
        if (ss_flag || monitor_mode) {
            xprintf("%05lX: %02X %c M%d\n", addr, PORTC, (RA5 ? 'R' : 'W'), monitor_mode);
            monitor(monitor_mode);
            monitor_mode = 0;
        }
    end_of_cycle:
        while(RA1); // Wait for DS = 0;
        HALT_on();
        reset_DFF(); // reset D-FF, /DTACK be zero
        while(!RA1); // Wait for DS = 1;
        db_setin(); // Set data bus as output
        HALT_off();
    }
}

