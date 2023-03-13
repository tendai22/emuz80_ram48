/* 
 * File:   iopin.h
 * Author: tendai22plus@gmail.com (Norihiro Kumagai)
 *
 * Created on 2023/03/13, 10:25
 */

#ifndef __IOPIN_H
#define __IOPIN_H

/* pin defintion for MPUPU_RAM48 on Z80(normal 40pin version) */
#define LAT(pin) LAT ## pin
#define TRIS(pin) TRIS ## pin
#define ANSEL(pin) ANSEL ## pin
#define WPU(pin) WUP ## pin
#define RxyPPS(pin) R ## pin ## PPS

/* cpu controls */
#define RESET   E1
#define INT     E2
#undef NMI
#define BUSRQ   E0
#undef BUSAK
#undef HALT

/* memory access */
#define WAIT    B5
#define MREQ    A1
#define IORQ    A0
#define RD      A5
#undef WR
#define M1      B7
#define RFSH    B4

#define SRAM_OE A4
#define SRAM_WE A2

/* others */
#define TEST    B6

#endif //__IOPIN_H
