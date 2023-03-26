uart_d  equ 0e001h
uart_c  equ 0e000h
        org 0
        ld  sp,0100h
        jp  02000h
        org 8h
        jp  putch
        org 010H        ; getkey, ret key to A
        jp  getch
        org 018H        ; kbhit, ret NZ if key ready
        jp  kbhit
        org 040H
getch:  ld  a,(uart_c)
        and a,1         ; check RXRDY
        jr  z,getch
        ld  a,(uart_d)
        ret
kbhit:  ld  a,(uart_c)
        and a,1         ; check RXRDY
        ret
putch:  push af
putch1: ld  a,(uart_c)
        and a,2         ; check TXRDY
        jr  z,putch1
        pop af
        ld  (uart_d),a
        ret
        end
