uart_d  equ 0e001h
uart_c  equ 0e000h
        org 0
        ld  sp,0100h
rxloop: ld  a,(uart_c)
        and a,1         ; check RXRDY
        jr  z,rxloop
        ld  a,(uart_d)
        push af
txloop: ld  a,(uart_c)
        and a,2         ; check TXRDY
        jr  z,txloop
        pop af
        ld  (uart_d),a
        jr  rxloop
        end

