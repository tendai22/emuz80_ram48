uart_d  equ 0e001h
uart_c  equ 0e000h
        org 0
        ld  sp,01000h
rxloop: ld  a,(uart_c)  ; PIR9
        and a,1         ; check RXIF
        jr  z,rxloop
        ld  a,(uart_d)
        push af
txloop: ld  a,(uart_c)  ; U3FIFO
        and a,2      ; check TXIF
        jr  z,txloop
        pop af
        ld  (uart_d),a
        jr  rxloop
        end

