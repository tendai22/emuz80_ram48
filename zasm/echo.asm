uart_d  equ 01h
uart_c  equ 00h
txbf    equ 10h
rxbe    equ 02h
        org 0
        ld  sp,0f000h
rxloop: in  a,(uart_c)  ; PIR9
        and a,rxbe         ; check RXIF
        jr  nz,rxloop
        in  a,(uart_d)
        push af
        ;ld  b,a
txloop: in  a,(uart_c)  ; U3FIFO
        and a,txbf      ; check TXIF
        jr  nz,txloop
        pop af
        ;ld  a,b
        out  (uart_d),a
        jr  rxloop
        end

