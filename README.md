# USB composite device with 4 CDC

This project implements USB composite device with 4 CDC/UART. Tested on PIC24FJDA206.

| Function    | IOIO Pin#   | PIC24FJ Pin# |
|-------------|:-----------:| ------------:|
|U1CTS (in)   |    1        |    RF4       |
|U1RTS (out)  |    2        |    RF5       |
|U1RX  (in)   |    3        |    RD8       |
|U1TX  (out)  |    4        |    RD9       |
|U2CTS (in)   |   10        |    RD1       |
|U2RTS (out)  |   11        |    RD2       |
|U2RX  (in)   |   12        |    RD3       |
|U2TX  (out)  |   13        |    RD4       |
|U3CTS (in)   |   27        |    RG6       |
|U3RTS (out)  |   28        |    RG7       |
|U3RX  (in)   |   29        |    RG8       |
|U3TX  (out)  |   30        |    RG9       |
|U4CTS (in)   |   37        |    RB6       |
|U4RTS (out)  |   38        |    RB7       |
|U4RX  (in)   |   39        |    RB8       |
|U4TX  (out)  |   40        |    RB9       |

Hardware flow control is not implemented, though there is traces of RTS and CTS 
pins being assigned. The reason for this is that hardware flow control setup 
information (whether it is enabled or not) is not contained in any USB CDC 
control data. Therefore, it is not possible for the firmware to know when to 
enable or disable hardware flow control. Another alternative is to always enable
hardware flow control and have external pull-down resistors on both RTS ad CTS 
lines. This allows the UART to still work even if RTS and CTS lines are not 
connected. However, since my PIC24FJ board (IOIO OTG) does not have those 
external pull-down resistor, hardware flow control is not implemented.
