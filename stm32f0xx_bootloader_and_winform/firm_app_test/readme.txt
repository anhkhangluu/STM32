Using Winform to load "stm32f0_1804.bin" or "stm32f0_1805.bin" to board
When using file "stm32f0_1804.bin", after loading it to board, if using hterm to transmit
a data 0x18 0x04 to board via serial port then board will feed back a data 0x18 0x04 to hterm and loop with circle 1s.
To stop feeding back data from board to hterm, send 0x00 to board
*Note: to load a new firm to board, feeding back data from board to hterm must be stopped

When using file "stm32f0_1805.bin", everythings is similar to file "stm32f0_1804.bin"
But data feed back is 0x18 0x05