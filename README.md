# STM32_ADF4351
Reference design for ADF4351 PLL control some a STM32 uC

# uC targeting
This design is fairly simple and will work on most STM32 uC.
The current target is the NUCLEO-L011K4 evaluation board.

# IO configuration
This design communicates with a host PC via a rs232 interface.
The microcontroller computes the register values and loads
the ADF4351 with values based on commands received.
SPI interface is handled in adf4351.c
Code is based on AD's reference implementation code base.

# Communications Protocol
Commands are single ascii chars followed by options arguments.
Command delimiter is the ';' charactor.

The communications protocol supports the following commands:
* set frequency (f)
     Set the PLL Frequency to output:
     - Command: f<freq_hz>;
          ex set to 146.835 MHz
            'f146835000;'
     - Responses:
       If accepted you should received 'k;'
       error will receive '?;'
* noop command (n)
    send noop command.  Does nothing, but does reply with ok.
    - Command: 'n;'
    - Response: 'k;' or '?;' if error in parsing.
* ident command (i)
    send identification request command
    - Command: i;
    - Response: 'ADF4351 PLL Controller v1;' or '?;' on error.
* pll power down (d)
    power down output from PLL.  (mute output)
    - Command: 'd;'
    - Response: 'k;' or '?;' on error.
* pll power up (u)
    power up output from PLL. (un-mute output)
    - Command: 'u;'
    - Response: 'k;' or '?;' on error.


