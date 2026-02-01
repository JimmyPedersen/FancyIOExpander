# FancyIOExpander

Small I2C-based GPIO expander board used to add up to 16 digital IOs to a host MCU. This document covers supported functions, the I2C register map, typical operation sequences, and a compact pin mapping between board labels and MCU signals.

## Overview
- Device: 16-bit I/O expander (two 8-bit ports: A and B) accessed over I2C.
- Typical uses: additional GPIOs, input with pull-ups, interrupt-on-change, open-drain/latched outputs.
- I2C addressing: base 0x20 (A2,A1,A0 hardware address pins; device address = 0x20 | (A2<<2)|(A1<<1)|A0).

## Features
- 16 general-purpose digital I/Os (PORTA[0..7], PORTB[0..7])
- Per-pin direction (input/output)
- Per-pin polarity inversion
- Per-pin pull-ups (when configured as inputs)
- Interrupt-on-change with per-pin enable and compare options
- Output latch register for deterministic outputs
- Readback of GPIO and captured interrupt values

## Register map (byte addresses)
All registers are single-byte accessible. Two-port layout (A = port A, B = port B).

- 0x00 IODIRA   R/W  Direction for Port A (1=input, 0=output)
- 0x01 IODIRB   R/W  Direction for Port B
- 0x02 IPOLA    R/W  Input Polarity Port A (1=invert)
- 0x03 IPOLB    R/W  Input Polarity Port B
- 0x04 GPINTENA R/W  Interrupt on change enable Port A (1=enabled)
- 0x05 GPINTENB R/W  Interrupt on change enable Port B
- 0x06 DEFVALA  R/W  Default compare value for interrupt Port A
- 0x07 DEFVALB  R/W  Default compare value for interrupt Port B
- 0x08 INTCONA  R/W  Interrupt control Port A (1=compare to DEFVAL, 0=compare to previous)
- 0x09 INTCONB  R/W  Interrupt control Port B
- 0x0A IOCON    R/W  Configuration register (see IOCON bits below)
- 0x0B GPPUA    R/W  Pull-up enable Port A (1=pull-up enabled)
- 0x0C GPPUB    R/W  Pull-up enable Port B
- 0x0D INTFA    R    Interrupt flag Port A (read-only, 1=pin caused int)
- 0x0E INTFB    R    Interrupt flag Port B
- 0x0F INTCAPA  R    Interrupt captured value Port A (value at time of interrupt)
- 0x10 INTCAPB  R    Interrupt captured value Port B
- 0x11 GPIOA    R/W  Port A GPIO (read input or write output)
- 0x12 GPIOB    R/W  Port B GPIO
- 0x13 OLATA    R/W  Output latch Port A (read/write)
- 0x14 OLATB    R/W  Output latch Port B

### IOCON (typical bits)
- BANK (if present): register addressing mode (0 = sequential)
- MIRROR: INT pins mirrored (1 = INT pins are internally OR'ed)
- SEQOP: disable sequential operation (affects auto-increment)
- ODR: INT pin type (1 = open-drain)
- INTPOL: active level for INT (1 = active-high)

Refer to your variant's IOCON for exact bit positions.

## Register behavior notes
- IODIRx: set a bit to 1 to configure the corresponding pin as input; 0 = output.
- GPIOx vs OLATx: writing to GPIO of an output updates OLAT; read GPIO returns actual pin state, OLAT returns last written latch values.
- Interrupt clearing: reading INTCAPx or GPIOx typically clears the interrupt; INTFA/INTFB indicate which pins triggered.
- DEFVAL/INTCON: configure whether interrupts compare to a default value or to previous state.

## Quick operation sequences

1) Basic initialization (example I2C pseudo-ops)
- Write IOCON to configure INT polarity/ODR/SEQOP as desired.
- Write IODIRA/IODIRB to set directions.
- Write GPPUA/GPPUB to enable input pull-ups where needed.
- Clear interrupts by reading INTCAPx or GPIOx after configuration.

2) Configure a pin as input with pull-up and interrupt-on-change
- Write bit=1 in IODIRx (input)
- Set corresponding GPPUx bit = 1
- Set GPINTENx bit = 1
- Optionally configure INTCONx/DEFVALx to choose compare behavior

3) Drive an output
- Write bit=0 in IODIRx (output)
- Write to OLATx or GPIOx to set pin level (0 or 1)

4) Handling interrupts
- On INT asserted, master reads INTFx to see which pin(s) triggered, then reads INTCAPx to obtain captured values and clear the interrupt (or read GPIOx depending on variant).


