# FancyIOExpander

Firmware for a custom I2C-based IO Expander running on a PIC18F24Q10 microcontroller. This device provides 16 additional GPIO pins, along with ADC, DAC, PWM, and EEPROM functionality, controlled via I2C.

## Features

- **16 General Purpose I/O Pins** organized into 2 virtual ports (Port 0 and Port 1).
- **Advanced GPIO Control**:
  - Direction (Input/Output) via `TRIS` register.
  - Output Latch `LAT` and Port Read `PORT`.
  - Input Levels types `INLVL`.
  - Slew Rate Control `SLRCON`.
  - Open Drain Control `ODCON`.
  - Weak Pull-Up `WPU`.
  - Analog Select `ANSEL`.
  - Interrupt-On-Change (IOC) with Flag (`IOCxF`), Negative Edge (`IOCxN`), and Positive Edge (`IOCxP`).
- **Analog Features**:
  - ADC Reading on all pins (mapped to PIC ADC channels).
  - DAC Output (5-bit resolution).
- **PWM Output**: 4 PWM/CCP modules map-able to pins.
- **EEPROM**: 256 bytes of user-accessible storage.
- **Configurable I2C Address**: Default `0x2A` (Decimal 42).

## Hardware Pin Mapping

The 16 virtual pins are mapped to the PIC18F24Q10 physical pins as follows:

### Port 0
| Virtual Pin | Physical Pin | Port/Bit | Analog Channel | Notes |
|-------------|--------------|----------|----------------|-------|
| P0.0        | 3            | RA3      | ANA3           | |
| P0.1        | 4            | RA4      | ANA4           | |
| P0.2        | 5            | RA5      | ANA5           | |
| P0.3        | 7            | RA7      | ANA7           | |
| P0.4        | 6            | RA6      | ANA6           | |
| P0.5        | 16           | RC0      | ANC0           | |
| P0.6        | 17           | RC1      | ANC1           | |
| P0.7        | 18           | RC2      | ANC2           | |

### Port 1
| Virtual Pin | Physical Pin | Port/Bit | Analog Channel | Notes |
|-------------|--------------|----------|----------------|-------|
| P1.0        | 12           | RB4      | ANB4           | |
| P1.1        | 11           | RB3      | ANB3           | |
| P1.2        | 10           | RB2      | ANB2           | |
| P1.3        | 9            | RB1      | ANB1           | |
| P1.4        | 8            | RB0      | ANB0           | |
| P1.5        | 23           | RC7      | ANC7           | |
| P1.6        | 22           | RC6      | ANC6           | |
| P1.7        | 21           | RC5      | ANC5           | |

## I2C Interface

- **Slave Address**: `0x2A` (default).
- **SDA/SCL**: RC3/RC4 (Standard I2C pins on PIC18F Q10).

The device uses a 16-bit address protocol (Big Endian on wire) to access both EEPROM and Internal Registers.

### Address Structure

The 16-bit address sent by the master is interpreted as follows:

- **1st Byte (High Byte)**: `[CMD] [P2] [P1] [P0] [X3] [X2] [X1] [X0]`
  - Bit 7 (`CMD`): Mode Select.
    - `0` = EEPROM Access.
    - `1` = Register/Command Access.
  - Bits 6-4 (`P`): Port Number (0-7).
  - Bits 3-0 (`X`): Extra Data / Pin Number (0-15).
- **2nd Byte (Low Byte)**: `[C7] [C6] [C5] [C4] [C3] [C2] [C1] [C0]`
  - Bits 7-0 (`C`): Command ID / Register ID.

### EEPROM Access (`CMD` = 0)
When Bit 7 of the High Byte is `0`:
- Address range: `0x0000` to `0x00FF`.
- Provides read/write access to internal 256 bytes of EEPROM.

### Register Access (`CMD` = 1)
When Bit 7 of the High Byte is `1` (e.g., Address `0x80XX`):
- Low Byte specifies the Register or Command ID.
- High Byte specifies target Port and Pin/Extra data.

#### Standard GPIO Registers (ID 0-31)
These registers map directly to the PIC per-port registers.

| ID | Name | R/W | Description |
|----|------|-----|-------------|
| 0  | LAT | R/W | Output Latch |
| 1 | PORT | R | Input Port Value |
| 2 | TRIS | R/W | Tristate Control (1=Input, 0=Output) |
| 3 | INLVL | R/W | Input Level Select |
| 4 | SLRCON | R/W | Slew Rate Control |
| 5 | ODCON | R/W | Open Drain Control |
| 6 | WPU | R/W | Weak Pull-Up Enable |
| 7 | ANSEL | R/W | Analog Select (1=Analog, 0=Digital) |
| 8 | IOCxF | R/W | Interrupt-on-Change Flag |
| 9 | IOCxN | R/W | IOC Negative Edge Enable |
| 10 | IOCxP | R/W | IOC Positive Edge Enable |

**Usage**: Set High Byte to `0x80` (Port 0) or `0x90` (Port 1) + standard command ID in Low Byte.
Example: Write `0x55` to `LAT0` (Port 0 Latch) -> Write I2C Addr `0x8000`, Data `0x55`.

#### Special Functions

**Chip Info (ID 32)** - *Read Only*
- **Address**: `0x8020` (Port 0)
- **Read Byte 0**: Firmware Version.
- **Read Byte 1**: Interface Configuration (`(PortCount << 4) + PinsPerPort` = 0x28 for 2 ports, 8 pins).

**DAC Control (ID 32)** - *Write Only*
- **Address**: `0x8020`
- **Data**: Value (0-31). Sets the 5-bit DAC1 output.

**ADC Read (ID 33)** - *Read Only*
- **Address**: `0x[8+Port][Pin] 21` (Hex)
  - **High Byte**: Constructs from Port and Pin.
    - Port 0: `0x8` + Pin Hex Digit. (e.g., P0.5 -> `0x85`)
    - Port 1: `0x9` + Pin Hex Digit. (e.g., P1.0 -> `0x90`)
  - **Low Byte**: `0x21` (Command 33).
- **Read**: Returns 16-bit ADC value (Little Endian).
  - 1st Byte: Low byte of result.
  - 2nd Byte: High byte of result.
- **Note**: Reading multiple bytes creates an auto-incrementing block read of ADC values across pins.

**PWM Configuration (ID 33)** - *Write Only*
- **Address**: `0x[8+Port][Pin] 21` (Hex)
  - Same addressing schema as ADC Read.
  - **High Byte**: `0x8X` for Port 0, `0x9X` for Port 1.
  - **Low Byte**: `0x21`.
- **Data**: Module Index.
  - `0`: Disable PWM on this pin.
  - `1`: CCP1 (Port C/B supported usually, see limitations).
  - `2`: CCP2.
  - `3`: PWM3.
  - `4`: PWM4.
- **Constraints**:
  - Modules 1-2 (CCP) cannot be mapped to Port A (Virtual P0.0-P0.4).
  - Modules 3-4 (PWM) cannot be mapped to Port B (Virtual P1.0-P1.4).

**PWM Duty Cycle (ID 34)** - *Write Only*
- **Address**: `0x8[Module] 22` (Hex)
  - **High Byte**: `0x80` + Module Index.
  - **Low Byte**: `0x22` (Command 34).
  - *Example*: Set Duty for Module 3 -> Address `0x8322`.
- **Data**: 8-bit Duty Cycle.
  - The value is shifted left by 2 internally (effectively set to top 8 bits of 10-bit duty).

## Example Sequences

### Initialize Pin as Output
1. Write `TRIS` (ID 2): Address `0x8002` (Port 0, Register 2), Data `0x00` (All outputs).
2. Write `LAT` (ID 0): Address `0x8000`, Data `0x01` (Set Pin 0 high).

### Read Analog Value from P0.0
1. Set `ANSEL` (ID 7): Address `0x8007`, Data `0x01` (Set Pin 0 to analog).
2. Read ADC (ID 33):
   - Write Address: `0x8021` (Port 0, Pin 0 [low nibble=0], Command 33).
   - Restart/Read 2 Bytes. -> Returns ADC Value of P0.0.

### Enable PWM on P0.5 (Port C pin)
1. Configure Pin (ID 33):
   - Address: `0x8521` (Port 0 [bits 6-4=0], Pin 5 [bits 3-0=5] -> `0x80 | 0x05` = `0x85`, Cmd 33).
   - Data: `3` (Select PWM3 Module).
2. Set Duty Cycle (ID 34):
   - Address: `0x8322` (Extra/Module=3 -> `0x80 | 0x03` = `0x83`, Cmd 34).
   - Data: `128` (50% Duty).
