# GPIO External Interrupt Testbench

## Overview

The `tb_gpio_ext_irq.v` testbench simulates button press events to test the external GPIO interrupt functionality of the E203 HBirdV2 SoC without requiring a physical development board.

## Button Mapping

Based on `board_ddr200t.h`, the buttons are mapped to GPIOA pins as follows:

| Button | GPIO Bit | Description |
|--------|----------|-------------|
| U (Up) | 3 | Navigation Up |
| D (Down) | 4 | Navigation Down |
| L (Left) | 5 | Navigation Left |
| R (Right) | 6 | Navigation Right |
| C (Center) | 7 | Select/Confirm |

## Interrupt Flow

The interrupt path is:

```
Button Press → GPIOA Input → GPIO Peripheral (apb_gpio)
    → gpioA_irq → PLIC (e203_subsys_plic)
    → plic_ext_irq → CPU External Interrupt
```

## Testbench Features

1. **Simulated Button Presses**: Generates waveforms that simulate button press and release events
2. **Interrupt Monitoring**: Tracks and displays GPIO and PLIC interrupt counts
3. **Waveform Dump**: Supports VCD/FSDB waveform output for debugging
4. **Configurable Test Duration**: Button press duration and intervals are configurable

## Usage

### With Icarus Verilog

```bash
cd vsim
make install
cd run
# Modify TB_V_FILES in Makefile to use tb_gpio_ext_irq.v instead of tb_top.v
iverilog -o sim.out -I ../install/rtl/core -I ../install/rtl/perips \
    -I ../install/rtl/perips/apb_i2c -D iverilog -g2005-sv \
    ../install/rtl/*/*.v ../install/rtl/*/*/*.v \
    ../install/tb/tb_gpio_ext_irq.v
vvp sim.out +TESTCASE=/path/to/testcase +DUMPWAVE=1
gtkwave tb_gpio_ext_irq.vcd
```

### With VCS

```bash
cd vsim
make install
cd run
# Modify TB_V_FILES in Makefile to use tb_gpio_ext_irq.v instead of tb_top.v
vcs +v2k -sverilog -full64 -timescale=1ns/10ps \
    +incdir+../install/rtl/core+../install/rtl/perips \
    ../install/rtl/*/*.v ../install/rtl/*/*/*.v \
    ../install/tb/tb_gpio_ext_irq.v
./simv +TESTCASE=/path/to/testcase +DUMPWAVE=1
verdi -ssf tb_gpio_ext_irq.fsdb &
```

## Test Program Requirements

To properly test GPIO interrupts, you need a test program that:

1. Configures GPIO pins 3-7 as inputs
2. Enables rising edge interrupts on these pins
3. Registers a PLIC interrupt handler for GPIOA (IRQ 15)
4. Enables global interrupts

Example C code (similar to `sdk/plic.c`):

```c
#include "hbirdv2.h"
#include "hbirdv2_gpio.h"

#define BUTTON_MASK 0x000000F8  // Bits 3-7

void plic_gpioa_handler(void) {
    int mask = gpio_clear_interrupt(GPIOA);
    // Handle the specific button
    if (mask & (1 << 3)) { /* Button U pressed */ }
    if (mask & (1 << 4)) { /* Button D pressed */ }
    // ...
}

int main(void) {
    gpio_enable_input(GPIOA, BUTTON_MASK);
    gpio_enable_interrupt(GPIOA, BUTTON_MASK, GPIO_INT_RISE);
    PLIC_Register_IRQ(PLIC_GPIOA_IRQn, 1, plic_gpioa_handler);
    __enable_irq();
    while(1);
}
```

## Signal Monitoring

The testbench monitors the following signals:

- `gpio_input[31:0]`: Simulated GPIO input values
- `gpioA_irq`: GPIO peripheral interrupt output
- `plic_ext_irq`: PLIC external interrupt to CPU

## Expected Output

When running the testbench, you should see messages like:

```
============================================================
   GPIO External Interrupt Testbench for E203 HBirdV2
============================================================
[500] Reset released, starting simulation...
[41000] Button U pressed
[41800] GPIO Interrupt Triggered! Count: 1
[41800] PLIC External Interrupt! Count: 1
[42200] Button U released
...
============================================================
                    Test Results Summary
============================================================
GPIO Interrupts Detected: 15
PLIC External Interrupts: 15
============================================================
```

## Notes

- The testbench is designed to work with the same memory initialization method as `tb_top.v`
- Button debouncing is not simulated (buttons have clean edges)
- The interrupt count depends on the test program's interrupt handling
