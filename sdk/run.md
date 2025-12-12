# GPIO External Interrupt Simulation Guide

This document explains how to simulate the GPIO external interrupt demo (demo_plic) using Icarus Verilog.

## Prerequisites

1. Install Icarus Verilog:
   ```bash
   sudo apt-get install iverilog
   ```

## Compile the Simulation

```bash
cd /path/to/e203_hbirdv2
iverilog -o gpio_tb.exec \
    -I rtl/e203/core \
    -I rtl/e203/perips \
    -I rtl/e203/perips/apb_i2c \
    -D DISABLE_SV_ASSERTION=1 \
    -D iverilog \
    -g2005-sv \
    $(find rtl -name "*.v") \
    tb/tb_gpio_ext_int.v
```

## Run the Simulation

### With Firmware
```bash
mkdir -p run_gpio_test && cd run_gpio_test
vvp ../gpio_tb.exec +TESTCASE=../sdk/demo_plic +DUMPWAVE=1
```

### Quick Hardware Test (Force GPIO Configuration)
For testing the GPIO interrupt hardware without waiting for firmware initialization:
```bash
mkdir -p run_gpio_test && cd run_gpio_test
vvp ../gpio_tb.exec +FORCE_GPIO_CONFIG +DUMPWAVE=1
```

This mode forces the GPIO INTEN and INTTYPE registers to enable rising-edge interrupts
on the button pins (GPIO[3:7]), allowing quick verification of the interrupt signal path.

## Simulation Parameters

The testbench accepts several plusargs:

- `+TESTCASE=<path>`: Path to firmware hex file (without .verilog extension)
- `+DUMPWAVE=1`: Enable waveform dump (creates .vcd file)
- `+TIMEOUT=<cycles>`: Override simulation timeout (default: 100M cycles)
- `+FORCE_GPIO_CONFIG`: Force GPIO configuration for quick hardware testing

## Important Notes

### Firmware Initialization Time

The demo_plic firmware uses printf for debug output, which requires UART initialization.
This takes a significant number of clock cycles (potentially millions) before the firmware
reaches the point where it configures GPIO interrupts.

The testbench monitors the GPIO INTEN register to detect when the firmware has configured
GPIO interrupts. Once detected, it begins simulating button presses.

### Simulation Speed

Icarus Verilog is a free, open-source simulator but is slower than commercial simulators.
A full simulation of the demo_plic firmware may take several minutes.

For faster testing, consider:
1. Creating a minimal test firmware that directly configures GPIO/PLIC without printf
2. Using a commercial simulator (VCS, ModelSim, etc.)
3. Reducing INIT_WAIT_CYCLES parameter in the testbench

### GPIO to PLIC Interrupt Path

The interrupt path from GPIO button press to CPU interrupt:
1. Button press (rising edge) on GPIO[3-7]
2. GPIO detects edge and sets interrupt pending in GPIO_INTSTATUS
3. GPIO asserts gpioA_irq signal
4. PLIC receives gpioA_irq on input 15 (PLIC_GPIOA_IRQn)
5. PLIC evaluates priority and asserts plic_ext_irq to CPU
6. CPU takes external interrupt if enabled (mstatus.MIE and mie.MEIE)
7. Firmware ISR handles interrupt

### Expected Test Results

When successful, you should see:
- "GPIOA Interrupt triggered!" messages for each button press
- LED outputs changing (GPIO[20-25])
- Non-zero interrupt count in the test summary