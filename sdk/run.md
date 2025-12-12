# Compile with Icarus Verilog
iverilog -o gpio_tb.exec \
    -I rtl/e203/core \
    -I rtl/e203/perips \
    -I rtl/e203/perips/apb_i2c \
    -D DISABLE_SV_ASSERTION=1 \
    -D iverilog \
    -g2005-sv \
    $(find rtl -name "*.v") \
    tb/tb_gpio_ext_int.v

# Run simulation
mkdir run_gpio_test && cd run_gpio_test
vvp ../gpio_tb.exec +DUMPWAVE=1