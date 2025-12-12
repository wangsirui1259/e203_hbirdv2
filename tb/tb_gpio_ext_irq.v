// ============================================================================
// GPIO External Interrupt Testbench for E203 HBirdV2 SoC
// ============================================================================
// This testbench simulates button press events to test the external GPIO
// interrupt functionality without requiring a physical development board.
//
// Button mapping (from board_ddr200t.h):
//   - Button U: GPIO bit 3
//   - Button D: GPIO bit 4
//   - Button L: GPIO bit 5
//   - Button R: GPIO bit 6
//   - Button C: GPIO bit 7
//
// The testbench generates simulated button press waveforms and monitors
// the PLIC external interrupt signal to verify interrupt handling.
// ============================================================================

`include "e203_defines.v"

module tb_gpio_ext_irq();

  // =========================================================================
  // Clock and Reset
  // =========================================================================
  reg  clk;
  reg  lfextclk;
  reg  rst_n;

  wire hfclk = clk;

  // =========================================================================
  // Internal Signal Hierarchy Defines
  // =========================================================================
  `define CPU_TOP u_e203_soc_top.u_e203_subsys_top.u_e203_subsys_main.u_e203_cpu_top
  `define EXU `CPU_TOP.u_e203_cpu.u_e203_core.u_e203_exu
  `define ITCM `CPU_TOP.u_e203_srams.u_e203_itcm_ram.u_e203_itcm_gnrl_ram.u_sirv_sim_ram

  // GPIO and PLIC signals
  `define PLIC_EXT_IRQ u_e203_soc_top.u_e203_subsys_top.u_e203_subsys_main.plic_ext_irq
  `define GPIOA_IRQ u_e203_soc_top.u_e203_subsys_top.u_e203_subsys_main.gpioA_irq

  // PC monitoring points
  `define PC_WRITE_TOHOST       `E203_PC_SIZE'h80000086

  wire [`E203_XLEN-1:0] x3 = `EXU.u_e203_exu_regfile.rf_r[3];
  wire [`E203_PC_SIZE-1:0] pc = `EXU.u_e203_exu_commit.alu_cmt_i_pc;
  wire [`E203_PC_SIZE-1:0] pc_vld = `EXU.u_e203_exu_commit.alu_cmt_i_valid;

  // =========================================================================
  // Test Counters and Status
  // =========================================================================
  reg [31:0] pc_write_to_host_cnt;
  reg [31:0] cycle_count;
  reg pc_write_to_host_flag;
  reg [31:0] gpio_irq_cnt;
  reg [31:0] plic_irq_cnt;

  // =========================================================================
  // Button Simulation Signals (directly drive GPIO input)
  // =========================================================================
  // GPIO bit mapping for buttons (from board_ddr200t.h)
  localparam BUTTON_U_BIT = 3;
  localparam BUTTON_D_BIT = 4;
  localparam BUTTON_L_BIT = 5;
  localparam BUTTON_R_BIT = 6;
  localparam BUTTON_C_BIT = 7;

  // GPIO input signals - directly connected to SoC
  reg [31:0] gpio_input;
  
  // Button press simulation registers
  reg btn_u_pressed;
  reg btn_d_pressed;
  reg btn_l_pressed;
  reg btn_r_pressed;
  reg btn_c_pressed;

  // Combine button signals into GPIO input
  always @(*) begin
    gpio_input = 32'b0;
    gpio_input[BUTTON_U_BIT] = btn_u_pressed;
    gpio_input[BUTTON_D_BIT] = btn_d_pressed;
    gpio_input[BUTTON_L_BIT] = btn_l_pressed;
    gpio_input[BUTTON_R_BIT] = btn_r_pressed;
    gpio_input[BUTTON_C_BIT] = btn_c_pressed;
  end

  // =========================================================================
  // PC Monitor - detect test completion
  // =========================================================================
  always @(posedge hfclk or negedge rst_n) begin 
    if(rst_n == 1'b0) begin
      pc_write_to_host_cnt <= 32'b0;
      pc_write_to_host_flag <= 1'b0;
    end
    else if (pc_vld & (pc == `PC_WRITE_TOHOST)) begin
      pc_write_to_host_cnt <= pc_write_to_host_cnt + 1'b1;
      pc_write_to_host_flag <= 1'b1;
    end
  end

  // Cycle counter
  always @(posedge hfclk or negedge rst_n) begin 
    if(rst_n == 1'b0) begin
      cycle_count <= 32'b0;
    end
    else begin
      cycle_count <= cycle_count + 1'b1;
    end
  end

  // =========================================================================
  // GPIO Interrupt Monitor
  // =========================================================================
  reg gpioa_irq_prev;
  reg plic_ext_irq_prev;

  always @(posedge hfclk or negedge rst_n) begin
    if(rst_n == 1'b0) begin
      gpio_irq_cnt <= 32'b0;
      plic_irq_cnt <= 32'b0;
      gpioa_irq_prev <= 1'b0;
      plic_ext_irq_prev <= 1'b0;
    end
    else begin
      gpioa_irq_prev <= `GPIOA_IRQ;
      plic_ext_irq_prev <= `PLIC_EXT_IRQ;
      
      // Count rising edges of GPIO interrupt
      if (`GPIOA_IRQ && !gpioa_irq_prev) begin
        gpio_irq_cnt <= gpio_irq_cnt + 1'b1;
        $display("[%0t] GPIO Interrupt Triggered! Count: %d", $time, gpio_irq_cnt + 1);
      end
      
      // Count rising edges of PLIC external interrupt
      if (`PLIC_EXT_IRQ && !plic_ext_irq_prev) begin
        plic_irq_cnt <= plic_irq_cnt + 1'b1;
        $display("[%0t] PLIC External Interrupt! Count: %d", $time, plic_irq_cnt + 1);
      end
    end
  end

  // =========================================================================
  // Testcase and Memory Loading
  // =========================================================================
  reg[8*300:1] testcase;
  integer dumpwave;

  initial begin
    $display("============================================================");
    $display("   GPIO External Interrupt Testbench for E203 HBirdV2");
    $display("============================================================");
    
    if($value$plusargs("TESTCASE=%s",testcase)) begin
      $display("TESTCASE=%s", testcase);
    end

    // Initialize signals
    pc_write_to_host_flag <= 0;
    clk      <= 0;
    lfextclk <= 0;
    rst_n    <= 0;
    
    // Initialize button states (all released)
    btn_u_pressed <= 1'b0;
    btn_d_pressed <= 1'b0;
    btn_l_pressed <= 1'b0;
    btn_r_pressed <= 1'b0;
    btn_c_pressed <= 1'b0;

    // Release reset
    #120 rst_n <= 1;
    
    $display("[%0t] Reset released, starting simulation...", $time);

    // Wait for test completion or timeout
    @(pc_write_to_host_cnt == 32'd8) #10;

    // Display test results
    $display("");
    $display("============================================================");
    $display("                    Test Results Summary");
    $display("============================================================");
    $display("TESTCASE: %s", testcase);
    $display("Total Cycle Count: %d", cycle_count);
    $display("GPIO Interrupts Detected: %d", gpio_irq_cnt);
    $display("PLIC External Interrupts: %d", plic_irq_cnt);
    $display("Final x3 Register Value: %d", x3);
    $display("============================================================");
    
    if (x3 == 1) begin
      $display("                      TEST PASSED");
      $display("============================================================");
    end
    else begin
      $display("                      TEST FAILED");
      $display("============================================================");
    end
    
    #10 $finish;
  end

  // =========================================================================
  // Button Press Simulation Task
  // =========================================================================
  // Task to simulate a button press with configurable duration
  task press_button;
    input [2:0] button_id;  // 0=U, 1=D, 2=L, 3=R, 4=C
    input [31:0] press_duration;  // in clock cycles
    begin
      case (button_id)
        3'd0: begin
          $display("[%0t] Button U pressed", $time);
          btn_u_pressed <= 1'b1;
          repeat(press_duration) @(posedge clk);
          btn_u_pressed <= 1'b0;
          $display("[%0t] Button U released", $time);
        end
        3'd1: begin
          $display("[%0t] Button D pressed", $time);
          btn_d_pressed <= 1'b1;
          repeat(press_duration) @(posedge clk);
          btn_d_pressed <= 1'b0;
          $display("[%0t] Button D released", $time);
        end
        3'd2: begin
          $display("[%0t] Button L pressed", $time);
          btn_l_pressed <= 1'b1;
          repeat(press_duration) @(posedge clk);
          btn_l_pressed <= 1'b0;
          $display("[%0t] Button L released", $time);
        end
        3'd3: begin
          $display("[%0t] Button R pressed", $time);
          btn_r_pressed <= 1'b1;
          repeat(press_duration) @(posedge clk);
          btn_r_pressed <= 1'b0;
          $display("[%0t] Button R released", $time);
        end
        3'd4: begin
          $display("[%0t] Button C pressed", $time);
          btn_c_pressed <= 1'b1;
          repeat(press_duration) @(posedge clk);
          btn_c_pressed <= 1'b0;
          $display("[%0t] Button C released", $time);
        end
        default: begin
          $display("[%0t] Invalid button ID: %d", $time, button_id);
        end
      endcase
    end
  endtask

  // =========================================================================
  // Button Press Sequence Generator
  // =========================================================================
  // This initial block generates button press events to trigger GPIO interrupts
  initial begin
    // Wait for system to stabilize after reset
    #500;
    
    // Wait additional cycles for software initialization
    repeat(5000) @(posedge clk);
    
    $display("");
    $display("============================================================");
    $display("          Starting Button Press Simulation");
    $display("============================================================");
    
    // Simulate button press sequence
    // Press Button U (rising edge trigger)
    press_button(3'd0, 100);  // Press for 100 cycles
    repeat(500) @(posedge clk);  // Wait between presses
    
    // Press Button D
    press_button(3'd1, 100);
    repeat(500) @(posedge clk);
    
    // Press Button L
    press_button(3'd2, 100);
    repeat(500) @(posedge clk);
    
    // Press Button R
    press_button(3'd3, 100);
    repeat(500) @(posedge clk);
    
    // Press Button C
    press_button(3'd4, 100);
    repeat(500) @(posedge clk);
    
    // Random button presses
    $display("");
    $display("[%0t] Starting random button press sequence...", $time);
    
    repeat(10) begin
      repeat($urandom_range(100, 500)) @(posedge clk);
      press_button($urandom_range(0, 4), $urandom_range(50, 200));
    end
    
    $display("");
    $display("[%0t] Button simulation completed.", $time);
    $display("============================================================");
  end

  // =========================================================================
  // Timeout Watchdog
  // =========================================================================
  initial begin
    #40000000
    $display("");
    $display("============================================================");
    $display("                   TIMEOUT - Test Terminated");
    $display("============================================================");
    $display("GPIO Interrupts Detected: %d", gpio_irq_cnt);
    $display("PLIC External Interrupts: %d", plic_irq_cnt);
    $finish;
  end

  // =========================================================================
  // Clock Generation
  // =========================================================================
  always begin 
    #2 clk <= ~clk;  // 250MHz clock
  end

  always begin 
    #33 lfextclk <= ~lfextclk;  // ~15MHz low frequency clock
  end

  // =========================================================================
  // Waveform Dump Configuration
  // =========================================================================
  initial begin
    if($value$plusargs("DUMPWAVE=%d", dumpwave)) begin
      if(dumpwave != 0) begin
        `ifdef vcs
          $display("VCS waveform dump enabled");
          $fsdbDumpfile("tb_gpio_ext_irq.fsdb");
          $fsdbDumpvars(0, tb_gpio_ext_irq, "+mda");
        `endif

        `ifdef iverilog
          $display("Icarus Verilog waveform dump enabled");
          $dumpfile("tb_gpio_ext_irq.vcd");
          $dumpvars(0, tb_gpio_ext_irq);
        `endif
      end
    end
  end

  // =========================================================================
  // Memory Initialization
  // =========================================================================
  integer i;
  reg [7:0] itcm_mem [0:(`E203_ITCM_RAM_DP*8)-1];
  
  initial begin
    $readmemh({testcase, ".verilog"}, itcm_mem);

    for (i=0; i<(`E203_ITCM_RAM_DP); i=i+1) begin
      `ITCM.mem_r[i][00+7:00] = itcm_mem[i*8+0];
      `ITCM.mem_r[i][08+7:08] = itcm_mem[i*8+1];
      `ITCM.mem_r[i][16+7:16] = itcm_mem[i*8+2];
      `ITCM.mem_r[i][24+7:24] = itcm_mem[i*8+3];
      `ITCM.mem_r[i][32+7:32] = itcm_mem[i*8+4];
      `ITCM.mem_r[i][40+7:40] = itcm_mem[i*8+5];
      `ITCM.mem_r[i][48+7:48] = itcm_mem[i*8+6];
      `ITCM.mem_r[i][56+7:56] = itcm_mem[i*8+7];
    end

    $display("ITCM Memory Initialized");
    $display("ITCM 0x00: %h", `ITCM.mem_r[8'h00]);
    $display("ITCM 0x01: %h", `ITCM.mem_r[8'h01]);
  end

  // =========================================================================
  // JTAG Interface (inactive)
  // =========================================================================
  wire jtag_TDI = 1'b0;
  wire jtag_TDO;
  wire jtag_TCK = 1'b0;
  wire jtag_TMS = 1'b0;
  wire jtag_TRST = 1'b0;
  wire jtag_DRV_TDO = 1'b0;

  // =========================================================================
  // SoC Instantiation
  // =========================================================================
  e203_soc_top u_e203_soc_top(
    .hfextclk(hfclk),
    .hfxoscen(),

    .lfextclk(lfextclk),
    .lfxoscen(),

    .io_pads_jtag_TCK_i_ival (jtag_TCK),
    .io_pads_jtag_TMS_i_ival (jtag_TMS),
    .io_pads_jtag_TDI_i_ival (jtag_TDI),
    .io_pads_jtag_TDO_o_oval (jtag_TDO),
    .io_pads_jtag_TDO_o_oe (),

    // GPIO A - connected to button simulation
    .io_pads_gpioA_i_ival(gpio_input),
    .io_pads_gpioA_o_oval(),
    .io_pads_gpioA_o_oe  (),

    // GPIO B - unused in this test
    .io_pads_gpioB_i_ival(32'b0),
    .io_pads_gpioB_o_oval(),
    .io_pads_gpioB_o_oe  (),

    // QSPI0
    .io_pads_qspi0_sck_o_oval (),
    .io_pads_qspi0_cs_0_o_oval(),
    .io_pads_qspi0_dq_0_i_ival(1'b1),
    .io_pads_qspi0_dq_0_o_oval(),
    .io_pads_qspi0_dq_0_o_oe  (),
    .io_pads_qspi0_dq_1_i_ival(1'b1),
    .io_pads_qspi0_dq_1_o_oval(),
    .io_pads_qspi0_dq_1_o_oe  (),
    .io_pads_qspi0_dq_2_i_ival(1'b1),
    .io_pads_qspi0_dq_2_o_oval(),
    .io_pads_qspi0_dq_2_o_oe  (),
    .io_pads_qspi0_dq_3_i_ival(1'b1),
    .io_pads_qspi0_dq_3_o_oval(),
    .io_pads_qspi0_dq_3_o_oe  (),

    // Reset and PMU
    .io_pads_aon_erst_n_i_ival (rst_n),
    .io_pads_aon_pmu_dwakeup_n_i_ival (1'b1),
    .io_pads_aon_pmu_vddpaden_o_oval (),
    .io_pads_aon_pmu_padrst_o_oval    (),

    // Boot configuration
    .io_pads_bootrom_n_i_ival       (1'b0),  // Boot from ROM
    .io_pads_dbgmode0_n_i_ival       (1'b1),
    .io_pads_dbgmode1_n_i_ival       (1'b1),
    .io_pads_dbgmode2_n_i_ival       (1'b1) 
  );

endmodule
