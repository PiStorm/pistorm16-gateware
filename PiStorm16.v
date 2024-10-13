// Copyright © 2024 Michal Schulz <michal.schulz@gmx.de>
// https://github.com/michalsc
//
// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
//
//
// PiStorm16 module
// ================
// The top module of PiStorm16 firmware connecting everything else in place.

`default_nettype none

module PiStorm16(
    // Amiga clock
    input wire CLK_7M,
    output wire ECLK,
    
    // Amiga address bus
    output wire [23:1] A_OUT,
    output wire [23:1] A_OE,
    
    // Amiga data bus
    input wire [15:0] D_IN,
    output wire [15:0] D_OUT,
    output wire [15:0] D_OE,
    
    // EXT port
    input wire [7:2] EXT_IN,
    output wire [7:2] EXT_OUT,
    output wire [7:2] EXT_OE,
    
    // Read/Write
    output wire RnW_OE,
    output wire RnW_OUT,
    
    // Address strobe
    output wire nAS_OE,
    output wire nAS_OUT,
    
    // Lower/Upper byte on D active
    output wire nLDS_OE,
    output wire nUDS_OE,
    
    output wire nLDS_OUT,
    output wire nUDS_OUT,
    
    input wire nDTACK,
    input wire nBERR,
    
    input wire nBG_IN,
    output wire nBG_OE,
    
    input wire nBR_IN,
    output wire nBR_OE,
    
    input wire nBGACK_IN,
    output wire nBGACK_OE,
    
    input wire nHALT_IN,
    output wire nHALT_OE,
    
    input wire nRESET_IN,
    output wire nRESET_OE,
    
    input wire nVPA,
    input wire nVMA_IN,
    output wire nVMA_OE,
    
    // IPL
    input wire [2:0] IPL,
    
    // FC2..0
    output wire [2:0] FC_OUT,
    output wire [2:0] FC_OE,
    
    // Pi interface
    input wire [27:0] PI_GPIO_IN,
    output wire [27:0] PI_GPIO_OUT,
    output wire [27:0] PI_GPIO_OE,
    
    // Debug port
    output wire DBG_DAT,
    output wire DBG_CLK,
    
    // Test pads
    input wire [8:1] TP_IN,
    output wire [8:1] TP_OUT,
    output wire [8:1] TP_OE,
    
    input wire TP19_IN,
    output wire TP19_OUT,
    output wire TP19_OE,
    
    input wire TP20_IN,
    output wire TP20_OUT,
    output wire TP20_OE,
        
    // PLL
    input wire SYS_PLL_CLKOUT0,
    input wire IN_CLK_50M
);

`include "global.vh"

// EClock generator
EClock eclock(
    .CLOCK_IN(CLK_7M),
    .ECLOCK_OUT(ECLK)
);

// System clock comes from PLL
wire sys_clk = SYS_PLL_CLKOUT0;

// Wire DBG pins as outputs from Pi GPIO 
assign DBG_DAT = PI_GPIO_IN[5];
assign DBG_CLK = PI_GPIO_IN[27];
assign PI_GPIO_OE[5] = 0;
assign PI_GPIO_OE[27] = 0;

// Synchronize IPL, and handle skew.
(* async_reg = "true" *) reg [2:0] ipl_sync [1:0];
reg [2:0] ipl;

always @(negedge CLK_7M) begin
    ipl_sync[0] <= ~IPL;
    ipl_sync[1] <= ipl_sync[0];

    if (ipl_sync[0] == ipl_sync[1])
        ipl <= ipl_sync[0];
end

// Synchronize bus control signal inputs
(* async_reg = "true" *) reg reset_sync;
(* async_reg = "true" *) reg halt_sync;
(* async_reg = "true" *) reg berr_n_sync;

always @(posedge sys_clk) begin
    berr_n_sync <= nBERR;
    halt_sync <= nHALT_IN;

    if (mc_clk_falling) begin
        reset_sync <= nRESET_IN;
        is_bm <= nBG_IN;
    end
end

// Wire IPL line as inputs from 68k bus to PI
assign PI_GPIO_OUT[2:0] = ~ipl;
assign PI_GPIO_OE[2:0] = 3'b111;

// req_active is on when transfer is in progress. It is always exposed to GPIO3
reg req_active;
assign PI_GPIO_OUT[3] = PI_REG_DATA_HI == PI_A ? ~second_cycle : req_active;
assign PI_GPIO_OE[3] = 'b1;

// KB_RESET (reset Into CPU) is exposed on GPIO4
// When r_reset_drive is set to 1, nRESET_IN will be masked by it!!!
assign PI_GPIO_OUT[4] = (reset_sync | r_reset_drive);
assign PI_GPIO_OE[4] = 'b1;

// RD or WR commands from Pi to FPGA, always input
wire PI_RD;
wire PI_WR;

assign PI_RD = PI_GPIO_IN[6];
assign PI_WR = PI_GPIO_IN[7];
assign PI_GPIO_OE[7:6] = 2'b00;

// Data out from FPGA to Pi
reg [15:0] pi_data_out;
assign PI_GPIO_OUT[23:8] = pi_data_out;

// Data out from FPGA to Pi is driven on read requests only (WR=1, RD=0)
wire drive_pi_data_out = ~PI_RD & PI_WR;
assign PI_GPIO_OE[23:8] = {16{drive_pi_data_out}};

// Data from Pi to FPGA is read on the same port inputs
wire [15:0] pi_data_in;
assign pi_data_in = PI_GPIO_IN[23:8];

// Address from Pi to FPGA, always input
wire [2:0] PI_A;
assign PI_A = PI_GPIO_IN[26:24];
assign PI_GPIO_OE[26:24] = 3'b000;

// Registers used to drive entire buses at once
reg r_fc_drive = 0;
reg r_abus_drive = 0;
reg r_dbus_drive = 0;
reg r_control_drive = 0;

assign FC_OE = {3{r_fc_drive}};
assign D_OE = {16{r_dbus_drive}};
assign A_OE = {23{r_abus_drive}};
assign nLDS_OE = r_control_drive;
assign nUDS_OE = r_control_drive;
assign RnW_OE = r_control_drive;
assign nAS_OE = r_control_drive;

// Test pads
assign TP_OE = 8'b0;
assign TP19_OE = 0;
assign TP20_OE = 0;

// Pi control register.
reg [14:0] pi_control = 15'b00000000000000;
wire r_br_drive = pi_control[0];
wire r_reset_drive = pi_control[1];
wire r_halt_drive = pi_control[2];
wire [7:0] r_dtack_delay = {1'b0, pi_control[14:8]};

reg r_bg_drive;
reg r_as_drive;
reg r_vma_drive;
reg r_bgack_drive;
reg r_rw_drive;
reg r_rw_clear;
reg r_lds_drive;
reg r_uds_drive;
reg r_uds_clear;
reg r_lds_clear;
reg r_as_ds_clear;

// Disable all outputs for now
assign nVMA_OE = r_vma_drive;
assign nRESET_OE = r_reset_drive;
assign nHALT_OE = r_halt_drive;
assign nBGACK_OE = r_bgack_drive;
assign nBR_OE = r_br_drive;
assign nBG_OE = r_bg_drive;

(* async_reg = "true" *) reg [1:0] r_fb_as;

always @(posedge sys_clk) begin
    r_fb_as  <= { r_fb_as[0],  ~nAS_OUT };
end

FFLatchN UDS(
    .OUT(nUDS_OUT),
    .SET(r_uds_drive),
    .CLK(CLK_7M),
    .RESET(r_as_ds_clear)
);

FFLatchN LDS(
    .OUT(nLDS_OUT),
    .SET(r_lds_drive),
    .CLK(CLK_7M),
    .RESET(r_as_ds_clear)
);

FFLatchN AS(
    .OUT(nAS_OUT),
    .SET(r_as_drive),
    .CLK(CLK_7M),
    .RESET(r_as_ds_clear)
);

FFLatchNPR RW(
    .OUT(RnW_OUT),
    .SET(r_rw_drive),
    .CLK(CLK_7M),
    .RESET(r_rw_clear)
);

wire mc_clk_falling;
wire mc_clk_rising;
wire mc_clk_latch;
wire mc_clk_sync;

/*  Frequency   DTACK_DELAY   DTACK_PROBED
     120 MHz        15
     133 MHz        16
     143 MHz        18
     145 MHz        20             16
*/
ClockSync /*#(.DTACK_DELAY(16))*/ CLKSync (
    .SYSCLK(sys_clk),
    .DTACK(nDTACK),
    .MCCLK(CLK_7M),
    .DTACK_DELAY(r_dtack_delay),
    .MCCLK_FALLING(mc_clk_falling),
    .MCCLK_RISING(mc_clk_rising),
    .MCCLK_SYNC(mc_clk_sync),
    .DTACK_LATCH(mc_clk_latch)
);

localparam [3:0] FW_MAJOR = 4'd1;
localparam [3:0] FW_MINOR = 4'd0;
localparam [2:0] FW_TYPE_PS32 = 3'd1;
localparam [2:0] FW_TYPE_PS16 = 3'd2;
localparam [4:0] FW_EXT_DATA = 5'd0;

wire [15:0] firmware_version = { FW_MAJOR, FW_MINOR, FW_TYPE_PS16, FW_EXT_DATA };
wire [15:0] pi_status = {7'd0, second_cycle, req_active, req_terminated_normally, ipl, halt_sync, reset_sync, is_bm};

reg [2:0] req_fc;
reg req_read;
reg second_cycle;
reg [1:0] req_size;
reg req_terminated_normally;
reg is_bm;
reg [31:0] req_data_read;
reg [31:0] req_data_write;
reg [23:0] req_address;
reg [23:0] r_address_p2;

reg [23:0] r_abus;
reg [15:0] r_dbus;
reg r_size_word;
reg r_is_read;

assign D_OUT = r_dbus[15:0];
assign A_OUT = r_abus[23:1];
assign FC_OUT = req_fc;

reg [31:0] r_data_write;

always @(*) begin
    case (PI_A)
        PI_REG_DATA_LO: pi_data_out = req_data_read[15:0];
        PI_REG_DATA_HI: pi_data_out = req_data_read[31:16];
        PI_REG_ADDR_LO: pi_data_out = req_address[15:0];
        PI_REG_ADDR_HI: pi_data_out = {2'd0, req_fc, req_read, req_size, req_address[23:16]};
        PI_REG_STATUS: pi_data_out = pi_status;
        PI_REG_VERSION: pi_data_out = firmware_version;
        default: pi_data_out = 16'bx;
    endcase
end

// Synchronize WR command from Pi
//reg pi_wr_a;
//reg pi_wr_b;
//wire pi_wr_falling = (pi_wr_sync == 2'b10);
//wire pi_wr_rising = (pi_wr_sync == 2'b01);

//reg pi_wr_falling;

reg addr_hi_written;

always @(negedge PI_WR) begin       
    addr_hi_written <= 1'b0;
    case (PI_A) // synthesis full_case
        PI_REG_DATA_LO: req_data_write[15:0] <= pi_data_in;
        PI_REG_DATA_HI: req_data_write[31:16] <= pi_data_in;
        PI_REG_ADDR_LO: req_address[15:0] <= pi_data_in;
        PI_REG_ADDR_HI: begin
            req_address[23:16] <= pi_data_in[7:0];
            req_size <= pi_data_in[9:8];
            req_read <= pi_data_in[10];
            req_fc <= pi_data_in[13:11];
            addr_hi_written <= 1'b1;
        end
        PI_REG_CONTROL: begin
            if (pi_data_in[15]) begin
                pi_control <= pi_control | pi_data_in[14:0];                    
            end else
                pi_control <= pi_control & ~pi_data_in[14:0];
        end
    endcase
end

always @(posedge addr_hi_written or posedge r_clear_req_active) begin
    if (r_clear_req_active) req_active <= 1'b0;
    else if (addr_hi_written) req_active <= 1'b1;
end

reg [3:0] state = STATE_WAIT;
wire [3:0] next_state;

reg high_word;

FSMComb fsmc(
    .ACTIVATE(req_active & CLK_7M),
    .LATCH(mc_clk_latch),
    .MUST_CONTINUE(high_word),
    .MC_CLK_RISING(mc_clk_rising), // CLK_7M),
    .AS_FEEDBACK(~nAS_OUT), // r_fb_as[1]),
    .CURRENT(state),
    .NEXT(next_state)
);

reg r_clear_req_active;

reg [31:0] r_data_read;

always @(*) begin
    if (high_word) 
        r_dbus = r_data_write[31:16];
    else
        r_dbus = r_data_write[15:0];
end

always @(posedge mc_clk_latch) begin
    if (high_word) 
        r_data_read[31:16] <= D_IN;
    else
        r_data_read[15:0] <= D_IN;
end

// Main state machine
always @(posedge sys_clk) begin

    //if (!req_active) r_clear_req_active <= 1'b0;

    state <= next_state;
    
    case (state) // synthesis full_case
        STATE_WAIT:
        begin
            //r_as_ds_clear <= 1'b1;
            //r_rw_clear <= 1'b1;
            //r_control_drive <= 1'b0;
        end
        
        STATE_WAKEUP:
        begin
        end
        
        STATE_ACTIVATE:
        begin
            r_control_drive <= 1'b1;
            r_size_word <= req_size[0];
            r_is_read <= req_read;
            r_abus <= req_address;
                
            r_address_p2 <= req_address + 24'd2;
            r_data_write <= req_data_write;
            high_word <= req_size[1];
            second_cycle <= 1'b0;
        end
        
        // Setup bus is combination of S0 and S1 - prepare data on D/A/FC and go to driving address strobe
        STATE_SETUP_BUS:
        begin
            r_abus_drive <= 1'b1;
            r_fc_drive <= 1'b1;
        end
        
        // Drive address strobe and wait until it actually toggled. This happens in 7.14 MHz clock domain
        STATE_DRIVE_AS:
        begin
            // Assert address strobe
            r_as_drive <= 1'b1;

            // Drive RW low (for write) or high (for read)
            r_rw_drive <= ~r_is_read;
            
            // When entering S2 in read mode, drive LDS/UDS
            r_lds_drive <= r_is_read & (r_size_word | r_abus[0]);
            r_uds_drive <= r_is_read & (r_size_word | ~r_abus[0]);
        end
        
        // If write this is the second place where LDS/UDS can be driven
        STATE_DRIVE_DS:
        begin
            r_as_drive <= 1'b0;
            
            r_dbus_drive <= ~r_is_read;
            
            r_lds_drive <= ~r_is_read & (r_size_word | r_abus[0]);
            r_uds_drive <= ~r_is_read & (r_size_word | ~r_abus[0]);
        end

        // Wait for DSACK and latch data (if read)
        STATE_WAIT_DSACK:
        begin
            // Everything done in combinatorial logic
        end
        
        STATE_LATCH:
        begin           
            req_data_read <= r_data_read;
            r_lds_drive <= 1'b0;
            r_uds_drive <= 1'b0;
            r_rw_drive <= 1'b0;

            second_cycle <= 1'b1;
            r_clear_req_active <= ~high_word;
        end
        
        STATE_CLEAR_AS:
        begin
            r_clear_req_active <= 1'b0;
            r_as_ds_clear <= 1'b1;
            r_rw_clear <= 1'b1;
        end
        
        // On DSACK (delayed) select next cycle and deassert AS/LDS/UDS/RnW
        STATE_ON_DSACK:
        begin
            // Everything done in combinatorial logic
        end
        
        STATE_FINALIZE:
        begin
            r_as_ds_clear <= 1'b0;
            r_rw_clear <= 1'b0;
            r_abus_drive <= 1'b0;
            r_dbus_drive <= 1'b0;
            r_vma_drive <= 1'b0;
            r_fc_drive <= 1'b0;
            r_control_drive <= high_word;
        end
        
        STATE_CONTINUE:
        begin
            high_word <= 'b0;
            r_abus <= r_address_p2;
        end
    endcase
end

endmodule

