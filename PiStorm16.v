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

// System clock comes from PLL
wire sys_clk = SYS_PLL_CLKOUT0;

// Emu68 debug console, either asynchronous or synchronous serial
// Wire DBG pins as outputs from Pi GPIO 
assign DBG_DAT = PI_GPIO_IN[5];
assign DBG_CLK = PI_GPIO_IN[27];
assign PI_GPIO_OE[5] = 0;
assign PI_GPIO_OE[27] = 0;

// EClock generator
EClock eclock(
    .CLOCK_IN(CLK_7M),
    .ECLOCK_OUT(ECLK)
);

// Synchronize IPL, and handle skew.
wire [2:0] ipl;
IPLSynch IPLsync(
    .CLK(~CLK_7M),
    .IPL(ipl),
    .IPL_ASYNC(IPL)
);

// Synchronize bus control signal inputs
(* async_reg = "true" *) reg reset_sync;
(* async_reg = "true" *) reg halt_sync;
(* async_reg = "true" *) reg berr_n_sync;

always @(posedge sys_clk) begin
    berr_n_sync <= nBERR;
    halt_sync <= nHALT_IN;

    if (mc_clk_falling) begin
        reset_sync <= nRESET_IN;
        is_bm <= ~nBG_IN;
    end
end

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

reg r_bg_drive;
reg r_as_drive;
reg r_vma_drive;
reg r_bgack_drive;
reg r_rw_drive;
reg r_rw_clear;
reg r_lds_drive;
reg r_uds_drive;
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

ClockSync CLKSync (
    .SYSCLK(sys_clk),
    .DTACK(nDTACK),
    .MCCLK(CLK_7M),
    .DTACK_DELAY(r_dtack_delay),
    .MCCLK_FALLING(mc_clk_falling),
    .MCCLK_RISING(mc_clk_rising),
    .DTACK_LATCH(mc_clk_latch)
);

reg second_cycle;

reg r_terminated_normally;
reg is_bm;

reg [23:0] r_address_p2;

reg [23:0] r_abus;
reg [15:0] r_dbus;
reg r_size;
reg r_is_read;

assign D_OUT = r_dbus[15:0];
assign A_OUT = r_abus[23:1];
assign FC_OUT = req_fc;

reg [15:0] r_data_write [0:1];
reg [15:0] r_data_read [0:1];

wire r_br_drive;
wire r_reset_drive;
wire r_halt_drive;
wire [7:0] r_dtack_delay;

reg [31:0] req_data_read;
wire [31:0] req_data_write;
wire [23:0] req_address;
wire [1:0] req_size;
wire [2:0] req_fc;
wire req_read;
wire req_active;

RasPi pi_interface(
    .PI_GPIO_IN(PI_GPIO_IN),
    .PI_GPIO_OUT(PI_GPIO_OUT),
    .PI_GPIO_OE(PI_GPIO_OE),
    
    .IPL(ipl),
    .HALT(halt_sync),
    .MASTER(is_bm),
    .RESET(reset_sync | r_reset_drive),
    
    .DRIVE_BR(r_br_drive),
    .DRIVE_HALT(r_halt_drive),
    .DRIVE_RESET(r_reset_drive),
    .DTACK_DELAY(r_dtack_delay),
    
    .SYSCLK(sys_clk),
    .CLEAR_ACTIVE(r_clear_req_active),
    .SECOND_CYCLE(second_cycle),
    .ERROR(~r_terminated_normally),
    
    .REQUEST_DATA_IN(req_data_read),
    .REQUEST_DATA_OUT(req_data_write),
    .REQUEST_ADDRESS(req_address),
    .REQUEST_SIZE(req_size),
    .REQUEST_FC(req_fc),
    .REQUEST_IS_READ(req_read),
    .REQUEST_ACTIVE(req_active)
);

reg [3:0] state = STATE_WAIT;
wire [3:0] next_state;

reg high_word;

FSMComb fsmc(
    .ACTIVATE(req_active & CLK_7M),
    .LATCH(mc_clk_latch),
    .MUST_CONTINUE(high_word),
    .MC_CLK_RISING(mc_clk_rising),
    .AS_FEEDBACK(r_fb_as[1]),
    .CURRENT(state),
    .NEXT(next_state)
);

reg r_clear_req_active;

always @(*) begin
    r_dbus = r_data_write[high_word];  
end

always @(*) begin
    if (state == STATE_WAIT_DSACK) begin
        r_data_read[high_word] = D_IN;
    end
end

// Main state machine
always @(posedge sys_clk)
begin
    if (!req_active) r_clear_req_active <= 1'b0;

    state <= next_state;
                 
    case (state)
        STATE_WAIT:
        begin
        end
        
        STATE_ACTIVATE:
        begin
            r_control_drive <= 1'b1;
            r_size <= req_size[0];
            r_is_read <= req_read;
            r_abus <= req_address;
                
            r_address_p2 <= req_address + 24'd2;
            r_data_write[0] <= req_data_write[15:0];
            r_data_write[1] <= req_data_write[31:16];
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
            r_lds_drive <= r_is_read & (r_size | r_abus[0]);
            r_uds_drive <= r_is_read & (r_size | ~r_abus[0]);
        end
        
        // If write this is the second place where LDS/UDS can be driven
        STATE_DRIVE_DS:
        begin
            r_as_drive <= 1'b0;
            
            r_dbus_drive <= ~r_is_read;
            
            r_lds_drive <= ~r_is_read & (r_size | r_abus[0]);
            r_uds_drive <= ~r_is_read & (r_size | ~r_abus[0]);
        end

        // Wait for DSACK and latch data (if read)
        STATE_WAIT_DSACK:
        begin
            // Everything done in combinatorial logic
        end
        
        STATE_LATCH:
        begin           
            req_data_read <= { r_data_read[1], r_data_read[0] };
            r_lds_drive <= 1'b0;
            r_uds_drive <= 1'b0;
            r_rw_drive <= 1'b0;

            second_cycle <= 1'b1;
            r_clear_req_active <= ~high_word;
        end
        
        STATE_CLEAR_AS:
        begin
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
        
        default:
        begin
        end
        
    endcase
end

endmodule
