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
    
    // Address strobe
    output wire nAS_OE,
    
    // Lower/Upper byte on D active
    output wire nLDS_OE,
    output wire nUDS_OE,
    
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

//always @(posedge sys_clk) begin
//    if (mc_clk_falling) begin
always @(posedge mc_clk_falling) begin
        ipl_sync[0] <= ~IPL;
        ipl_sync[1] <= ipl_sync[0];

        if (ipl_sync[0] == ipl_sync[1])
            ipl <= ipl_sync[0];
//    end
end

// Synchronize bus control signal inputs
(* async_reg = "true" *) reg reset_sync;
(* async_reg = "true" *) reg halt_sync;
(* async_reg = "true" *) reg dtack_sync;
(* async_reg = "true" *) reg berr_n_sync;
(* async_reg = "true" *) reg [15:0] din_sync[0:1];

always @(posedge sys_clk) begin
    dtack_sync <= nDTACK;
    berr_n_sync <= nBERR;
    halt_sync <= nHALT_IN;
    din_sync[1] <= din_sync[0];
    din_sync[0] <= D_IN;

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
assign PI_GPIO_OUT[3] = req_active;
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

assign FC_OE = {3{r_fc_drive}};
assign D_OE = {16{r_dbus_drive}};
assign A_OE = {23{r_abus_drive}};

// Test pads
assign TP_OE = 8'b0;
assign TP19_OE = 0;
assign TP20_OE = 0;

// Pi control register.
reg [14:0] pi_control = 14'b00000000000000;
wire r_br_drive = pi_control[0];
wire r_reset_drive = pi_control[1];
wire r_halt_drive = pi_control[2];

reg r_bg_drive;
reg r_as_drive;
reg r_vma_drive;
reg r_bgack_drive;
reg r_rw_drive;
reg r_rw_clear;
reg r_lds_drive_read;
reg r_uds_drive_read;
reg r_lds_drive_write;
reg r_uds_drive_write;
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

(* async_reg = "true" *) reg [1:0] r_fb_uds;
(* async_reg = "true" *) reg [1:0] r_fb_lds;
(* async_reg = "true" *) reg [1:0] r_fb_as;
(* async_reg = "true" *) reg [1:0] r_fb_rw;

always @(posedge sys_clk) begin
    r_fb_uds <= { r_fb_uds[0], nUDS_OE };
    r_fb_lds <= { r_fb_lds[0], nLDS_OE };
    r_fb_as  <= { r_fb_as[0],  nAS_OE };
    r_fb_rw  <= { r_fb_rw[0],  RnW_OE };
end

FFLatch UDS(
    .OUT(nUDS_OE),
    .SET(r_uds_drive_read | r_uds_drive_write),
    .CLK(CLK_7M),
    .RESET(r_as_ds_clear)
);

FFLatch LDS(
    .OUT(nLDS_OE),
    .SET(r_lds_drive_read | r_lds_drive_write),
    .CLK(CLK_7M),
    .RESET(r_as_ds_clear)
);

FFLatch AS(
    .OUT(nAS_OE),
    .SET(r_as_drive),
    .CLK(CLK_7M),
    .RESET(r_as_ds_clear)
);

FFLatchPR RW(
    .OUT(RnW_OE),
    .SET(r_rw_drive),
    .CLK(CLK_7M),
    .RESET(r_rw_clear)
);

// Clock synchronizer
/*
(* async_reg = "true" *) reg [15:0] mc_clk_long;

reg mc_clk_rising;
reg mc_clk_falling;
reg mc_clk_latch;
reg mc_clk_latch_p1;

parameter DTACK_DELAY = 15;

(* async_reg = "true" *) reg [DTACK_DELAY:0] dtack_delay_line;

always @(negedge sys_clk) begin
    mc_clk_long <= { mc_clk_long[14:0], CLK_7M };
    dtack_delay_line <= {dtack_delay_line[DTACK_DELAY-1:0], DTACK};
    
    // The values for latch, rising and falling detector from shift register are matched
    // for 140 MHz sys_clk*/
/*
    case (mc_clk_long[7:6])
        2'b10: mc_clk_rising <= 1'b1;
        2'b01: mc_clk_falling <= 1'b1;
        default: begin
            mc_clk_rising <= 1'b0;
            mc_clk_falling <= 1'b0;
        end
    endcase
*/ /*
    case (mc_clk_long[1:0])
        2'b01: mc_clk_rising <= 1'b1;
        2'b10: mc_clk_falling <= 1'b1;
        default: begin
            mc_clk_rising <= 1'b0;
            mc_clk_falling <= 1'b0;
        end    
    endcase
    
    case (dtack_delay_line[DTACK_DELAY:DTACK_DELAY-2]) // synthesis full_case
        3'b110: DTACK_LATCH <= 1'b1;
        3'b100: DTACK_AFTER_LATCH <= 1'b1;
        default: begin
            MCCLK_RISING <= 1'b0;
            DTACK_AFTER_LATCH <= 1'b0;
        end
    endcase
end
*/

wire mc_clk_falling;
wire mc_clk_rising;
wire mc_clk_latch;

ClockSync CLKSync(
    .SYSCLK(sys_clk),
    .DTACK(nDTACK),
    .MCCLK(CLK_7M),
    .MCCLK_FALLING(mc_clk_falling),
    .MCCLK_RISING(mc_clk_rising),
    .DTACK_LATCH(mc_clk_latch)
);

// ## Pi interface.
localparam [2:0] PI_REG_DATA_LO = 3'd0;
localparam [2:0] PI_REG_DATA_HI = 3'd1;
localparam [2:0] PI_REG_ADDR_LO = 3'd2;
localparam [2:0] PI_REG_ADDR_HI = 3'd3;
localparam [2:0] PI_REG_STATUS = 3'd4;
localparam [2:0] PI_REG_CONTROL = 3'd4;
localparam [2:0] PI_REG_VERSION = 3'd7;

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
reg [1:0] r_size;
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
(* async_reg = "true" *) reg [1:0] pi_wr_sync;
wire pi_wr_falling = (pi_wr_sync == 2'b10);
wire pi_wr_rising = (pi_wr_sync == 2'b01);

always @(negedge sys_clk) begin
    pi_wr_sync <= { pi_wr_sync[0], PI_WR };
end

// ## Access state machine.
localparam [8:0] STATE_WAIT         = 'b000000000;
localparam [8:0] STATE_SETUP_BUS    = 'b000000001;
localparam [8:0] STATE_DRIVE_AS     = 'b000000010;
localparam [8:0] STATE_DRIVE_DS     = 'b000000100;
localparam [8:0] STATE_WAIT_DSACK   = 'b000001000;
localparam [8:0] STATE_ON_DSACK     = 'b000010000;
localparam [8:0] STATE_CONTINUE     = 'b000100000;
localparam [8:0] STATE_FINALIZE     = 'b001000000;

reg [8:0] state = STATE_WAIT;
reg high_word;

// Main state machine
always @(posedge sys_clk) begin
    if (pi_wr_falling) begin
        case (PI_A) // synthesis full_case
            PI_REG_DATA_LO: req_data_write[15:0] <= pi_data_in;
            PI_REG_DATA_HI: req_data_write[31:16] <= pi_data_in;
            PI_REG_ADDR_LO: req_address[15:0] <= pi_data_in;
            PI_REG_ADDR_HI: begin
                req_address[23:16] <= pi_data_in[7:0];
                req_size <= pi_data_in[9:8];
                req_read <= pi_data_in[10];
                req_fc <= pi_data_in[13:11];
                req_active <= 1'b1;
            end
            PI_REG_CONTROL: begin
                if (pi_data_in[15]) begin
                    pi_control <= pi_control | pi_data_in[14:0];                    
                end else
                    pi_control <= pi_control & ~pi_data_in[14:0];
            end
        endcase
    end 

    if (r_fb_rw[1] == 0) r_rw_clear <= 1'b0;
    else if (r_fb_rw[1] == 1) r_rw_drive <= 1'b0;

    if (r_fb_as[1] == 1)  r_as_drive <= 1'b0;
    if (r_fb_lds[1] == 1) begin 
        r_lds_drive_read <= 1'b0; 
        r_lds_drive_write <= 1'b0; 
    end
    if (r_fb_uds[1] == 1) begin
        r_uds_drive_read <= 1'b0;
        r_uds_drive_write <= 1'b0;
    end
    
    if (r_fb_as[1]  == 0/* && r_fb_lds[1] == 0 && r_fb_uds[1] == 0*/) r_as_ds_clear <= 1'b0;

    case (state) // synthesis full_case
        STATE_WAIT:
        begin
            // If request is pending (active) then start rolling    
            if (req_active) begin
                r_size <= req_size;
                r_is_read <= req_read;
                r_abus <= req_address;
                r_address_p2 <= req_address + 24'd2;
                r_data_write <= req_data_write;
                high_word <= req_size[1];
                second_cycle <= 1'b0;
                state <= STATE_SETUP_BUS;
            end
        end
        
        // Setup bus is combination of S0 and S1 - prepare data on D/A/FC and go to driving address strobe
        STATE_SETUP_BUS:
        begin
            if (high_word) r_dbus <= r_data_write[31:16];
            else r_dbus <= r_data_write[15:0];

            r_abus_drive <= 1'b1;
            r_fc_drive <= 1'b1;
            
            state <= STATE_DRIVE_AS;
        end
        
        // Drive address strobe and wait until it actually toggled. This happens in 7.14 MHz clock domain
        STATE_DRIVE_AS:
        begin
            // Assert address strobe
            r_as_drive <= 1'b1;

            // Drive RW low (for write) or high (for read)
            r_rw_drive <= ~r_is_read;
            
            // When entering S2 in read mode, drive LDS/UDS
            r_lds_drive_read <= r_is_read & (r_size[0] | r_abus[0]);
            r_uds_drive_read <= r_is_read & (r_size[0] | ~r_abus[0]);
            
            // On rising clk edge go to S2
            if (r_fb_as[1]) begin                
                state <= STATE_DRIVE_DS;
            end
        end
        
        // If write this is the second place where LDS/UDS can be driven
        STATE_DRIVE_DS:
        begin
            r_dbus_drive <= ~r_is_read;
            
            r_lds_drive_write <= ~r_is_read & (r_size[0] | r_abus[0]);
            r_uds_drive_write <= ~r_is_read & (r_size[0] | ~r_abus[0]);

            state <= STATE_WAIT_DSACK;
        end

        // Wait for DSACK and latch data (if read)
        STATE_WAIT_DSACK:
        begin
            if (mc_clk_latch) begin
                if (r_is_read) begin
                    if (high_word) req_data_read[31:16] <= din_sync[1];
                    else req_data_read[15:0] <= din_sync[1];
                end
                state <= STATE_ON_DSACK;
            end
        end
        
        // On DSACK (delayed) select next cycle and deassert AS/LDS/UDS/RnW
        STATE_ON_DSACK:
        begin
            r_as_ds_clear <= 1'b1;
            r_rw_clear <= 1'b1;
            
            if (!r_fb_as[1]) begin
                if (r_size == 'd3) begin
                    second_cycle <= 1'b1;
                    state <= STATE_CONTINUE;
                end
                else begin
                    req_active <= 1'b0; 
                    state <= STATE_FINALIZE;
                end
            end
        end
        
        STATE_CONTINUE:
        begin
            if (mc_clk_rising) begin
                r_abus_drive <= 1'b0;
                r_dbus_drive <= 1'b0;
                r_vma_drive <= 1'b0;
                r_fc_drive <= 1'b0;
                high_word <= 'b0;
                r_abus <= r_address_p2;
                r_size <= 2'b01;
                state <= STATE_SETUP_BUS;
            end
        end

        STATE_FINALIZE:
        begin           
            if (mc_clk_rising) begin
                r_abus_drive <= 1'b0;
                r_dbus_drive <= 1'b0;
                r_vma_drive <= 1'b0;
                r_fc_drive <= 1'b0;
                state <= STATE_WAIT;
            end
        end
    endcase
end

endmodule

