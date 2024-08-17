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
    output wire RnW_OUT,
    output wire RnW_OE,
    
    // Address strobe
    output wire nAS_OE,
    
    // Lower/Upper byte on D active
    output wire nLDS_OUT,
    output wire nLDS_OE,
    output wire nUDS_OUT,
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
    input wire IN_CLK_50M,
    input wire SYS_PLL_LOCKED
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

always @(posedge sys_clk) begin
    if (mc_clk_falling) begin
        ipl_sync[0] <= ~IPL;
        ipl_sync[1] <= ipl_sync[0];

        if (ipl_sync[0] == ipl_sync[1])
            ipl <= ipl_sync[0];
    end
end

// Synchronize bus control signal inputs
(* async_reg = "true" *) reg reset_sync;
(* async_reg = "true" *) reg halt_sync;
(* async_reg = "true" *) reg dtack_sync;
(* async_reg = "true" *) reg berr_n_sync;

always @(posedge sys_clk) begin
    reset_sync <= nRESET_IN;
    halt_sync <= nHALT_IN;
    dtack_sync <= nDTACK;
    berr_n_sync <= nBERR;
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
wire drive_pi_data_out = !PI_RD && PI_WR;
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
reg r_lds_drive;
reg r_uds_drive;

// Disable all outputs for now
assign nVMA_OE = r_vma_drive;
assign nRESET_OE = r_reset_drive;
assign nHALT_OE = r_halt_drive;
assign nBGACK_OE = r_bgack_drive;
assign nBR_OE = r_br_drive;
assign nBG_OE = r_bg_drive;
assign nUDS_OE = r_uds_drive;
assign nLDS_OE = r_lds_drive;
assign nAS_OE = r_as_drive;
assign RnW_OE = r_rw_drive;

// Synchronize clk_rising/clk_falling with MC_CLK.
(* async_reg = "true" *) reg [1:0] mc_clk_sync;

wire mc_clk_rising = (mc_clk_sync == 2'b01);
wire mc_clk_falling = (mc_clk_sync == 2'b10);

always @(negedge sys_clk) begin
    mc_clk_sync <= {mc_clk_sync[0], CLK_7M};
end

// Phase counter clocks sys_clk pulses since rising edge of 7.14MHz MC CLK
// At 200MHz clock it could count up to 28 (28 * 7.14), so make sure it is large
// enough
(* syn_keep = "true" *) reg [4:0] phase_counter;

always @(posedge sys_clk) begin
    if (mc_clk_rising)
        phase_counter <= 4'd0;
    else
        phase_counter <= phase_counter + 4'd1;
end


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
wire [15:0] pi_status = {8'd0, req_active, req_terminated_normally, ipl, halt_sync, reset_sync, is_bm};

reg [2:0] req_fc;
reg req_rw;
reg [1:0] req_size;
reg req_terminated_normally;
reg is_bm;
reg [31:0] req_data_read;
reg [31:0] req_data_write;
reg [23:0] req_address;

always @(*) begin
    case (PI_A)
        PI_REG_DATA_LO: pi_data_out <= req_data_read[15:0];
        PI_REG_DATA_HI: pi_data_out <= req_data_read[31:16];
        PI_REG_ADDR_LO: pi_data_out <= req_address[15:0];
        PI_REG_ADDR_HI: pi_data_out <= {2'd0, req_fc, req_rw, req_size, req_address[23:16]};
        PI_REG_STATUS: pi_data_out <= pi_status;
        PI_REG_VERSION: pi_data_out <= firmware_version;
        default: pi_data_out <= 16'bx;
    endcase
end

// Synchronize WR command from Pi
(* async_reg = "true" *) reg [1:0] pi_wr_sync;
wire pi_wr_falling = (pi_wr_sync == 2'b10);

always @(posedge sys_clk) begin
    pi_wr_sync <= { pi_wr_sync[0], PI_WR };
end

// ## Access state machine.
localparam [3:0] STATE_WAIT_ACTIVE_REQUEST = 4'd0;
localparam [3:0] STATE_WAIT_BUS_CYCLE_START = 4'd1;
localparam [3:0] STATE_WAIT_ASSERT_AS = 4'd2;
localparam [3:0] STATE_WAIT_OPEN_DATA_LATCH = 4'd3;
localparam [3:0] STATE_WAIT_TERMINATION = 4'd4;
localparam [3:0] STATE_WAIT_LATCH_DATA = 4'd5;
localparam [3:0] STATE_UPDATE_DATA_READ = 4'd6;
localparam [3:0] STATE_MAYBE_TERMINATE_ACCESS = 4'd7;
localparam [3:0] STATE_RESET = 4'd15;

reg [3:0] state;
reg [6:0] reset_delay;

// Main state machine
always @(posedge sys_clk) begin
    if (pi_wr_falling) begin
        case (PI_A)
            PI_REG_DATA_LO: req_data_write[15:0] <= pi_data_in;
            PI_REG_DATA_HI: req_data_write[31:16] <= pi_data_in;
            PI_REG_ADDR_LO: req_address[15:0] <= pi_data_in;
            PI_REG_ADDR_HI: begin
                req_address[23:16] <= pi_data_in[7:0];
                req_size <= pi_data_in[9:8];
                req_rw <= pi_data_in[10];
                req_fc <= pi_data_in[13:11];
                req_active <= 1'b1;
            end
            PI_REG_CONTROL: begin
                if (pi_data_in[15]) begin
                    pi_control <= pi_control | pi_data_in[14:0];
                    if (pi_data_in[1] == 'b1) begin
                        req_active <= 1'b1;
                        state <= STATE_RESET;
                        reset_delay <= 7'd0;
                    end
                end else
                    pi_control <= pi_control & ~pi_data_in[14:0];
            end
        endcase
    end
    
    case (state)
        STATE_RESET: begin
            if (mc_clk_rising) begin
                if (reset_delay == 7'd124) begin
                    state <= STATE_WAIT_ACTIVE_REQUEST;
                    pi_control[1] <= 1'b0;
                    req_active <= 1'b0;
                end
                else
                    reset_delay <= reset_delay + 7'd1;
            end
        end
    endcase
end

endmodule
