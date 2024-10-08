module RasPi(
    // PI bus
    input wire [27:0]   PI_GPIO_IN,
    output wire [27:0]  PI_GPIO_OUT,
    output wire [27:0]  PI_GPIO_OE,
    
    // 68k bus
    input wire [2:0] IPL,
    input wire HALT,
    input wire MASTER,
    input wire RESET,
    
    output wire DRIVE_RESET,
    output wire DRIVE_HALT,
    output wire DRIVE_BR,
    output wire [7:0] DTACK_DELAY,
    
    input wire SYSCLK,
    input wire CLEAR_ACTIVE,
    input wire SECOND_CYCLE,
    input wire ERROR,
    
    input wire [31:0]   REQUEST_DATA_IN,
    output reg [31:0]   REQUEST_DATA_OUT,
    output reg [23:0]   REQUEST_ADDRESS,
    output reg [2:0]    REQUEST_FC,
    output reg [1:0]    REQUEST_SIZE,
    output reg          REQUEST_IS_READ,
    output reg          REQUEST_ACTIVE
);

`include "global.vh"

// Wire IPL line as inputs from 68k bus to PI
assign PI_GPIO_OE[2:0] = 3'b111;
assign PI_GPIO_OUT[2:0] = IPL;

// req_active is on when transfer is in progress. It is always exposed to GPIO3
assign PI_GPIO_OUT[3] = REQUEST_ACTIVE;
assign PI_GPIO_OE[3] = 1'b1;

// reset request on GPIO4
assign PI_GPIO_OUT[4] = RESET;
assign PI_GPIO_OE[4] = 'b1;

// RD or WR commands from Pi to FPGA, always input
wire PI_RD;
wire PI_WR;
assign PI_RD = PI_GPIO_IN[6];
assign PI_WR = PI_GPIO_IN[7];
assign PI_GPIO_OE[7:6] = 2'b00;

// Data out from FPGA to Pi, driveon on read requests only (WR=1, RD=0)
reg [15:0] pi_data_out;
wire drive_pi_data_out = ~PI_RD & PI_WR;
assign PI_GPIO_OUT[23:8] = pi_data_out;
assign PI_GPIO_OE[23:8] = {16{drive_pi_data_out}};

// Data from Pi to FPGA is read on the same port inputs
wire [15:0] pi_data_in;
assign pi_data_in = PI_GPIO_IN[23:8];

// Address from Pi to FPGA, always input
wire [2:0] PI_A;
assign PI_A = PI_GPIO_IN[26:24];
assign PI_GPIO_OE[26:24] = 3'b000;

reg [2:0] ipl_sync_a;
reg [2:0] ipl_sync_b;

always @(negedge SYSCLK) begin
    ipl_sync_b <= ipl_sync_a;
    ipl_sync_a <= IPL;
end

// Firmware version
wire [15:0] firmware_version = { FW_MAJOR, FW_MINOR, FW_TYPE_PS16, FW_EXT_DATA };
wire [15:0] pi_status = {7'd0, SECOND_CYCLE, REQUEST_ACTIVE, ~ERROR, ipl_sync_b, HALT, RESET, MASTER};

always @(*) begin
    case (PI_A)
        PI_REG_DATA_LO: pi_data_out = REQUEST_DATA_IN[15:0];
        PI_REG_DATA_HI: pi_data_out = REQUEST_DATA_IN[31:16];
        PI_REG_ADDR_LO: pi_data_out = REQUEST_ADDRESS[15:0];
        PI_REG_ADDR_HI: pi_data_out = {2'd0, REQUEST_FC, REQUEST_IS_READ, REQUEST_SIZE, REQUEST_ADDRESS[23:16]};
        PI_REG_STATUS:  pi_data_out = pi_status;
        PI_REG_VERSION: pi_data_out = firmware_version;
        default:        pi_data_out = 16'bx;
    endcase
end

reg [14:0] pi_control = 15'b00000000000000;
assign DRIVE_BR = pi_control[0];
assign DRIVE_RESET = pi_control[1];
assign DRIVE_HALT = pi_control[2];
assign DTACK_DELAY = {1'b0, pi_control[14:8]};

// Synchronize WR command from Pi
reg pi_wr_a;
reg pi_wr_b;

always @(negedge SYSCLK) begin       
    pi_wr_a <= PI_WR;
    pi_wr_b <= pi_wr_a;
    
    if (pi_wr_b & ~pi_wr_a) begin
        case (PI_A) // synthesis full_case
            PI_REG_DATA_LO: REQUEST_DATA_OUT[15:0]  <= pi_data_in;
            PI_REG_DATA_HI: REQUEST_DATA_OUT[31:16] <= pi_data_in;
            PI_REG_ADDR_LO: REQUEST_ADDRESS[15:0]   <= pi_data_in;
            PI_REG_ADDR_HI: begin
                REQUEST_ADDRESS[23:16]  <= pi_data_in[7:0];
                REQUEST_SIZE            <= pi_data_in[9:8];
                REQUEST_IS_READ         <= pi_data_in[10];
                REQUEST_FC              <= pi_data_in[13:11];
                REQUEST_ACTIVE          <= 1'b1;
            end
            PI_REG_CONTROL: begin
                if (pi_data_in[15]) begin
                    pi_control <= pi_control | pi_data_in[14:0];                    
                end else
                    pi_control <= pi_control & ~pi_data_in[14:0];
            end
        endcase
    end else begin
        REQUEST_ACTIVE <= REQUEST_ACTIVE & ~CLEAR_ACTIVE;
    end
end

endmodule
