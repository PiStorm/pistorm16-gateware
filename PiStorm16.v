module PiStorm16(
    // Amiga clock
    input CLK_7M,
    
    // Amiga address bus
    output [23:1] A_OUT,
    output [23:1] A_OE,
    
    // Amiga data bus
    input [15:0] D_IN,
    output [15:0] D_OUT,
    output [15:0] D_OE,
    
    // Read/Write
    output RnW_OUT,
    output RnW_OE,
    
    // Address strobe
    output nAS_OUT,
    output nAS_OE,
    
    // Lower/Upper byte on D active
    output nLDS_OUT,
    output nLDS_OE,
    output nUDS_OUT,
    output nUDS_OE,
    
    input nDTACK,
    input nBERR,
    
    input nBG_IN,
    output nBG_OUT,
    output nBG_OE,
    
    input nBR_IN,
    output nBR_OUT,
    output nBR_OE,
    
    input nBGACK_IN,
    output nBGACK_OUT,
    output nBGACK_OE,
    
    input nHALT_IN,
    output nHALT_OUT,
    output nHALT_OE,
    
    input nRESET_IN,
    output nRESET_OE,
    
    input nVPA,
    input nVMA_IN,
    output nVMA_OUT,
    output nVMA_OE,
    
    // IPL
    input [2:0] IPL,
    
    // FC2..0
    output [2:0] FC_OUT,
    output [2:0] FC_OE,
    
    // Pi interface
    input [27:0] PI_GPIO_IN,
    output [27:0] PI_GPIO_OUT,
    output [27:0] PI_GPIO_OE,
    
    // Debug port
    output DBG_DAT,
    output DBG_CLK,
    
    // Test pads
    input [8:1] TP_IN,
    output [8:1] TP_OUT,
    output [8:1] TP_OE,
    
    // PLL
    input SYS_PLL_CLKOUT0,
    input IN_CLK_50M,
    input SYS_PLL_LOCKED
);

assign DBG_DAT = PI_GPIO_IN[5];
assign DBG_CLK = PI_GPIO_IN[27];

assign PI_GPIO_OE[5] = 0;
assign PI_GPIO_OE[27] = 0;

wire SYS_CLK = SYS_PLL_CLKOUT0;
wire MC_CLK = CLK_7M;

// Registers used to drive entire buses at once
reg FC_DRIVE = 0;
reg A_DRIVE = 0;
reg D_DRIVE = 0;

assign FC_OE = {3{FC_DRIVE}};
assign D_OE = {16{D_DRIVE}};
assign A_OE = {23{A_DRIVE}};

assign TP_OE = 8'b0;

reg RESET_DRIVE = 0;

// Disable all outputs for now
assign nVMA_OE = 0;
assign nRESET_OE = 0;
assign nHALT_OE = 0;
assign nBGACK_OE = 0;
assign nBR_OE = 0;
assign nBG_OE = 0;
assign nUDS_OE = 0;
assign nLDS_OE = 0;
assign nAS_OE = 0;
assign RnW_OE = 0;

reg[31:0] counter;

always @(posedge SYS_CLK) begin
    if (counter < 32'd7000000) begin
        counter <= counter + 32'd1;
    end else begin 
        RESET_DRIVE <= ~RESET_DRIVE;
        counter <= 0;
    end
end

// Synchronize clk_rising/clk_falling with MC_CLK.
(* async_reg = "true" *) reg [1:0] mc_clk_sync;

always @(negedge SYS_CLK) begin
    mc_clk_sync <= {mc_clk_sync[0], MC_CLK};
end

reg [3:0] phase_counter;

always @(posedge SYS_CLK) begin
    if (mc_clk_sync == 2'b01)
        phase_counter <= 4'd0;
    else
        phase_counter <= phase_counter + 4'd1;
end

endmodule
