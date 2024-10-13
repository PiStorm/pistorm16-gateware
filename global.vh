// ## Access state machine.

localparam STATE_WAIT       = 4'b0000;
localparam STATE_ACTIVATE   = 4'b0001;
localparam STATE_SETUP_BUS  = 4'b0011;
localparam STATE_DRIVE_AS   = 4'b0010;
localparam STATE_DRIVE_DS   = 4'b0110;
localparam STATE_WAIT_DSACK = 4'b0111;
localparam STATE_LATCH      = 4'b0101;
localparam STATE_CLEAR_AS   = 4'b0100;
localparam STATE_ON_DSACK   = 4'b1100;
localparam STATE_FINALIZE   = 4'b1101;
localparam STATE_CONTINUE   = 4'b1111;
  
// ## Pi interface.
localparam [2:0] PI_REG_DATA_LO = 3'd0;
localparam [2:0] PI_REG_DATA_HI = 3'd1;
localparam [2:0] PI_REG_ADDR_LO = 3'd2;
localparam [2:0] PI_REG_ADDR_HI = 3'd3;
localparam [2:0] PI_REG_STATUS = 3'd4;
localparam [2:0] PI_REG_CONTROL = 3'd4;
localparam [2:0] PI_REG_FREQUENCY = 3'd6;
localparam [2:0] PI_REG_VERSION = 3'd7;
