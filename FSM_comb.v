module FSMComb(
    input wire ACTIVATE,
    input wire LATCH,
    input wire MUST_CONTINUE,
    input wire MC_CLK_RISING,
    input wire AS_FEEDBACK,
    input wire [3:0] CURRENT,
    output reg [3:0] NEXT
);

`include "global.vh"

always @(*) begin
    case (CURRENT)
        STATE_WAIT:
            if (ACTIVATE)           NEXT = STATE_ACTIVATE;
            else                    NEXT = STATE_WAIT;
        
        STATE_ACTIVATE:             NEXT = STATE_SETUP_BUS;
        
        STATE_SETUP_BUS:            NEXT = STATE_DRIVE_AS;
        
        STATE_DRIVE_AS: 
            if (AS_FEEDBACK)        NEXT = STATE_DRIVE_DS;
            else                    NEXT = STATE_DRIVE_AS;
            
        STATE_DRIVE_DS:             NEXT = STATE_WAIT_DSACK;
        
        STATE_WAIT_DSACK: 
            if (LATCH)              NEXT = STATE_LATCH;
            else                    NEXT = STATE_WAIT_DSACK;
        
        STATE_LATCH:                NEXT = STATE_CLEAR_AS;
        
        STATE_CLEAR_AS:             NEXT = STATE_ON_DSACK;
        
        STATE_ON_DSACK:
            if (~AS_FEEDBACK & MC_CLK_RISING)
                                    NEXT = STATE_FINALIZE;
            else                    NEXT = STATE_ON_DSACK;
        
        STATE_FINALIZE:
            if (MUST_CONTINUE)      NEXT = STATE_CONTINUE;
            else                    NEXT = STATE_WAIT;
                
        STATE_CONTINUE:             NEXT = STATE_SETUP_BUS;

        default:                    NEXT = STATE_WAIT;
    endcase
end

endmodule
