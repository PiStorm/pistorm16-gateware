// Copyright © 2024 Michal Schulz <michal.schulz@gmx.de>
// https://github.com/michalsc
//
// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
//
//
// FSMComb module
// ==============
// Combinational part of main FSM, sweeping from one state to another and
// driving sequential part of FSM.

module FSMComb(
    input wire ACTIVATE,
    input wire LATCH,
    input wire MUST_CONTINUE,
    input wire MC_CLK_RISING,
    input wire AS_FEEDBACK,
    input wire ENTER_S6,
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
            if (ENTER_S6)           NEXT = STATE_WAIT_LATCH;
            else                    NEXT = STATE_WAIT_DSACK;
        
        STATE_WAIT_LATCH:
            if (LATCH)              NEXT = STATE_LATCH;
            else                    NEXT = STATE_WAIT_LATCH;
            
        STATE_LATCH:                NEXT = STATE_CLEAR_AS;
        
        STATE_CLEAR_AS:             NEXT = STATE_ON_DSACK;
        
        STATE_ON_DSACK:
            if (!AS_FEEDBACK && MC_CLK_RISING)
                                    NEXT = STATE_FINALIZE;
            else                    NEXT = STATE_ON_DSACK;
        
        STATE_FINALIZE:
            if (MUST_CONTINUE)      NEXT = STATE_SETUP_BUS;
            else                    NEXT = STATE_WAIT;
                
        default:                    NEXT = STATE_WAIT;
    endcase
end

endmodule
