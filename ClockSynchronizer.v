// Copyright © 2024 Michal Schulz <michal.schulz@gmx.de>
// https://github.com/michalsc
//
// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

module ClockSync
#(
    parameter DTACK_DELAY = 20
)
(
    input wire SYSCLK,
    input wire DTACK,
    input wire MCCLK,
    output reg MCCLK_RISING,
    output reg MCCLK_FALLING,
    output reg DTACK_LATCH,
    output reg DTACK_AFTER_LATCH
);

integer i;

(* async_reg = "true" *) reg [1:0] mc_clk_long;
reg dtack_delay_line [0:DTACK_DELAY];

always @(negedge SYSCLK)
begin
    mc_clk_long <= { mc_clk_long[0], MCCLK };
    //dtack_delay_line <= {dtack_delay_line[DTACK_DELAY-1:0], DTACK};
    
    case (mc_clk_long)
        2'b01: MCCLK_RISING <= 1'b1;
        2'b10: MCCLK_FALLING <= 1'b1;
        default: begin
            MCCLK_RISING <= 1'b0;
            MCCLK_FALLING <= 1'b0;
        end
    endcase
    
    dtack_delay_line[0] <= DTACK;
    for (i = 1; i < DTACK_DELAY; i = i + 1) begin
        dtack_delay_line[i] <= dtack_delay_line[i-1];
    end
    
    if (dtack_delay_line[DTACK_DELAY-2] && !dtack_delay_line[DTACK_DELAY-3])
        DTACK_LATCH <= 1'b1;
    else
        DTACK_LATCH <= 1'b0;
        
    if (dtack_delay_line[DTACK_DELAY-1] && !dtack_delay_line[DTACK_DELAY-2])
        DTACK_AFTER_LATCH <= 1'b1;
    else
        DTACK_AFTER_LATCH <= 1'b0;
    
    /*
    if (dtack_delay_line[DTACK_DELAY-1:DTACK_DELAY-2] == 2'b10)
        DTACK_LATCH <= 1'b1;
    else
        DTACK_LATCH <= 1'b0;
    
    if (dtack_delay_line[DTACK_DELAY:DTACK_DELAY-1] == 2'b10)
        DTACK_AFTER_LATCH <= 1'b1;
    else
        DTACK_AFTER_LATCH <= 1'b0;
    */
end

endmodule
