// Copyright © 2024 Michal Schulz <michal.schulz@gmx.de>
// https://github.com/michalsc
//
// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

module ClockSync
#(
    parameter DTACK_DELAY = 14
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

(* async_reg = "true" *) reg [2:0] mc_clk_long;
reg [DTACK_DELAY:0] dtack_delay_line;

always @(negedge SYSCLK)
begin
    mc_clk_long <= { mc_clk_long[1:0], MCCLK };
    dtack_delay_line <= {dtack_delay_line[DTACK_DELAY-1:0], DTACK};
    
    case (mc_clk_long) // synthesis full_case
        3'b001: MCCLK_RISING <= 1'b1;
        3'b110: MCCLK_FALLING <= 1'b1;
        default: begin
            MCCLK_RISING <= 1'b0;
            MCCLK_FALLING <= 1'b0;
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

endmodule
