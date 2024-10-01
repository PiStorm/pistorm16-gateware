// Copyright © 2024 Michal Schulz <michal.schulz@gmx.de>
// https://github.com/michalsc
//
// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

// FPGA clock    DTACK_DELAY
//   120 MHz          16


module ClockSync
#(
    parameter DTACK_DELAY = 15
)
(
    input wire SYSCLK,
    input wire DTACK,
    input wire MCCLK,
    output reg MCCLK_FALLING,
    output reg DTACK_LATCH
);

integer i;

(* async_reg = "true" *) reg [1:0] mc_clk_long;
reg dtack_delay_line [0:DTACK_DELAY];

always @(negedge SYSCLK)
begin
    mc_clk_long <= { mc_clk_long[0], MCCLK };
    //dtack_delay_line <= {dtack_delay_line[DTACK_DELAY-1:0], DTACK};
    
    if (mc_clk_long == 2'b10)
        MCCLK_FALLING <= 1'b1;
    else
        MCCLK_FALLING <= 1'b0;
    
    dtack_delay_line[0] <= DTACK;
    for (i = 1; i < DTACK_DELAY; i = i + 1) begin
        dtack_delay_line[i] <= dtack_delay_line[i-1];
    end
    
    if (dtack_delay_line[DTACK_DELAY-1] && !dtack_delay_line[DTACK_DELAY-2])
        DTACK_LATCH <= 1'b1;
    else
        DTACK_LATCH <= 1'b0;
end

endmodule
