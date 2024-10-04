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
    output reg MCCLK_RISING,
    output reg DTACK_LATCH
);

reg [1:0] mc_clk_long;
reg [DTACK_DELAY-1:0] dtack_delay_line;

always @(negedge SYSCLK)
begin
    mc_clk_long <= { mc_clk_long[0], MCCLK };
    
    MCCLK_FALLING <= (mc_clk_long == 2'b10) ? 1'b1 : 1'b0;
    MCCLK_RISING <= (mc_clk_long == 2'b01) ? 1'b1 : 1'b0;
    
    dtack_delay_line <= {dtack_delay_line[DTACK_DELAY-2:0], DTACK};
    
    DTACK_LATCH <= ~(dtack_delay_line[DTACK_DELAY-1] | DTACK);
end

endmodule
