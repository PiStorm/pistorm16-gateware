// Copyright © 2024 Michal Schulz <michal.schulz@gmx.de>
// https://github.com/michalsc
//
// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
//
// ClockSync module
// ================
// Clock synchronizer provides falling and rising strobes from low-speed 7.14 MHz clock.
// Further, it provides time-shifted signal from nDTACK bus of Amiga. This latch is used
// to control the main FSM.
// The reason of the above is partially screwed timing of A500/A600. There, the nDTACK
// appears somewhere in the middle of S4 state. According to m68k user manual, the data on
// the bus is then latched on falling clock edge of S6->S7 transition. This is fine but
// it could happen already earlier. The manual says that device shall take not longer than 90ns
// to stabilize data bus after asserting DTACK. Amiga does not.
// The delayed DTACK, either hardcoded as a constant or provided from a register is used to 
// fine-tune the delay so that Firmware latches data as soon as possible but not before the
// data is stable. Use with care.

module ClockSync
/*#(
    parameter DTACK_DELAY = 15
)*/
(
    input wire SYSCLK,
    input wire DTACK,
    input wire MCCLK,
    input wire [7:0] DTACK_DELAY,
    output reg MCCLK_FALLING,
    output reg MCCLK_RISING,
    output reg DTACK_LATCH,
    output reg DTACK_LATCH_WRITE
);

reg [1:0] mc_clk_long;
reg [7:0] cnt;

always @(negedge SYSCLK)
begin
    mc_clk_long <= { mc_clk_long[0], MCCLK };
    
    MCCLK_FALLING <= (mc_clk_long == 2'b10) ? 1'b1 : 1'b0;
    MCCLK_RISING <= (mc_clk_long == 2'b01) ? 1'b1 : 1'b0;
    
    if (DTACK) begin
        cnt <= 'd0;
        DTACK_LATCH <= 1'b0;
    end
    else begin
        if (cnt < DTACK_DELAY) cnt <= cnt + 'd1;
        else DTACK_LATCH <= 1'b1;
    end 
end

endmodule
