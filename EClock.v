// Copyright © 2024 Michal Schulz <michal.schulz@gmx.de>
// https://github.com/michalsc
//
// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
//   
//
// EClock module
// =============
// Purpose is to generate 714kHz clock from the source (7.14 MHz) by dividing the clock by factor of 10.
// The EClock does not have any constraint on synchronization with master CPU clock of any kind. It can be
// just whatever.
//
// The EClock is not used on A600

`default_nettype none
 
module EClock(
    input wire CLOCK_IN,
    output wire ECLOCK_OUT
);

reg [3:0] cnt = 0;
reg out;


assign ECLOCK_OUT = out;

always @(posedge CLOCK_IN) begin
    if (cnt == 4'd9)
        cnt <= 4'd0;
    else
        cnt <= cnt + 4'd1;

    // EClock is six CPU clocks low, 4 CPU clocks high
    if (cnt > 4'd5)
        out <= 1;
    else
        out <= 0;
end

endmodule