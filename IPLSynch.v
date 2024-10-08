// Copyright © 2024 Michal Schulz <michal.schulz@gmx.de>
// https://github.com/michalsc
//
// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
//
//
// IPLSynch module
// ===============
// Interrupt level synchronizer. The purpose of this module is to latch the IPL
// on falling edges of input clock (7.14MHz usually). If two subsequent latches give
// the same value then it is considered valid and output IPL register is set to that
// value. This behavior follows behavior of 68000 CPU.

module IPLSynch (
    input wire CLK,
    input wire [2:0] IPL_ASYNC,
    output reg [2:0] IPL
);

(* async_reg = "true" *) reg [2:0] ipl_sync [1:0];

always @(posedge CLK) begin
    ipl_sync[1] <= ipl_sync[0];
    ipl_sync[0] <= IPL_ASYNC;
    
    if (ipl_sync[1] == ipl_sync[0]) begin
        IPL <= ipl_sync[0];
    end
end

endmodule
