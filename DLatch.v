// Copyright © 2024 Michal Schulz <michal.schulz@gmx.de>
// https://github.com/michalsc
//
// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

module DLatch(
    input wire SET,
    input wire RESET,
    output reg OUT
);

always @(*) begin
    if (RESET == 1)
        OUT <= 1'b0;
    else if (SET == 1)
        OUT <= 1'b1;
end

endmodule
