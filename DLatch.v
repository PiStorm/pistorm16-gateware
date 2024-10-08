// Copyright © 2024 Michal Schulz <michal.schulz@gmx.de>
// https://github.com/michalsc
//
// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
//
//
// Latches module
// ==============
// The module provides several latches driven by high-speed clock domain and
// synchronized on the rising/falling edges of low-speed clock. Used to keep
// proper timings of Amiga-chipset relevant signals: AS, LDS, UDS, RnW

module DLatch(
    input wire SET,
    input wire RESET,
    input wire CLK,
    output reg OUT
);

always @(*) begin
    if (RESET == 1)
        OUT <= 1'b0;
    else if (SET == 1)
        OUT <= 1'b1;
end

endmodule


module FFLatch(
    input wire SET,
    input wire RESET,
    input wire CLK,
    output reg OUT
);

reg clear;
wire clocked_reset = RESET & clear;

always @(posedge CLK or posedge clocked_reset) begin
    if (clocked_reset)
        OUT <= 1'b0;
    else if (CLK & SET)
        OUT <= 1'b1;
end

always @(negedge CLK) begin
    if (RESET)
        clear <= 1'b1;
    else
        clear <= 1'b0; 
end

endmodule

module FFLatchN(
    input wire SET,
    input wire RESET,
    input wire CLK,
    output reg OUT
);

reg clear;
wire clocked_reset = RESET & clear;

always @(posedge CLK or posedge clocked_reset) begin
    if (clocked_reset)
        OUT <= 1'b1;
    else if (CLK & SET)
        OUT <= 1'b0;
end

always @(negedge CLK) begin
    if (RESET)
        clear <= 1'b1;
    else
        clear <= 1'b0; 
end

endmodule


module FFLatchPR(
    input wire SET,
    input wire RESET,
    input wire CLK,
    output reg OUT
);

always @(posedge CLK) begin
    if (SET)
        OUT <= 1'b1;
    else if (RESET)
        OUT <= 1'b0;
end

endmodule


module FFLatchNPR(
    input wire SET,
    input wire RESET,
    input wire CLK,
    output reg OUT
);

always @(posedge CLK) begin
    if (SET)
        OUT <= 1'b0;
    else if (RESET)
        OUT <= 1'b1;
end

endmodule

