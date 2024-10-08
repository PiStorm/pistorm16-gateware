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
