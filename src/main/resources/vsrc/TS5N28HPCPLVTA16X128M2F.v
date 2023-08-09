module TS5N28HPCPLVTA16X128M2F(
    Q, CLKW, CLKR, REB, WEB, AA, AB, D
);
parameter Bits = 128;
parameter Word_Depth = 16;
parameter Add_Width = 4;

output  reg [Bits-1:0]      Q;
input                   CLKW;
input                   CLKR;
input                   REB;
input                   WEB;
input   [Add_Width-1:0] AA;
input   [Add_Width-1:0] AB;
input   [Bits-1:0]      D;

reg [Bits-1:0] ram [0:Word_Depth-1];
always @(posedge CLKW) begin
    if(!WEB) begin
        ram[AA] <= D;
    end
end

always @(posedge CLKW) begin
    Q <= !REB ? ram[AB] : {4{$random}};
end

endmodule
