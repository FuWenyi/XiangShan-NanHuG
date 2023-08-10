module TS6N28HPCPLVTA512X2M2F(
    Q, CLKW, CLKR, REB, WEB, AA, AB, D
);
parameter Bits = 2;
parameter Word_Depth = 512;
parameter Add_Width = 9;

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

always @(posedge CLKR) begin
    Q <= !REB ? ram[AB] : {4{$random}};
end

endmodule
