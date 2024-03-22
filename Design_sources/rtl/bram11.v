// bram behavior code (can't be synthesis)
// 11 words(32bits)
module bram11
(
    input               CLK,    
    input   [3:0]       WE,     //write enable  
                                //0000: No write, 0001: the LSB 8bits, ... 1111: whole word, 
    input               EN,     //EN: bram enable
    input   [31:0]      Di,     //din
    output  [31:0]      Do,     //dout
    input   [11:0]      A       //bytes address 
                                //12'b0000_00xx_xx00 : xxxx represent the number words of the RAM.
                                //12'b0000_0010_1000 : access 1010(10) words
);

    //  11 words
    reg [31:0] RAM[0:10];
    reg [11:0] r_A;


    // **Do is combinational logic, so you have to use FF to makesure that data won't change before the clk sampleing.
    always @(posedge CLK) begin
        r_A <= A>>2;
    end

    assign Do = {32{EN}} & RAM[r_A];    // read

    // **RAM is driven by clk, so you dont need to use FF on wire A, the clk will ensure RAM gets the right wire A.
    always @(posedge CLK) begin
        if(EN) begin
	        if(WE[0]) RAM[A>>2][7:0] <= Di[7:0];
            if(WE[1]) RAM[A>>2][15:8] <= Di[15:8];
            if(WE[2]) RAM[A>>2][23:16] <= Di[23:16];
            if(WE[3]) RAM[A>>2][31:24] <= Di[31:24];
        end
    end

endmodule
