`timescale 1ns/1ps

module Jericalla_Evolution_TB;

    reg [16:0] instruccion;
    reg Clk;
    wire [31:0] DataOut;

    Jericalla_Evolution DUT (
        .instruction(instruccion),
        .CLK(Clk),
        .DS(DataOut)
    );

    always #10 Clk = ~Clk;

    initial begin
        Clk = 0;
        instruccion = 17'b10000110010000000;
        #20;

        instruccion = 17'b01001010000100010;
        #20;

        instruccion = 17'b10001100001000011;
        #20;

        instruccion = 17'b11000000011100100;
        #20;

        instruccion = 17'b11000000100000101;
        #20;

        instruccion = 17'b11000000100100110;
        #20;
        $stop;
    end

endmodule
