
module Jericalla_Evolution(
    input [16:0] instruction,
    input CLK,
    output reg [31:0] DS
);

wire cDemux, cWe, cRe, cZf;
wire [1:0] cSel;
wire B1_WE_OUT, B2_WE_OUT;
wire B1_RE_OUT, B2_RE_OUT;
wire [31:0] buffer1_D1_In, buffer1_D2_In, buffer1_D1_Out, buffer1_D2_Out;
wire [31:0] cDemuxData;
wire [31:0] buffer2_D1_In, buffer2_D2_In, buffer2_D1_Out, buffer2_D2_Out, buffer2_DD_Out;


Controller Control(//Inputs
    .Op(instruction[16:15]), 
    //Ouputs
    .Demuxo(cDemux), 
    .WeMD(cWe), 
    .ReMD(cRe),
    .Sel(cSel)
);
BancRegister Banco( //Inputs
    .RA1(instruction[9:5]), 
    .RA2(instruction[4:0]), 
    .WA(instruction[14:10]),
    //Outputs
    .data1Out(buffer1_D1_In), 
    .data2Out(buffer1_D2_In)
);
Buffer1 BufferOne( //Inputs
    .clk(CLK),
    .WriteEnableIn(cWe),
    .ReadEnIn(cRe),
    .data1In(buffer1_D1_In),
    .data2In(buffer1_D2_In),
    //Outputs 
    .WriteEnableOut(B1_WE_OUT),
    .ReadEnOut(B1_RE_OUT),
    .data1Out(buffer1_D1_Out),
    .data2Out(buffer1_D2_Out)
);
Demuxo Demux( //Inputs
    .dmx(cDemux),
    .dataIn(buffer1_D1_Out),
    //Outputs 
    .outAlu(cDemuxData),
    .outMem(cDemuxData)
);
ALU Alulu(
    .A(cDemuxData),
    .B(buffer1_D2_Out),
    .ALU_Sel(cSel),
    //Ouputs 
    .R(buffer2_D2_In),
    .Zero_Flag(cZf)
);

Buffer2 BufferTwo(
    .clk(CLK),
    .WriteEnableIn(B1_WE_OUT),
    .ReadEnIn(B1_RE_OUT),
    .data1In(buffer2_D2_In),
    .data2In(buffer1_D2_Out),
    .dataDemux(cDemuxData),
    //Outputs 
    .WriteEnableOut(B2_WE_OUT),
    .ReadEnOut(B2_RE_OUT),
    .data1Out(buffer2_D1_Out),
    .data2Out(buffer2_D2_Out),
    .dataDOut(buffer2_DD_Out)
);
MemoryData MemData(
    .WEn(B2_WE_OUT),
    .REn(B2_RE_OUT),
    .dataIn(buffer2_D2_Out),
    .dir(cDemuxData),
    .dataOut(DS)
);



endmodule

//Controlador
module Controller(
    input [1:0] Op,
    output reg Demuxo, WeMD, ReMD,
    output reg [3:0] Sel
);

always @(*) begin
    case (Op)
        2'b00: begin       
            Sel = 4'b0010;
            WeMD = 1'b0;   
            ReMD = 1'b0;
        end
        2'b01: begin
            Sel = 4'b0110;    
            WeMD = 1'b0;   
            ReMD = 1'b0;
        end
        2'b10: begin
            Sel = 4'b0111;   
            WeMD = 1'b0;   
            ReMD = 1'b0;
        end
        2'b11: begin
            WeMD = 1'b1;
            ReMD = 1'b1;
        end
    endcase
end
endmodule


//Banco de Registros
module BancRegister( 
    input [4:0] RA1, RA2, WA,
    input [31:0] DW,
    input WE,
    output reg [31:0] data1Out, data2Out
);

reg [31:0] mem [0:31];

initial begin
    $readmemb("data", mem);
end

always @ * begin
    if(WE) begin
        mem[WA] = DW;
    end
    data1Out = mem[RA1];
    data2Out = mem[RA2];
end
endmodule

//Buffer ###
module Buffer1 (
    input clk, WriteEnableIn, ReadEnIn,
    input [31:0] data1In, data2In,
    output reg WriteEnableOut, ReadEnOut,
    output reg [31:0] data1Out, data2Out
);

    always @(posedge clk) begin
        WriteEnableOut <= WriteEnableIn;
        data1Out <= data1In;
        data2Out <= data1In;
    end
endmodule

//Buffer ###
module Buffer2 (
    input clk, WriteEnableIn, ReadEnIn,
    input [31:0] data1In, data2In, dataDemux,
    output reg WriteEnableOut, ReadEnOut,
    output reg [31:0] data1Out, data2Out, dataDOut
);

    always @(posedge clk) begin
        WriteEnableOut <= WriteEnableIn;
        data1Out <= data1In;
        data2Out <= data1In;
        dataDOut <= dataDemux;
    end
endmodule

//Demultiplexor Alu-Mem
module demuxo(
    input dmx,
    input [31:0] dataIn,
    output reg [31:0] outAlu, outMem    
);

always @(*) begin
    case (dmx)
        1'b0: outAlu = dataIn;               
        1'b1: outMem = dataIn;        
    endcase
end
endmodule

//Memoria de Datos
module MemoryData(
    input WEn, REn,
    input [31:0] dataIn, 
    input [3:0] dir,
    output reg [31:0] dataOut
);

reg [31:0] mem [0:127];

always @ * begin
    if(WEn) begin
        mem[dir] = dataIn;
    end
    if(REn) begin
        dataOut = mem[dir];
    end
end

endmodule

//Alu
module ALU (
    input [31:0] A,
    input [31:0] B,
    input [3:0] ALU_Sel,
    output reg [31:0] R,
    output reg Zero_Flag
);

always @(*) begin
    case (ALU_Sel)
        4'b0000: R = A & B;        
        4'b0001: R = A | B;        
        4'b0010: R = A + B;       
        4'b0110: R = A - B;        
        4'b0111: R = (A < B) ? 32'd1 : 32'd0;
        4'b1100: R = ~(A | B);        
        default: R = 32'd0;      
    endcase
    
    Zero_Flag = (R == 32'd0) ? 1'b1 : 1'b0;
end

endmodule
