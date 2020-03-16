
`timescale 1ns / 1ps

module final(input clk, input [15:0] sw,output [15:0] led);

reg [15:0] inst;

reg [1:0] cycle;
wire[7:0] sum;
wire RF_Rp_zero;


//program counter values
reg PC_clr,PC_ld,PC_inc;
//instruction registers vaues
reg IR_ld;
wire [15:0] Instr;
//8 bit mux values
reg  D_addr_sel;
wire  [7:0] PC_addr;
reg [7:0] D_addr;
wire[7:0] muxOut8;
//D values
reg D_rd,D_wr;
wire[15:0] Rp_data;//shared with data register
wire[15:0] R_data;



assign  RF_Rp_zero=(Rp_data==0);
//program counter
PC pc(clk,PC_clr,PC_ld,PC_inc,sum,PC_addr);
//instruction register
IR ir(clk,IR_ld,R_data,Instr);
//mux
Bit8MuxSig mux1(D_addr_sel,PC_addr,D_addr,muxOut8);
//module D(input [7:0] addr,input rd, input wr,input[15:0] W_data, output  [15:0] R_data);
D memory(muxOut8,D_rd,D_wr,Rp_data,R_data,sw[7:0],led);
//adder
reg [7:0] PC_count;
//Adder adder(PC_count,Instr[7:0],sum);
Adder adder(PC_count,Instr[7:0],sum);
//alu
// module ALU(input [2:0] alu_s, input [15:0] A, B, output[15:0] data);
reg [2:0] alu_s;
wire[15:0] Rq_data,data;

//Bit16MuxSig
//Bit16MuxSig(input s1,s0,input [15:0] sig1,sig2, input [7:0] sig3, output [15:0] signal);
reg RF_s0,RF_s1;
wire [15:0]W_data;
reg [15:0] RF_W_data;
Bit16MuxSig mux2(RF_s1,RF_s0,data,R_data,RF_W_data,W_data);
//data register
/*
module dataReg(
input W_wr,Rp_rd,Rq_rd,input[3:0] W_addr,Rp_addr,Rq_addr,input [15:0] W_data,
output [15:0] Rp_data,Rq_data);
*/

reg W_wr,Rp_rd,Rq_rd;
reg [3:0] RF_W_addr,RF_Rp_addr,RF_Rq_addr;
dataReg reg16( W_wr,Rp_rd,Rq_rd,RF_W_addr,RF_Rp_addr,RF_Rq_addr, W_data,
 Rp_data,Rq_data);
 ALU al(inst[14:12],Rp_data,Rq_data,data);
wire decode_IR_ld;
wire decode_PC_ld;
wire decode_PC_inc;
wire decode_PC_clr;
wire decode_D_addr_sel;
wire decode_D_rd;
wire decode_D_wr;
wire decode_RF_s1;
wire decode_RF_s0;
wire decode_W_wr;
wire decode_rd;
wire decode_rd_Rp;
wire decode_rd_Rq;
wire [2:0] decode_alu_s;
wire [7:0] decode_D_addr;
wire [7:0] decode_RF_W_data;
wire [3:0] decode_RF_W_addr;
wire [3:0] decode_RF_Rp_addr;
wire [3:0] decode_RF_Rq_addr;

reg temp_IR_ld;
reg temp_PC_ld;
reg temp_PC_inc;
reg temp_PC_clr;
reg temp_D_addr_sel;
reg temp_D_rd;
reg temp_D_wr;
reg temp_RF_s1;
reg temp_RF_s0;
reg temp_W_wr;
reg temp_rd_Rp;
reg temp_rd_Rq;
reg [2:0] temp_alu_s;
reg [7:0] temp_D_addr;
reg [7:0] temp_RF_W_data;
reg [3:0] temp_RF_W_addr;
reg [3:0] temp_RF_Rp_addr;
reg [3:0] temp_RF_Rq_addr;
reg reset;

//decoder
//use to be Instr

always@(*)
begin
alu_s=Instr[14:12];
end

reg[7:0] imm,dir,offset;
reg[3:0] adress0,adress1,adress2;
reg fill;


initial begin
PC_clr=0;
PC_inc<=0;
IR_ld<=1;
D_rd=1;
D_addr_sel=0;
W_wr=0;
D_wr<=0;
end


always@(posedge clk or posedge reset)
begin
if(reset)//reset signal is sent
begin
//PC_clr=1;
//PC_inc<=0;
//counter set to zero
end
else
//PC_clr=0;//counter clear is disengaged
begin
    case(cycle)
    2'b00:begin cycle<=cycle+1;//fetch phase
    PC_inc<=0;//incrementing is stopped
    D_addr_sel=0;//selecting to read D adress from counter
    D_rd=1;//choosing to read
    D_wr=0;//and not write
    IR_ld<=1;//instruction from instruction register is loaded
    inst=Instr;
    PC_clr=0;
    PC_count=PC_addr;//take a note of where the program counter is now 
    adress2<=inst[11:8];
adress1<=inst[7:4];
adress0<=inst[3:0];

imm=inst[7:0];
dir=inst[7:0];
offset=inst[7:0];
alu_s=inst[14:12];
end
    2'b01:begin cycle<=cycle+1;//decode phase hold values from decoder in tempory registers
case(inst[15:12])
	4'b0000:begin
	fill=0;
	RF_W_addr<=adress2;	W_wr<=1;	RF_Rp_addr<=adress1;	Rp_rd<=1;	RF_Rq_addr<=adress0;	Rq_rd<=1;	RF_s1<=0;	RF_s0<=0;	
        RF_W_data<=16'h0000;    D_rd<=1'b0;    D_wr<=1'b0;    D_addr<=4'b0000;D_addr_sel<=0;    
        PC_ld<=0;    PC_clr<=0;    PC_inc<=0;    IR_ld<=0;
	end
	4'b0001:begin
	RF_W_addr<=adress2;	W_wr<=1;	RF_Rp_addr<=adress1;	Rp_rd<=1;	RF_Rq_addr<=adress0;	Rq_rd<=1;	RF_s1<=0;	RF_s0<=0;	
            RF_W_data<=16'h0000;    D_rd<=1'b0;    D_wr<=1'b0;    D_addr<=4'b0000;D_addr_sel<=0;    
            PC_ld<=0;    PC_clr<=0;    PC_inc<=0;    IR_ld<=0;
	fill=0;
	end
	4'b0010:begin
	fill=0;
	RF_W_addr<=adress2;	W_wr<=1;	RF_Rp_addr<=adress1;	Rp_rd<=1;	RF_Rq_addr<=adress0;	Rq_rd<=1;	RF_s1<=0;	RF_s0<=0;	
            RF_W_data<=16'h0000;    D_rd<=1'b0;    D_wr<=1'b0;    D_addr<=4'b0000;D_addr_sel<=0;    
            PC_ld<=0;    PC_clr<=0;    PC_inc<=0;    IR_ld<=0;
	end
	4'b0011:begin
	fill=0;
	RF_W_addr<=adress2;	W_wr<=1;	RF_Rp_addr<=adress1;	Rp_rd<=1;	RF_Rq_addr<=adress0;	Rq_rd<=1;	RF_s1<=0;	RF_s0<=0;	
            RF_W_data<=16'h0000;    D_rd<=1'b0;    D_wr<=1'b0;    D_addr<=4'b0000;D_addr_sel<=0;    
            PC_ld<=0;    PC_clr<=0;    PC_inc<=0;    IR_ld<=0;
	end
	4'b0100:begin
	fill=0;
	RF_W_addr<=adress2;	W_wr<=1;	RF_Rp_addr<=adress1;	Rp_rd<=1;	RF_Rq_addr<=adress0;	Rq_rd<=1;	RF_s1<=0;	RF_s0<=0;	
            RF_W_data<=16'h0000;    D_rd<=1'b0;    D_wr<=1'b0;    D_addr<=4'b0000;D_addr_sel<=0;    
            PC_ld<=0;    PC_clr<=0;    PC_inc<=0;    IR_ld<=0;
	end
	4'b0101:begin
	fill=0;
	RF_W_addr<=adress2;	W_wr<=1;	RF_Rp_addr<=adress1;	Rp_rd<=1;	RF_Rq_addr<=adress0;	Rq_rd<=1;	RF_s1<=0;	RF_s0<=0;	
            RF_W_data<=16'h0000;    D_rd<=1'b0;    D_wr<=1'b0;    D_addr<=4'b0000;D_addr_sel<=0;    
            PC_ld<=0;    PC_clr<=0;    PC_inc<=0;    IR_ld<=0;
	end
	4'b0110:begin
	fill=0;
	RF_W_addr<=adress2;	W_wr<=1;	RF_Rp_addr<=adress1;	Rp_rd<=1;	RF_Rq_addr<=adress0;	Rq_rd<=1;	RF_s1<=0;	RF_s0<=0;	
            RF_W_data<=16'h0000;    D_rd<=1'b0;    D_wr<=1'b0;    D_addr<=4'b0000;D_addr_sel<=0;    
            PC_ld<=0;    PC_clr<=0;    PC_inc<=0;    IR_ld<=0;
	end
	4'b0111:begin
	fill=0;
	RF_W_addr<=adress2;	W_wr<=1;	RF_Rp_addr<=adress1;	Rp_rd<=1;	RF_Rq_addr<=adress0;	Rq_rd<=1;	RF_s1<=0;	RF_s0<=0;	
            RF_W_data<=16'h0000;    D_rd<=1'b0;    D_wr<=1'b0;    D_addr<=4'b0000;D_addr_sel<=0;    
            PC_ld<=0;    PC_clr<=0;    PC_inc<=0;    IR_ld<=0;
	end
	4'b1000:begin
	RF_W_addr<=adress2;	W_wr<=1;	RF_Rp_addr<=adress1;	Rp_rd<=0;	RF_Rq_addr<=adress0;	Rq_rd<=0;	RF_s1<=1;	RF_s0<=0;	
	RF_W_data<=imm;	D_rd<=1'b0;	D_wr<=1'b0;	D_addr<=4'b0000;D_addr_sel<=0;	
	PC_ld<=0;	PC_clr<=0;	PC_inc<=0;	IR_ld<=0;
	end
	4'b1001:begin//load word from memoery to register
	fill=0;
	RF_W_addr<=adress2;	W_wr<=1;	RF_Rp_addr<=0;	Rp_rd<=0;	RF_Rq_addr<=adress0;	Rq_rd<=0;	RF_s1<=0;	RF_s0<=1;	
	RF_W_data<=0;	D_rd<=1'b1;	D_wr<=1'b0;	D_addr<=dir;D_addr_sel<=1;	
	PC_ld<=0;	PC_clr<=0;	PC_inc<=0;	IR_ld<=0;
	end
	4'b1010:begin//store word from data register into permanent memory 
	fill=0;
	RF_W_addr<=0;	W_wr<=0;	RF_Rp_addr<=adress2;	Rp_rd<=1;	RF_Rq_addr<=adress0;	Rq_rd<=0;	RF_s1<=1;	RF_s0<=0;	
	RF_W_data<=0;	D_rd<=1'b0;	D_wr<=1'b1;	D_addr<=dir;D_addr_sel<=1;	
	PC_ld<=0;	PC_clr<=0;	PC_inc<=0;	IR_ld<=0;
	end
	4'b1011:begin//biz
	fill=0;
	   if(RF_Rp_zero)
	   begin
	       RF_W_addr<=adress2;	W_wr<=1;	RF_Rp_addr<=adress2;	Rp_rd<=1;	RF_Rq_addr<=0;	Rq_rd<=0;	RF_s1<=1;	RF_s0<=0;	
	RF_W_data<=offset;	D_rd<=1'b1;	D_wr<=1'b1;	D_addr<=dir;D_addr_sel<=1;	
	PC_ld<=1;	PC_clr<=0;	PC_inc<=0;	IR_ld<=1;
	   end
	end
	4'b1100:begin//bnz
	fill=0;
	   if(~RF_Rp_zero)
	   begin
	       RF_W_addr<=adress2;	W_wr<=1;	RF_Rp_addr<=adress2;	Rp_rd<=1;	RF_Rq_addr<=0;	Rq_rd<=0;	RF_s1<=1;	RF_s0<=0;	
	RF_W_data<=offset;	D_rd<=1'b1;	D_wr<=1'b1;	D_addr<=dir;D_addr_sel<=1;	
	PC_ld<=1;	PC_clr<=0;	PC_inc<=0;	IR_ld<=1;
	   end
	end
	4'b1101:begin//jal
	fill=0;
	
	end
	4'b1110:begin //unconditional jump instruction
	RF_W_addr<=adress2;	W_wr<=1;	RF_Rp_addr<=adress2;	Rp_rd<=1;	RF_Rq_addr<=0;	Rq_rd<=0;	RF_s1<=1;	RF_s0<=0;	
	RF_W_data<=offset;	D_rd<=1'b1;	D_wr<=1'b1;	D_addr<=dir;D_addr_sel<=1;	
	PC_ld<=1;	PC_clr<=0;	PC_inc<=0;	IR_ld<=1;
	fill=0;
	end
	4'b1111:begin//JR
	fill=0;
	end
endcase

end
    2'b10:begin cycle<=cycle+1;//execute phase

    
end
    2'b11:begin cycle<=0;//reset
    PC_inc=1;//incrementing is resumed
    IR_ld<=0;
    PC_ld<=0;
    PC_clr<=0;
    D_wr<=0;
    W_wr<=0;
    D_addr_sel<=0;
    D_rd<=1;
end
    endcase
end    
end

initial begin
cycle=0;
end


endmodule



module Bit16MuxSig(input s1,s0,input [15:0] sig1,sig2, input [7:0] sigg3, output [15:0] signal);
reg [15:0] outSig;
wire [1:0] sel;
wire [15:0] sig3;
assign sig3[7:0]=sigg3;
assign sig3[15:8]=4'b0000;
assign sel[0]=s0;
assign sel[1]=s1;
assign signal=outSig;
always@(*)begin
	case(sel)
	2'b00:outSig<=sig1;
	2'b01:outSig<=sig2;
	2'b10:outSig<=sig3;
	2'b11:outSig<=0;
	endcase
end
endmodule 

module Bit8MuxSig(input D_addr_sel ,input [7:0] sig1,sig2 ,output [7:0] signal);
reg [7:0] outSig;
assign signal=outSig;
always@(*)begin
	case(D_addr_sel)
	1'b0:outSig=sig1;
	1'b1:outSig=sig2;
	endcase
end
endmodule 

module ALU(input [2:0] alu_s, input [15:0] A, B, output[15:0] data);

wire[15:0] addAns,subAns,andAns,orAns,xorAns,notAns,slaAns,sraAns;
reg [15:0] ans;
assign data=ans;
		
bit16Add one(A,B,addAns);
bit16Sub two(A,B,subAns);
bit16And three(A,B,andAns);
bit16Or  four(A,B,orAns);
bit16Xor five(A,B,xorAns);
bit16Not six(A,B,notAns);
bit16Sla seven(A,B,slaAns);
bit16Sra eight(A,B,sraAns);

	always@(*) begin
	case(alu_s)
		3'b000:ans=addAns;//ADD
		3'b001:ans=subAns;//SUB
		3'b010:ans=andAns;//AND
		3'b011:ans=orAns;//OR
		3'b100:ans=xorAns;//XOR
		3'b101:ans=notAns;//NOT
		3'b110:ans=slaAns;// left shift
		3'b111:ans=sraAns;// right shift
	endcase
	end

endmodule 

module dataReg(
input W_wr,Rp_rd,Rq_rd,input[3:0] W_addr,Rp_addr,Rq_addr,input [15:0] W_data,
output [15:0] Rp_data,Rq_data);

reg [15:0] mem [15:0];

reg[15:0] rp_data,rq_data;
//assign Rp_data=rp_data;
//assign Rq_data=rq_data;

assign Rp_data=rp_data;
assign Rq_data=rq_data;


always@(*)
begin

/*
if(W_wr)
mem[W_addr]<=W_data;
if(Rq_rd)
rq_data<=mem[Rq_addr];
if(Rp_rd)
rp_data<=mem[Rq_addr];
*/



if((Rp_rd&&W_wr)&&(Rq_addr==W_addr))
begin
rp_data<=W_data;
mem[W_addr]<=W_data;
end
//if write and read is called on the same register
//simply output the value of W_data without writting to register
else if((Rp_rd&&W_wr)&&(Rp_addr==W_addr))
rp_data<=W_data;
else 
begin
if(Rp_rd)//else read out register normally
rp_data<=mem[Rp_addr];
if(Rq_rd)
rq_data<=mem[Rq_addr];
if(W_wr)
mem[W_addr]<=W_data;
end




end

endmodule

module D(input [7:0] addr,input rd, input wr,input[15:0] W_data, output [15:0] R_data,input [7:0] sw,output [7:0] led);
reg [15:0] mem [255:0];
reg[15:0] r_data;
assign R_data=r_data;
//assign test=mem[245];
assign led=mem[sw];

always@(*)
begin
//if a read and write operation is done at the same time 
//simply output W_data to R_data without writting anything 
if(rd&&wr)
begin r_data=W_data; end 
else if(wr)
mem[addr]=W_data;
else if(rd)
r_data=mem[addr];
end

//the program written from registers 0 and onward
initial begin
//load 0 to 7 to registers 0 to 7
//store into data registers instructions 
mem[0]<=16'b1000000000000000;
mem[1]<=16'b1000000100000001;
mem[2]<=16'b1000001000000010;
mem[3]<=16'b1000001100000011;
mem[4]<=16'b1000010000000100;
mem[5]<=16'b1000010100000101;
mem[6]<=16'b1000011000000110;
mem[7]<=16'b1000011100000111;

//logical operations instructions 
//add 110(hex 6) and 011 (hex 3) to make 1001(hex 9) store in reg 8
mem[8]<=16'b0000100001100011;
//subtract =110(hex 6) and 011 (hex 3) to make 011(hex 3) store in reg 9
mem[9]<=16'b0001100101100011;
//AND 0010(hex 1) with 0011 (Hex 3) to get 0010(hex 2) store in reg 10
mem[10]<=16'b0010101000110010;
//OR  101(hex 5) and 111(hex 7) to get 111(hex 7) and store in reg 11
mem[11]<=16'b0011101101010111;
//XOR 100(hex 4) and 101(hex 5) to get 0001(hex 1) store in reg 12
mem[12]<=16'b0100110001000101;
//not 0101(hex 5) which extened is0000000000000101 to get 1111111111110101 (hex FFFA) and store in register 13
mem[13]<=16'b0101110101010000;
//left shift 0010(hex 2) to get 0100(4) and store in register 14
mem[14]<=16'b0110111000100000;
//right shift 0010(hex 2) to get 0001(hex 1) and store in register 15
mem[15]<=16'b0111111100100000;

//memory storage instructions 
//store information from register 8 into memory location 240 binary 1111 0000
mem[16]<=16'b1010100011110000;
//store information from register 9 into memory location 241 binary 1111 0001
mem[17]<=16'b1010100111110001;
//store information from register 10 into memory location 242 binary 1111 0010
mem[18]<=16'b1010101011110010;
//store information from register 11 into memory location 243 binary 1111 0011
mem[19]<=16'b1010101111110011;


//store information from register 12 into memory location 244 binary 1111 0100
mem[20]<=16'b1010110011110100;
//store information from register 13 into memory location 245 binary 1111 0101
mem[21]<=16'b1010110111110101;
//store information from register 14 into memory location 246 binary 1111 0110
mem[22]<=16'b1010111011110110;
//store information from register 15 into memory location 247 binary 1111 0111
mem[23]<=16'b1010111111110111;



end


endmodule



module bit16Add (input [15:0] A,B,output [15:0]addAns);
assign addAns=A+B;
endmodule

module bit16Sub (input [15:0]A,B,output [15:0]subAns);
assign subAns=A-B;
endmodule

module bit16And (input [15:0]A,B,output [15:0]andAns);
assign andAns=A&B;
endmodule

module bit16Or  (input [15:0]A,B,output [15:0]orAns);
assign orAns=A|B;
endmodule

module bit16Xor (input [15:0]A,B,output [15:0]xorAns);
assign xorAns=A^B;
endmodule

module bit16Not (input [15:0]A,B,output [15:0]notAns);
assign notAns=~A;
endmodule

module bit16Sla (input [15:0]A,B,output [15:0]slaAns);
assign slaAns=A<<1;
endmodule

module bit16Sra (input [15:0]A,B,output [15:0]sraAns);
assign sraAns=A>>1;
endmodule

module FSM
(
input clk, reset,
input [3:0] Opcode,
input RF_Rp_zero,
output IR_ld,
output PC_ld,
output PC_inc,
output PC_clr,
output D_addr_sel,
output D_rd,
output D_wr,
output RF_s1,
output RF_s0,
output W_wr,
output rd_Rp,
output rd_Rq,
output [2:0] alu_s
);

reg [14:0] controls;



always@(*)
begin
case(Opcode)
4'b0000: controls = 15'b000000000111000;//ADD
4'b0001: controls = 15'b000000000111001;//SUB
4'b0010: controls = 15'b000000000111010;//AND
4'b0011: controls = 15'b000000000111011;//OR
4'b0100: controls = 15'b000000000111100;//XOR
4'b0101: controls = 15'b000000000110101;//NOT
4'b0110: controls = 15'b000000000110110;//SLA
4'b0111: controls = 15'b000000000110111;//SRA
4'b1000: controls = 15'b000000010100000;//LI
4'b1001: controls = 15'b000011001100000;//LW
4'b1010: controls = 15'b000010100010000;//SW
4'b1011: controls = 15'b010001000000000;//BIZ//need to figure these ones out 
4'b1100: controls = 15'b010001000000000;//BNZ
4'b1101: controls = 15'b010000000100000;//JAL
4'b1110: controls = 15'b010001000000000;//JMP
4'b1111: controls = 15'b000101001010000;//JR
endcase
end

assign {IR_ld, PC_ld, PC_inc, PC_clr, D_addr_sel, D_rd, D_wr, RF_s1, RF_s0, W_wr, rd_Rp, rd_Rq, alu_s} = controls;

endmodule

/////////////////////////////


module Decoder
(
input clk, reset,
input [15:0] Instr,
input RF_Rp_zero,
output IR_ld,
output PC_ld,
output PC_inc,
output PC_clr,
output D_addr_sel,
output D_rd,
output D_wr,
output RF_s1,
output RF_s0,
output W_wr,
output rd_Rp,
output rd_Rq,
output [2:0] alu_s,
output [7:0] D_addr,
output [7:0] RF_W_data,
output [3:0] RF_W_addr,
output [3:0] RF_Rp_addr,
output [3:0] RF_Rq_addr
);
assign alu_s=Instr[14:11];

reg t_IR_ld;
reg t_PC_ld;
reg t_PC_inc;
reg t_PC_clr;
reg t_D_addr_sel;
reg t_D_rd;
reg t_D_wr;
reg t_RF_s1;
reg t_RF_s0;
reg t_W_wr;
reg t_rd_Rp;
reg t_rd_Rq;
reg [2:0] t_alu_s;
reg [7:0] t_D_addr;
reg [7:0] t_W_data;
reg [3:0] t_W_addr;
reg [3:0] t_Rp_addr;
reg [3:0] t_Rq_addr;

wire [3:0] adress0,adress1,adress2;
wire [7:0] imm,dir,offset;
assign adress2=Instr[11:8];
assign adress1=Instr[7:4];
assign adress0=Instr[3:0];

assign imm[7:4]=adress1;
assign imm[3:0]=adress0;

assign dir[7:4]=adress1;
assign dir[3:0]=adress0;

assign offset[7:4]=adress1;
assign offset[3:0]=adress0;

reg [15:0] JAL;
initial begin
JAL=0;
end


assign IR_ld=t_IR_ld;	
assign	PC_ld=	 t_PC_ld;	
assign	  PC_inc	=	 t_PC_inc;	
assign	  PC_clr	=	 t_PC_clr;	
assign	  D_addr_sel	=	 t_D_addr_sel;	
assign	  D_rd	=	 t_D_rd;	

assign	  D_wr	=	 t_D_wr;	

assign	  RF_s1	=t_RF_s1;	
assign	  RF_s0	=t_RF_s0;	

assign	  W_wr	=	 t_W_wr;	
assign	  rd_Rp	=	 t_rd_Rp;	
assign	  rd_Rq	=	 t_rd_Rq;	
assign	  alu_s	=	 Instr[14:12];	
assign	 D_addr	=	  t_D_addr;	
assign	   RF_W_data	=	 t_W_data;	
assign	   RF_W_addr	=	  t_W_addr;	
assign	   RF_Rp_addr	=	  t_Rp_addr;	
assign	   RF_Rq_addr	=	  t_Rq_addr;	


 reg fill;
 
always@(*)
begin
case(Instr[15:12])
	4'b0000:begin
	fill=0;
	t_W_addr<=adress2;	t_W_wr<=1;	t_Rp_addr<=adress1;	t_rd_Rp<=1;	t_Rq_addr<=adress0;	t_rd_Rq<=1;	t_RF_s1<=0;	t_RF_s0<=0;	
        t_W_data<=16'h0000;    t_D_rd<=1'b0;    t_D_wr<=1'b0;    t_D_addr<=4'b0000;t_D_addr_sel<=0;    
        t_PC_ld<=0;    t_PC_clr<=0;    t_PC_inc<=0;    t_IR_ld<=0;
	end
	4'b0001:begin
	t_W_addr<=adress2;	t_W_wr<=1;	t_Rp_addr<=adress1;	t_rd_Rp<=1;	t_Rq_addr<=adress0;	t_rd_Rq<=1;	t_RF_s1<=0;	t_RF_s0<=0;	
            t_W_data<=16'h0000;    t_D_rd<=1'b0;    t_D_wr<=1'b0;    t_D_addr<=4'b0000;t_D_addr_sel<=0;    
            t_PC_ld<=0;    t_PC_clr<=0;    t_PC_inc<=0;    t_IR_ld<=0;
	fill=0;
	end
	4'b0010:begin
	fill=0;
	t_W_addr<=adress2;	t_W_wr<=1;	t_Rp_addr<=adress1;	t_rd_Rp<=1;	t_Rq_addr<=adress0;	t_rd_Rq<=1;	t_RF_s1<=0;	t_RF_s0<=0;	
            t_W_data<=16'h0000;    t_D_rd<=1'b0;    t_D_wr<=1'b0;    t_D_addr<=4'b0000;t_D_addr_sel<=0;    
            t_PC_ld<=0;    t_PC_clr<=0;    t_PC_inc<=0;    t_IR_ld<=0;
	end
	4'b0011:begin
	fill=0;
	t_W_addr<=adress2;	t_W_wr<=1;	t_Rp_addr<=adress1;	t_rd_Rp<=1;	t_Rq_addr<=adress0;	t_rd_Rq<=1;	t_RF_s1<=0;	t_RF_s0<=0;	
            t_W_data<=16'h0000;    t_D_rd<=1'b0;    t_D_wr<=1'b0;    t_D_addr<=4'b0000;t_D_addr_sel<=0;    
            t_PC_ld<=0;    t_PC_clr<=0;    t_PC_inc<=0;    t_IR_ld<=0;
	end
	4'b0100:begin
	fill=0;
	t_W_addr<=adress2;	t_W_wr<=1;	t_Rp_addr<=adress1;	t_rd_Rp<=1;	t_Rq_addr<=adress0;	t_rd_Rq<=1;	t_RF_s1<=0;	t_RF_s0<=0;	
            t_W_data<=16'h0000;    t_D_rd<=1'b0;    t_D_wr<=1'b0;    t_D_addr<=4'b0000;t_D_addr_sel<=0;    
            t_PC_ld<=0;    t_PC_clr<=0;    t_PC_inc<=0;    t_IR_ld<=0;
	end
	4'b0101:begin
	fill=0;
	t_W_addr<=adress2;	t_W_wr<=1;	t_Rp_addr<=adress1;	t_rd_Rp<=1;	t_Rq_addr<=adress0;	t_rd_Rq<=1;	t_RF_s1<=0;	t_RF_s0<=0;	
            t_W_data<=16'h0000;    t_D_rd<=1'b0;    t_D_wr<=1'b0;    t_D_addr<=4'b0000;t_D_addr_sel<=0;    
            t_PC_ld<=0;    t_PC_clr<=0;    t_PC_inc<=0;    t_IR_ld<=0;
	end
	4'b0110:begin
	fill=0;
	t_W_addr<=adress2;	t_W_wr<=1;	t_Rp_addr<=adress1;	t_rd_Rp<=1;	t_Rq_addr<=adress0;	t_rd_Rq<=1;	t_RF_s1<=0;	t_RF_s0<=0;	
            t_W_data<=16'h0000;    t_D_rd<=1'b0;    t_D_wr<=1'b0;    t_D_addr<=4'b0000;t_D_addr_sel<=0;    
            t_PC_ld<=0;    t_PC_clr<=0;    t_PC_inc<=0;    t_IR_ld<=0;
	end
	4'b0111:begin
	fill=0;
	t_W_addr<=adress2;	t_W_wr<=1;	t_Rp_addr<=adress1;	t_rd_Rp<=1;	t_Rq_addr<=adress0;	t_rd_Rq<=1;	t_RF_s1<=0;	t_RF_s0<=0;	
            t_W_data<=16'h0000;    t_D_rd<=1'b0;    t_D_wr<=1'b0;    t_D_addr<=4'b0000;t_D_addr_sel<=0;    
            t_PC_ld<=0;    t_PC_clr<=0;    t_PC_inc<=0;    t_IR_ld<=0;
	end
	4'b1000:begin
	t_W_addr<=adress2;	t_W_wr<=1;	t_Rp_addr<=adress1;	t_rd_Rp<=0;	t_Rq_addr<=adress0;	t_rd_Rq<=0;	t_RF_s1<=1;	t_RF_s0<=0;	
	t_W_data<=imm;	t_D_rd<=1'b0;	t_D_wr<=1'b0;	t_D_addr<=4'b0000;t_D_addr_sel<=0;	
	t_PC_ld<=0;	t_PC_clr<=0;	t_PC_inc<=0;	t_IR_ld<=0;
	end
	4'b1001:begin//load word from memoery to register
	fill=0;
	t_W_addr<=adress2;	t_W_wr<=1;	t_Rp_addr<=0;	t_rd_Rp<=0;	t_Rq_addr<=adress0;	t_rd_Rq<=0;	t_RF_s1<=0;	t_RF_s0<=1;	
	t_W_data<=0;	t_D_rd<=1'b1;	t_D_wr<=1'b0;	t_D_addr<=dir;t_D_addr_sel<=1;	
	t_PC_ld<=0;	t_PC_clr<=0;	t_PC_inc<=0;	t_IR_ld<=0;
	end
	4'b1010:begin//store word from data register into permanent memory 
	fill=0;
	t_W_addr<=0;	t_W_wr<=0;	t_Rp_addr<=adress2;	t_rd_Rp<=1;	t_Rq_addr<=adress0;	t_rd_Rq<=0;	t_RF_s1<=1;	t_RF_s0<=0;	
	t_W_data<=0;	t_D_rd<=1'b0;	t_D_wr<=1'b1;	t_D_addr<=dir;t_D_addr_sel<=1;	
	t_PC_ld<=0;	t_PC_clr<=0;	t_PC_inc<=0;	t_IR_ld<=0;
	end
	4'b1011:begin//biz
	fill=0;
	   if(RF_Rp_zero)
	   begin
	       t_W_addr<=adress2;	t_W_wr<=1;	t_Rp_addr<=adress2;	t_rd_Rp<=1;	t_Rq_addr<=0;	t_rd_Rq<=0;	t_RF_s1<=1;	t_RF_s0<=0;	
	t_W_data<=offset;	t_D_rd<=1'b1;	t_D_wr<=1'b1;	t_D_addr<=dir;t_D_addr_sel<=1;	
	t_PC_ld<=1;	t_PC_clr<=0;	t_PC_inc<=0;	t_IR_ld<=1;
	   end
	end
	4'b1100:begin//bnz
	fill=0;
	   if(~RF_Rp_zero)
	   begin
	       t_W_addr<=adress2;	t_W_wr<=1;	t_Rp_addr<=adress2;	t_rd_Rp<=1;	t_Rq_addr<=0;	t_rd_Rq<=0;	t_RF_s1<=1;	t_RF_s0<=0;	
	t_W_data<=offset;	t_D_rd<=1'b1;	t_D_wr<=1'b1;	t_D_addr<=dir;t_D_addr_sel<=1;	
	t_PC_ld<=1;	t_PC_clr<=0;	t_PC_inc<=0;	t_IR_ld<=1;
	   end
	end
	4'b1101:begin//jal
	fill=0;
	
	end
	4'b1110:begin //unconditional jump instruction
	t_W_addr<=adress2;	t_W_wr<=1;	t_Rp_addr<=adress2;	t_rd_Rp<=1;	t_Rq_addr<=0;	t_rd_Rq<=0;	t_RF_s1<=1;	t_RF_s0<=0;	
	t_W_data<=offset;	t_D_rd<=1'b1;	t_D_wr<=1'b1;	t_D_addr<=dir;t_D_addr_sel<=1;	
	t_PC_ld<=1;	t_PC_clr<=0;	t_PC_inc<=0;	t_IR_ld<=1;
	fill=0;
	end
	4'b1111:begin//JR
	fill=0;
	end
endcase


end

endmodule 

//////////////////////

module PC
(
input clk, clear,
input load,
input increment,
input [7:0] D,
output [7:0] Q
);

reg [7:0] Q1;

//counter starts at 0;
initial begin
Q1=0;
end

always@(posedge clk)
begin
if(clear)               // Clear takes 1st priority
Q1 <= 0;
else if(load)           // Load takes 2nd priority
Q1 <= D;
else if(increment)      // Incremenet tkes 3rd priority
Q1 <= Q + 1;
end
assign Q = Q1;
endmodule

module IR
(
input clk, load,
input [15:0] current_Instr,
output [15:0] Instr
);

reg [15:0] next_Instr;

///changed from always@(posedge clk)
always@(*)
begin
if(load)
next_Instr <= current_Instr;
end

assign Instr = next_Instr;

endmodule

module Adder
(
input [7:0] A,
input [7:0] B,
output [7:0] res
);

reg [7:0] C;

always@(*)
begin
C = A + B - 1;
end
assign res = C;

endmodule

