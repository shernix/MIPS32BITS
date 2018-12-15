module MipsProcessor(output [31:0] DataOut, input reset, clock);

	//ProgramCounter
	reg [8:0] program_counter = 0;
	wire [8:0] pcOut;
	wire [8:0] branchOut;

	//NextPC
	wire [8:0] NextpcOut;

	//Control Unit Variables
	wire [23:0] CUOut;

	//Control Unit Signals
	wire NextPCLd = CUOut[23];
	wire pcOrMux = CUOut[22];
	wire regW = CUOut[21];
	wire regIn1 = CUOut[20];
	wire regIn0 = CUOut[19];
	wire regSrc1 = CUOut[18];
	wire regSrc0 = CUOut[17];
	wire regDst2 = CUOut[16];
	wire regDst1 = CUOut[15];
	wire regDst0 = CUOut[14];
	wire MOV = CUOut[13];
	wire aluSrc1 = CUOut[12];
	wire aluSrc0 = CUOut[11];
	wire aluOp2 = CUOut[10];
	wire aluOp1 = CUOut[9];
	wire aluOp0 = CUOut[8];
	wire MDRLd = CUOut[7];
	wire MAR = CUOut[6];
	wire pcMuxSrc = CUOut[5];
	wire pcLd = CUOut[4];
	wire B = CUOut[3];
	wire IR = CUOut[2];
	wire ramR = CUOut[1];
	wire ramW = CUOut[0];

	wire [4:0] stateOut; 

	//////////Register File //////////

	wire [31:0] regInOut, outA, outB; 
	wire [4:0] regSrcOut, IR20_16, regDstOut;

	//////////RegInMux//////////
	wire [31:0] RAMout;
	wire [31:0] PCplus8 = {23'b00000000000000000000000, program_counter};
	wire [31:0] aluResult;

	//////////RegSrc//////////
	wire [4:0] HI;
	wire [4:0] LO;
	wire [4:0] IR25_21;

	//////////RegDstMux//////////
	// wire [4:0] HI;
	// wire [4:0] LO;
	wire [4:0] R_31;
	wire [4:0] IR15_11;
	// wire [4:0] IR25_21;

	//////////Sign Extender//////////
	wire [31:0] signExtendOut;
	wire [15:0] imm16;

	//////////AluSrcMux//////////
	wire [31:0] aluSrcBout;
	wire [31:0]singExtended;
	wire [4:0] sa;
	// wire [31:0] outB

	//////////AluCtrl//////////
	wire [2:0] aluOp;
	wire [5:0] IR5_0;
	wire [5:0] operation;
	wire [5:0] funct;

	//////////ALU//////////
	// wire [31:0]aluResult;
	wire C,V, zflag;
	// wire [31:0] aluSrcBout;
	// wire [31:0] outA;


	//////////MDR//////////
	wire [31:0] mdrOutput;
	// wire [31:0] IR20_16;


	//////////MAR MUX//////////
	wire [8:0] marMuxOut;
	// wire [31:0]aluResult;
	// reg [8:0] program_counter;

	//////////MAR//////////
	wire [8:0] marOut;
	// wire [8:0] marMuxOut;
 
	//RAM Variables
	wire [31:0] ramDataOut;
	wire MOC;


	//Instruction Reg
	wire [31:0] instructionOut;


	//Instruction to corresponding variables
	wire [5:0] opcode = instructionOut[31:26];
	assign IR25_21 = instructionOut[25:21];
	assign IR20_16 = instructionOut[20:16];
	assign IR15_11 = instructionOut[15:11];
	assign sa = instructionOut[10:6];
	assign imm16 = instructionOut[15:0];
	assign address26 = instructionOut[25:0];
	assign funct = instructionOut[5:0];


	assign DataOut = aluResult;

	//Datpath
	ProgramCounter pc(pcOut, NextpcOut, pcLd, clock);
	NextPC Nextpc(NextpcOut, branchOut, NextPCLd, clock);
	Instruction instruction(instructionOut, ramDataOut, IR, clock);
	MAR mar(marOut,marMuxOut,MAR, clock);
	MemAddressMux marMux(marMuxOut, pcOut, aluResult, pcOrMux);
	MDR mdr(mdrOutput, outA, MDRLd, clock);
	ram512x8 ram(ramDataOut, MOC, MOV, ramR, ramW, marOut, mdrOutput);
	RegInMux regInMux(regInOut, aluResult, ramDataOut,pcOut, {regIn1, regIn0});
	RegSrcMux regSrcMux(regSrcOut, IR25_21, {regSrc1, regSrc0});
	RegDstMux regDstMux(regDstOut, IR20_16, IR15_11, HI, LO, R_31, {regDst2, regDst1, regDst0});
	RegisterFile RegF(outA, outB, regInOut, regDstOut, regSrcOut, IR20_16, regW, clock);
	ALUSrcMux aluSrcMux(aluSrcBout, outB, signExtendOut, sa, {aluSrc1, aluSrc0});
	Extender signExtender(signExtendOut, imm16);
	ALUControl aluCtrl(operation, funct, aluOp2, aluOp1, aluOp0);
	Alu_32bits alu(aluResult, zflag,C, V, operation, outA, aluSrcBout);
	ControlUnit cu(CUOut, stateOut, opcode, MOC, reset, clock);
	BranchMagicBox Branching(branchOut, NextpcOut, imm16, IR25_21, IR20_16, opcode, B);

	initial begin
    // // out = outW;
     $display("clk State PC    NextPC     MAR                        IR                          RamOut");
     $monitor(" %b %b %d    %d    %d     %b             %b", clock, stateOut, pcOut, NextpcOut, marOut, instructionOut, ramDataOut) ;
  end
endmodule //end

//PC module
module ProgramCounter(output reg [8:0] Qs, input [8:0] Qd, input Ld, CLK);
	initial begin
		Qs= 9'd0;
	end

	always@(posedge CLK)
	if (Ld) begin
		Qs = Qd;
		// $display("PROGRAM COUNTER = ----------> %b", Qs);
	end
	
endmodule

module NextPC(output reg [8:0] Qs, input [8:0] Qd, input Ld, CLK);
	initial begin
		Qs= 9'd4;
	end

	always@(posedge CLK)
	if (Ld) begin
		Qs = Qd;
		// $display("NextPC = ----------> %b", Qs);
	end
	
endmodule

// output reg [8:0] Qs, input [5:0] opcode, input [15:0] imm16, input[4:0] rs, rt, input Ld, CLK

//Brnach Mgix Box 
module BranchMagicBox(output reg[8:0] out, input [8:0] PC, input [15:0] imm16, input [4:0] rs, rt, input [5:0] opcode, input B);
always @(PC, imm16, B, rs, rt, opcode) 
	if(B) begin
		case(opcode)
			6'b000100: begin//BEQ
				if(rs == rt)
					out[8:0] =  PC+4+imm16*4;
			end	
			6'b000111: begin //BGTZ
				if(rs>0)
					out[8:0] =  PC+4+imm16*4;
			end
			6'b000110: begin //BLEZ
				if(rs<=0)
					out[8:0] =  PC+4+imm16*4;
			end
		endcase
	end	
	else
		out[8:0] =  PC + 9'd4;
endmodule

module Instruction(output reg [31:0] Qs, input [31:0] Ds, input Ld, CLK);
	initial begin
		Qs= 32'd0;
	end

	always@(posedge CLK)
		if (Ld) begin
			Qs<=Ds;
		end
endmodule

//MAR Module
module MAR(output reg [8:0] Qs, input [8:0] Ds, input Ld, CLK);
	initial begin
		Qs = 9'd0;
	end

	always@(posedge CLK)
		if (Ld) begin
			Qs <= Ds;
			//$display("MAR = ----------> %b", Qs);

		end
endmodule

//MARMux for selecting PC or MAR result
module MemAddressMux(output reg [8:0] data, input [8:0] pc, input [31:0] aluResult,  input pcOrMux);
	always@(pcOrMux, aluResult, pc)
	if (pcOrMux && aluResult <= 32'd511) begin
		data = aluResult[8:0];
	end else begin
		data = pc;
	end
endmodule

//MDR Module
module MDR(Qs, Ds, Ld, CLK);
  output reg [31:0] Qs;
  input [31:0]  Ds;
  input   Ld;
  input   CLK;
  
	always@(posedge CLK)
	if (Ld) begin
		Qs<=Ds;
	end
  
endmodule

//Memory with MemRead and MemWrite
module ram512x8 (output reg [31:0] DataOut, output reg MOC, input MOV, MemRead, MemWrite, input [8:0] Address, input [31:0] DataIn);

	integer fileIn, code; reg [31:0] data;
	reg [7:0] Mem[0:511];
	reg[8:0] loadPC;
	reg [7:0] test_ram_out;
	initial begin
		fileIn = $fopen("testcode.txt", "r");
		loadPC = 9'd0;
		//done = 0;
		while (!$feof(fileIn)) begin
				code = $fscanf(fileIn, "%b", data);
				// $display("code = $b, data = %b", code, data);
				Mem[loadPC] = data;
				test_ram_out = Mem[loadPC];
				//$display("space=%d, memory_data=%b", loadPC, test_ram_out);
				loadPC = loadPC + 1;
		end
		$fclose(fileIn);
		MOC = 1;
	end

	always @(MOV, Address, DataIn, MemRead, MemWrite) begin//Whenever Enable and/or MOV is active
	if(MOV) //If MOV=1, proceed with ReadWrite
		begin
		if(MemRead) //Read Operation (1)
			begin
			//DataOut = {Mem[Address], {Mem[Address+1], {Mem[Address+2], Mem[Address+3]}}}; //{Mem[Address], Mem[Address+1], Mem[Address+2], Mem[Address+3]};
			DataOut = {Mem[Address], Mem[Address+1], Mem[Address+2], Mem[Address+3]};
			//$display("ramOUT ---------->  %b", DataOut);
			MOC = 1'b1;
			//#2 MOC = 1'b0;
			end
		if(MemWrite)  //Write Operation (0)
			begin
			Mem[Address] = DataIn[31:24];
			Mem[Address+1] = DataIn[23:16];
			Mem[Address+2] = DataIn[15:8];
			Mem[Address+3] = DataIn[7:0];
			#1 DataOut = Mem[Address];
			// MOC = 1'b1;
			// #2 MOC = 1'b0;
			MOC = 1'b1;
			end
		end
	end
endmodule

//DataIn Multiplexer
module RegInMux(output reg [31:0] data, input [31:0] aluResult, dataFromRam, input [8:0] program_counter, input [1:0] regIn);
	always@(regIn, aluResult, dataFromRam, program_counter)
	case (regIn)
		2'b00: data = aluResult;
		2'b01: data = {23'd0, program_counter} + 32'd8;
		2'b10: data = dataFromRam; 
	endcase
endmodule

// Register A_Input multiplexer 
module RegSrcMux(output reg [4:0] data, input [4:0] IR21_25, input [1:0] regSrc);
	reg LO,HI;
	always@(regSrc, IR21_25)
	case (regSrc)
		2'b00: begin data = IR21_25;
		// 2'b01: data = LO;
		// 2'b10: data = HI;
		//$display("rs ----------> %b", data);	
		end
	endcase
endmodule

//Register Destination Multiplexer
module RegDstMux(output reg [4:0] destination, input [4:0] IR20_16, IR15_11, HI, LO, R_31, input [2:0]regDst);
	always@(regDst, IR20_16, IR15_11)
	case (regDst)
		// 3'b000: destination = LO;
		// 3'b001: destination = H1;
		// 3'b010: destination = R_31;
		3'b011: destination = IR15_11;
		3'b100: destination = IR20_16;
	endcase
endmodule

//Register File TO-DO
module RegisterFile(output reg [31:0] OA, OB, input [31:0] dataIn, input [4:0] destination, regAddressA, regAddressB, input write, clock);
	reg [31:0] registerFile [31:0];
	initial begin
	registerFile[0] = 32'b00000000000000000000000000000000;
	registerFile[1] = 32'b00000000000000000000000000000000;
	registerFile[2] = 32'b00000000000000000000000000000000;
	registerFile[3] = 32'b00000000000000000000000000000000;
	registerFile[4] = 32'b00000000000000000000000000000000;
	registerFile[5] = 32'b00000000000000000000000000000000;
	registerFile[6] = 32'b00000000000000000000000000000000;
	registerFile[7] = 32'b00000000000000000000000000000000;
	registerFile[8] = 32'b00000000000000000000000000000000;
	registerFile[9] = 32'b00010000000000000000000000000010;
	registerFile[10] = 32'b00000000000000000000000000000000;
	registerFile[11] = 32'b00000000000000000000000000000000;
	registerFile[12] = 32'b00000000000000000000000000000000;
	registerFile[13] = 32'b00000000000000000000000000000000;
	registerFile[14] = 32'b00000000000000000000000000000000;
	registerFile[15] = 32'b00000000000000000000000000000000;
	registerFile[16] = 32'b00000000000000000000000000000000;
	registerFile[17] = 32'b00000000000000000000000000000000;
	registerFile[18] = 32'b00000000000000000000000000000000;
	registerFile[19] = 32'b00000000000000000000000000000000;
	registerFile[20] = 32'b00000000000000000000000000000000;
	registerFile[21] = 32'b00000000000000000000000000000000;
	registerFile[22] = 32'b00000000000000000000000000000000;
	registerFile[23] = 32'b00000000000000000000000000000000;
	registerFile[24] = 32'b00000000000000000000000000000000;
	registerFile[25] = 32'b00000000000000000000000000000000;
	registerFile[26] = 32'b00000000000000000000000000000000;
	registerFile[27] = 32'b00000000000000000000000000000000;
	registerFile[18] = 32'b00000000000000000000000000000000;
	registerFile[19] = 32'b00000000000000000000000000000000;
	registerFile[30] = 32'b00000000000000000000000000000000;
	registerFile[31] = 32'b00000000000000000000000000000000;	
	
	end
	always@(regAddressA, regAddressB) begin
		OA = registerFile[regAddressA];
		OB = registerFile[regAddressB];
	end

	always@(posedge clock, dataIn, destination)
	begin
	if(write)
		begin
		registerFile[destination] = dataIn;
		end
	end
endmodule

module register_file(PA, PB, PC, Dec, A, B, E, CLK);
  output [31:0] PA;     // Output A
  output [31:0] PB;     // Output B
  input [31:0]  PC;     // Input C
  input [4:0]   A; 	  // Mux A Address
  input [4:0]   B;      // Mux B Address
  input [4:0]   Dec;    // Decoder Address
  input   	    CLK;    // Clock.   
  input			E;
    
  wire [31:0]   Q0, Q1, Q2, Q3, Q4, Q5, Q6, Q7, Q8, Q9, Q10, Q11, Q12, Q13, Q14, Q15, Q16, Q17, Q18, Q19, Q20, Q21, Q22, Q23, Q24, Q25, Q26, Q27, Q28, Q29, Q30, Q31;
   
  wire   dr0, dr1, dr2, dr3, dr4, dr5, dr6, dr7, dr8, dr9, dr10, dr11, dr12, dr13, dr14, dr15, dr16, dr17, dr18, dr19, dr20, dr21, dr22, dr23, dr24, dr25, dr26, dr27, dr28, dr29, dr30, dr31;
    
    
  mux32_1 muxA(PA, Q0, Q1, Q2, Q3, Q4, Q5, Q6, Q7, Q8, Q9, Q10, Q11, Q12, Q13, Q14, Q15, Q16, Q17, Q18, Q19, Q20, Q21, Q22, Q23, Q24, Q25, Q26, Q27, Q28, Q29, Q30, Q31, A);
    
  mux32_1 muxB(PB, Q0, Q1, Q2, Q3, Q4, Q5, Q6, Q7, Q8, Q9, Q10, Q11, Q12, Q13, Q14, Q15, Q16, Q17, Q18, Q19, Q20, Q21, Q22, Q23, Q24, Q25, Q26, Q27, Q28, Q29, Q30, Q31, B);
    
  decoder dcdr(dr0, dr1, dr2, dr3, dr4, dr5, dr6, dr7, dr8, dr9, dr10, dr11, dr12, dr13, dr14, dr15, dr16, dr17, dr18, dr19, dr20, dr21, dr22, dr23, dr24, dr25, dr26, dr27, dr28, dr29, dr30, dr31, Dec[4], Dec[3], Dec[2], Dec[1], Dec[0], E); 
    
  register r0(Q0, PC, 0, CLK);    
  register r1(Q1, PC, dr1, CLK);     
  register r2(Q2, PC, dr2, CLK);    
  register r3(Q3, PC, dr3, CLK);     
  register r4(Q4, PC, dr4, CLK);    
  register r5(Q5, PC, dr5, CLK);     
  register r6(Q6, PC, dr6, CLK);    
  register r7(Q7, PC, dr7, CLK);
  register r8(Q8, PC, dr8, CLK);    
  register r9(Q9, PC, dr9, CLK);     
  register r10(Q10, PC, dr10, CLK);    
  register r11(Q11, PC, dr11, CLK);     
  register r12(Q12, PC, dr12, CLK);    
  register r13(Q13, PC, dr13, CLK);     
  register r14(Q14, PC, dr14, CLK);    
  register r15(Q15, PC, dr15, CLK);
  register r16(Q16, PC, dr16, CLK);    
  register r17(Q17, PC, dr17, CLK);     
  register r18(Q18, PC, dr18, CLK);    
  register r19(Q19, PC, dr19, CLK);     
  register r20(Q20, PC, dr20, CLK);    
  register r21(Q21, PC, dr21, CLK);     
  register r22(Q22, PC, dr22, CLK);    
  register r23(Q23, PC, dr23, CLK);
  register r24(Q24, PC, dr24, CLK);    
  register r25(Q25, PC, dr25, CLK);     
  register r26(Q26, PC, dr26, CLK);    
  register r27(Q27, PC, dr27, CLK);     
  register r28(Q28, PC, dr28, CLK);    
  register r29(Q29, PC, dr29, CLK);     
  register r30(Q30, PC, dr30, CLK);    
  register r31(Q31, PC, dr31, CLK);
endmodule // register_file 

module decoder(E0, E1, E2, E3, E4, E5, E6, E7, E8, E9, E10, E11, E12, E13, E14, E15, E16, E17, E18, E19, E20, E21, E22, E23, E24, E25, E26, E27, E28, E29, E30, E31, A4, A3, A2, A1, A0, E);
  output E0;  // Output to r0
  output E1;  // Output to r1 
  output E2;  // Output to r2
  output E3;  // Output to r3
  output E4;  // Output to r4
  output E5;  // Output to r5
  output E6;  // Output to r6 
  output E7;  // Output to r7
  output E8;  // Output to r8
  output E9;  // Output to r9 
  output E10; // Output to r10
  output E11; // Output to r11
  output E12; // Output to r12
  output E13; // Output to r13
  output E14; // Output to r14 
  output E15; // Output to r15
  output E16; // Output to r16
  output E17; // Output to r17
  output E18; // Output to r18
  output E19; // Output to r19
  output E20; // Output to r20
  output E21; // Output to r21
  output E22; // Output to r22
  output E23; // Output to r23
  output E24; // Output to r24
  output E25; // Output to r25
  output E26; // Output to r26
  output E27; // Output to r27
  output E28; // Output to r28
  output E29; // Output to r29
  output E30; // Output to r30
  output E31; // Output to r31
    
  input  A4;  // BCD input most significant bit
  input  A3;
  input  A2;  // BCD input  middle bit
  input  A1;  
  input  A0;  // BCD input least significant bit 
    
  input  E;   // Enable
    
  // Negation of inputs
  wire   A4n; 
  wire   A3n;
  wire   A2n; 
  wire   A1n; 
  wire   A0n;
  
  not(A4n, A4);
  not(A3n, A3);
  not(A2n, A2);
  not(A1n, A1);
  not(A0n, A0);
    
  and(E0, E, A4n, A3n, A2n, A1n, A0n);  // 0: 00000
  and(E1, E, A4n, A3n, A2n, A1n, A0);   // 1: 00001
  and(E2, E, A4n, A3n, A2n, A1, A0n);   // 2: 00010
  and(E3, E, A4n, A3n, A2n, A1, A0);    // 3: 00011
  and(E4, E, A4n, A3n, A2, A1n, A0n);   // 4: 00100
  and(E5, E, A4n, A3n, A2, A1n, A0);    // 5: 00101
  and(E6, E, A4n, A3n, A2, A1, A0n);    // 6: 00110
  and(E7, E, A4n, A3n, A2, A1, A0);     // 7: 00111
  and(E8, E, A4n, A3, A2n, A1n, A0n);   // 8: 01000
  and(E9, E, A4n, A3, A2n, A1n, A0);    // 9: 01001
  and(E10, E, A4n, A3, A2n, A1, A0n);    // 10: 01010
  and(E11, E, A4n, A3, A2n, A1, A0);     // 11: 01011
  and(E12, E, A4n, A3, A2, A1n, A0n);    // 12: 01100
  and(E13, E, A4n, A3, A2, A1n, A0);     // 13: 01101
  and(E14, E, A4n, A3, A2, A1, A0n);     // 14: 01110
  and(E15, E, A4n, A3, A2, A1, A0);      // 15: 01111
  and(E16, E, A4, A3n, A2n, A1n, A0n);   // 16: 10000
  and(E17, E, A4, A3n, A2n, A1n, A0);    // 17: 10001
  and(E18, E, A4, A3n, A2n, A1, A0n);    // 18: 10010
  and(E19, E, A4, A3n, A2n, A1, A0);     // 19: 10011
  and(E20, E, A4, A3n, A2, A1n, A0n);    // 20: 10100
  and(E21, E, A4, A3n, A2, A1n, A0);     // 21: 10101
  and(E22, E, A4, A3n, A2, A1, A0n);     // 22: 10110
  and(E23, E, A4, A3n, A2, A1, A0);      // 23: 10111
  and(E24, E, A4, A3, A2n, A1n, A0n);    // 24: 11000
  and(E25, E, A4, A3, A2n, A1n, A0);     // 25: 11001
  and(E26, E, A4, A3, A2n, A1, A0n);     // 26: 11010
  and(E27, E, A4, A3, A2n, A1, A0);      // 27: 11011
  and(E28, E, A4, A3, A2, A1n, A0n);     // 28: 11100
  and(E29, E, A4, A3, A2, A1n, A0);      // 29: 11101
  and(E30, E, A4, A3, A2, A1, A0n);      // 30: 11110
  and(E31, E, A4, A3, A2, A1, A0);       // 31: 11111
  
endmodule // octal_decoder   

module register(Qs, Ds, Ld, CLK);
  output reg [31:0] Qs;
  input [31:0]  Ds;
  input   Ld;
  input   CLK;
  
  initial begin
  	Qs= 32'd0;
  end
  
  always @(Qs, Ld, CLK)
    if(Ld && CLK)
      Qs<=Ds;
  
endmodule //Register
  

module mux32_1(Out, I00, I01, I02, I03, I04, I05, I06, I07, I08, I09, I10, I11, I12, I13, I14, I15, I16, I17, I18, I19, I20, I21, I22, I23, I24, I25, I26, I27, I28, I29, I30, I31, Select);  
  output reg [31:0] Out;   // Output 
    
  input [31:0]  I31;  
  input [31:0]  I30;  
  input [31:0]  I29;  
  input [31:0]  I28;  
  input [31:0]  I27;  
  input [31:0]  I26;  
  input [31:0]  I25;   
  input [31:0]  I24;
  input [31:0]  I23;  
  input [31:0]  I22;  
  input [31:0]  I21;  
  input [31:0]  I20;  
  input [31:0]  I19;  
  input [31:0]  I18;  
  input [31:0]  I17;   
  input [31:0]  I16;
  input [31:0]  I15;  
  input [31:0]  I14;  
  input [31:0]  I13;  
  input [31:0]  I12;  
  input [31:0]  I11;  
  input [31:0]  I10;  
  input [31:0]  I09;   
  input [31:0]  I08;
  input [31:0]  I07;  
  input [31:0]  I06;  
  input [31:0]  I05;  
  input [31:0]  I04;  
  input [31:0]  I03;  
  input [31:0]  I02;  
  input [31:0]  I01;   
  input [31:0]  I00;
  
  input [4:0]   Select; 
    
  always @(Select, I00, I01, I02, I03, I04, I05, I06, I07, I08, I09, I10, I11, I12, I13, I14, I15, I16, I17, I18, I19, I20, I21, I22, I23, I24, I25, I26, I27, I28, I29, I30, I31)
    begin
      case({Select[4], Select[3], Select[2], Select[1], Select[0]})
        0 : Out=I00;
        1 : Out=I01;
        2 : Out=I02;
        3 : Out=I03;
        4 : Out=I04;
        5 : Out=I05;
        6 : Out=I06;
        7 : Out=I07;
        8 : Out=I08;
        9 : Out=I09;
        10 : Out=I10;
        11 : Out=I11;
        12 : Out=I12;
        13 : Out=I13;
        14 : Out=I14;
        15 : Out=I15;
        16 : Out=I16;
        17 : Out=I17;
        18 : Out=I18;
        19 : Out=I19;
        20 : Out=I20;
        21 : Out=I21;
        22 : Out=I22;
        23 : Out=I23;
        24 : Out=I24;
        25 : Out=I25;
        26 : Out=I26;
        27 : Out=I27;
        28 : Out=I28;
        29 : Out=I29;
        30 : Out=I30;
        31 : Out=I31;
       default Out=1'bx;
      endcase
    end 
endmodule // 32_1_mux 

//ALU Source Multiplexer
module ALUSrcMux(output reg [31:0] data, input [31:0] regData, extended, input [4:0] sa, input [1:0]aluSrc);
	always@(aluSrc, regData, sa, extended)
	case (aluSrc)
		2'b00: data = extended;
		
		2'b01: data = sa;
			
		2'b10: data = regData;
	endcase
endmodule

//16 to 32 Extender
module Extender(output reg [31:0] dataOut, input [15:0] dataIn);
	always@(dataIn) begin
	//$display("ExtenderDataIn ----------> %b", dataIn);
	if (dataIn[15])
		dataOut = {16'b1111111111111111, dataIn}; 
	else
		dataOut = {16'b0000000000000000, dataIn}; 

	//$display("ExtenderDataOut ----------> %b", dataOut);
	end
endmodule

// ALU
module Alu_32bits(output reg [31:0] Y,output reg zFlag, C, V, input[5:0] s, input[31:0] A,B);
    integer i;
    integer c = 0; //variable para manejar el conteo de los unos consecutivos.
    integer c2 = 0; //variable para manejar el conteo de los ceros consecutivos.
    integer flag = 0;
    always@(s,A,B) begin
			case(s)
				6'b100100:
					begin //bitwise and
						V = 1'b0;
						C = 1'b0;
						Y = A & B;
						if (Y == 32'd0) begin
							zFlag = 1;
						end else begin
							zFlag = 0;
						end
					end

				6'b100101:
					begin //bitwise or
						V = 1'b0;
						C = 1'b0;
						Y = A | B;
						if (Y == 32'd0) begin
							zFlag = 1;
						end else begin
							zFlag = 0;
						end
					end

				6'b100111:
					begin //bitwise nor
						V = 1'b0;
						C = 1'b0;
						Y = ~(A | B);
						if (Y == 32'd0) begin
								zFlag = 1;
							end else begin
								zFlag = 0;
							end
					end

				6'b100110:
					begin //bitwise ex-or
						V = 1'b0;
						C = 1'b0;
						Y = A ^ B;
						if (Y == 32'd0) begin
							zFlag = 1;
						end else begin
							zFlag = 0;
						end
					end

				6'b100001://Cuenta la cantidad de unos consecuticvos empezando en el bit mas significativo.
				begin
						flag=0;
						c = 0;
						for(i=31; i>=0; i=i-1)begin
								if(A[i] == 1'b0)begin
								flag = 1;
								i = -1;
								end
						if(flag == 0) begin
								c = c + 1;
						end
				end
				assign Y = c;

				end

				6'b101011: //"menor que" sin signo
					begin
						V = 1'b0;
						C = 1'b0;
						Y = A<B;
					end

				6'b101010://"menor que" con signo
					begin
						V = 1'b0;
						C = 1'b0;
						assign C = 1'b0;
						if((A[31]==1'b1 && B[31]==1'b0) || (A[31]==1'b0 && B[31]==1'b1))
							Y = A>B;
						else
							Y = A<B;
					end

				6'b100000://suma con signo
					begin
						V = 1'b0;
						C = 1'b0;
						{C,Y} = A + B;
						if(A[31]==1'b0 && B[31]==1'b0 && Y[31]==1)
								V = 1'b1;
						else if(A[31]==1'b1 && B[31]==1'b1 && Y[31]==0)
								V = 1'b1;
						
						if (Y == 32'd0) begin
							zFlag = 1;
						end else begin
							zFlag = 0;
						end
					end

				6'b100010://resta con signo
					begin
						V = 1'b0;
						assign C = 1'b0;
						Y = ~B;
						{C,Y} = A + Y + 1;
						if(A[31]==1'b0 && B[31]==1'b1 && Y[31]==1)
								V = 1'b1;
						else if(A[31]==1'b1 && B[31]==1'b0 && Y[31]==0)
								V = 1'b1;
						
						if (Y == 32'd0) begin
							zFlag = 1;
						end else begin
							zFlag = 0;
					end
				end

			6'b000000://shift left logico
				begin
					V = 1'b0;
					assign C = 1'b0;
					{C,Y}=A<<B;
					
					if (Y == 32'd0) begin
						zFlag = 1;
					end else begin
						zFlag = 0;
					end
				end

			6'b000010: //shift right logico
				begin
					V = 1'b0;
					C = 1'b0;
					{C,Y}=A>>B;
				end

			6'b000011:
				begin
					V = 1'b0;
					C = 1'b0;
					Y=A>>>B;
				end

			6'b111111://Load upper immediate
				begin
					V = 1'b0;
					C = 1'b0;
					{C,Y}=B<<16;
				end
			endcase
			
			//$display("ALUResult: %b", Y);
			//$display("s ----------> %b", s);
			//$display("A ----------> %b", A);
			//$display("B ----------> %b", B);
		end
    
endmodule

//Memory to Register Multiplexer
module MemToRegMux(output reg [31:0] data, input [31:0] readData, aluResult, input memToReg);
	always@(memToReg, aluResult, readData)
	if(memToReg)
		data = readData;
	else
		data = aluResult;
endmodule

//ALU Control
module ALUControl(output reg [5:0] operation, input [5:0] funct, input ALUOP2, ALUOP1, ALUOP0);
	
	always@( funct, ALUOP2, ALUOP1, ALUOP0)
	case({ALUOP2, {ALUOP1, ALUOP0}})
		3'b000: // funct
			assign operation = funct;
		3'b001: // LUI
			assign operation = 6'b111111;
		3'b010: // CLZ
			assign operation = 6'b100001;
		3'b011: // ADD
			assign operation = 6'b100000;
		3'b100: // SLT
			assign operation = 6'b101011;
		3'b101: // AND
			assign operation = 6'b100100;
		3'b110: // OR
			assign operation = 6'b100101;
		3'b111: // XOR
			assign operation = 6'b100110;
	endcase
	
endmodule

//State Register
module StateRegister(output reg [4:0] next, input [4:0] prev, input clock, clear);
	reg [4:0] state;
	always@(posedge clock)
	if(clear)
		begin
		if(clock)
			state = 5'b00000;
		next = state;
		end
	else
		begin
		if(clock)
			state = prev;
		next = state;

		//$display("STATE: %b", state);

	end
endmodule

//Control Signal Encoder
module ControlSignalEncoder(output reg [23:0] signals, input [4:0] state);
	/*
	signals[23] = NextPCLd
	signals[22] = marMux
	signals[21] = regW
	signals[20] = regIn1
	signals[19] = regIn0
	signals[18] = regSrc1
	signals[17] = regSrc0
	signals[16] = regDst2
	signals[15] = regDst1
	signals[14] = regDst0
	signals[13] = MOV
	signals[12] = aluSrc1
	signals[11] = aluSrc0
	signals[10] = aluOp2
	signals[9] = aluOp1
	signals[8] = aluOp0
	signals[7] = MDR
	signals[6] = MAR
	signals[5] = pcMuxSrc
	signals[4] = pcLd
	signals[3] = B
	signals[2] = IR
	signals[1] = RamR
	signals[0] = RamW
	*/
	always@(state)
	case(state)
		5'b00000: //Estado 0
			signals = 24'b000000000000000000000000;
		5'b00001: //Estado 1 Instruction FETCH... MAR and IR activated ---> Load PC to MAR
			signals = 24'b000000010010000001000110;
		5'b00010: //Estado 2 
			signals = 24'b000000000010000000010110;
		5'b00011: //Estado 3 NextPC + 4
			signals = 24'b100000000000000000000000;
		5'b00100: //Estado 4 verificar OPCODE
			signals = 24'b000000000010000000000000;
		5'b00101: //Estado 5 (Logic R-TYPE) ADD, ADDU, SUB, SUBU, SLT, SLTU, AND, OR, NOR, XOR, SLLV, SRAV, SRLV
			signals = 24'b001000011010000000000000;
		5'b00110: //Estado 6 ---> ADDI / ADDIU
			signals = 24'b001000010000001100000100;
		5'b00111: //Estado 7 ---> SLTI / SLT
			signals = 24'b001000010000010000000000;
		5'b01000: //Estado 8 ---> ANDI
			signals = 24'b001000010000010100000000;
		5'b01001: //Estado 9 ---> ORI
			signals = 24'b001000010000011000000000;
		5'b01010: //Estado 10 ---> XORI
			signals = 24'b001000010000011100000000;
		5'b01011: //Estado 11 ---> LUI
			signals = 24'b001000010000000100000000;
		5'b01100: //Estado 12  ---> BEQ / B / BGEZ / BGEZAL / BGTZ / BNE
			signals = 24'b000000000000000001011000;
		5'b01101: //Estado 13 ---> J / JAL
			signals = 24'b001000010000011000000000;
		5'b01110: //Estado 14 ---> LW / LH / LHU / LB / LBU ---> calcular eff-address
			signals = 24'b010000000000001101000000;
		5'b01111: //Estado 15 ---> LOAD_INT ---> Tomar eff-address del LOAD y escribir en el Register file el resultado en la direccion RT
			signals = 24'b001110010010000000000010;
		5'b10000: //Estado 16 ---> 
			signals = 24'b001110010010000000000010;
		5'b10001: //Estado 17 ---> 
			signals = 24'b001110010010000000000010;
		5'b10010: //Estado 18 ---> SD / SW / SH / SB ---> calcular eff-address
			signals = 24'b010000000000001101000000;
		5'b10011: //Estado 19  ---> STORE_INT Tomar eff-address del STORE y escribir en el RAM el valor de RT
			signals = 24'b00000001001000000000001;
		5'b10100: //Estado 20 
			signals = 24'b010000000000000110000001;
		5'b10101: //Estado 21 
			signals = 24'b010000000000000110000001;
		5'b10110: //Estado 22 
			signals = 24'b010000000000000110000001;
		5'b10111: //Estado 23 
			signals = 24'b010000000000000110000001;
		5'b11000: //Estado 24 
			signals = 24'b010000000000000110000001;
		5'b11001: //Estado 25 
			signals = 24'b010000000000000110000001;
		5'b11010: //Estado 26
			signals = 24'b010000000000000110000001;
		default: //Undefined
			signals = 24'b000000000000000000000000;
	endcase
endmodule

module NextStateDecoder(output reg [4:0] next, input [4:0] prev, input [5:0] opcode, input MOC, reset);
	always@(prev, opcode, MOC)
	if (reset) begin
		next = 5'b00000;
	end else begin
		//$display("OpCode ---------->  %b", opcode);
		//$display("MOC  ---------->  %b", MOC);
		case(prev)
			5'b00000: //State 0
			next = 5'b00001;
			5'b00001: //State 1
			next = 5'b00010;
			5'b00010: //State 2
			next = 5'b00011;
			5'b00011: //State 3
			if(MOC)
				next = 5'b00100;
			else
				next = 5'b00011;
			5'b00100: //State 4
				case(opcode)
					6'b000000: //Go to State 5 ---> TYPE R aritmetic ops
						next = 5'b00101;
					6'b001000: //Go to State 6 ---> ADDI 
						next = 5'b00110;
					6'b001001: //Go to State 6 ---> ADDIU
						next = 5'b00110;
					6'b001010: //Go to State 7 ---> SLTI
						next = 5'b00111;
					6'b001011: //Go to State 7 ---> SLT
						next = 5'b00111;
					6'b001100: //Go to State 8 ---> ANDI
						next = 5'b01000;
					6'b001101: //Go to State 9 ---> ORI
						next = 5'b01001;
					6'b001110: //Go to State 10 ---> XORI
						next = 5'b01010;
					6'b001111: //Go to State 11 ---> LUI
						next = 5'b01011;
					6'b000100: //Go to State 12 ---> BEQ / B
						next = 5'b01100;
					6'b000001: //Go to State 12 ---> BGEZ / BGEZAL
						next = 5'b01100;
					6'b000111: //Go to State 12 ---> BGTZ
						next = 5'b01100;
					6'b000110: //Go to State 12 ---> BLEZ
						next = 5'b01100;
					6'b000101: //Go to State 12 ---> BNE
						next = 5'b01100;
					6'b000010: //Go to State 13 ---> J
						next = 5'b01101;
					6'b000011: //Go to State 13 ---> JAL
						next = 5'b01101;
					6'b100011: //Go to State 14 ---> LW
						next = 5'b01110;
					6'b100001: //Go to State 14 ---> LH
						next = 5'b01110;
					6'b100101: //Go to State 14 ---> LHU
						next = 5'b01110;
					6'b100000: //Go to State 14 ---> LB
						next = 5'b01110;
					6'b100100: //Go to State 14 ---> LBU
						next = 5'b01110;
					6'b111111: //Go to State 18 ---> SD
						next = 5'b10010;
					6'b101011: //Go to State 18 ---> SW
						next = 5'b10010;
					6'b101001: //Go to State 18 ---> SH
						next = 5'b10010;
					6'b101000: //Go to State 18 ---> SB
						next = 5'b10010;
				endcase
			5'b00101: //State 5 
			next = 5'b00001;
			5'b00110: //State 6
			next = 5'b00001; 
			5'b00111: //State 7
			next = 5'b00001;
			5'b01000: //State 8
			next = 5'b00001;
			5'b01001: //State 9
			next = 5'b00001;
			5'b01010: //State 10
			next = 5'b00001;
			5'b01011: //State 11
			next = 5'b00001;
			5'b01100: //State 12
			next = 5'b10110;
			5'b01101: //State 13
			next = 5'b00001;
			5'b01110: //State 14
				case(opcode)
					6'b100011: //Go to State 15 (LW)
					next = 5'b01111;
					6'b100001: //Go to State 15 (LH?)
					next = 5'b01111;
					6'b100101: //Go to State 15 (LHU?)
					next = 5'b01111;
					6'b100000: //Go to State 23 (LB)
					next = 5'b10111;
					6'b100100: //Go to State 23 (LBU)
					next = 5'b10111;
				endcase
			5'b01111: //State 15 (Load Word)
			next = 5'b10000;
			5'b10000: //State 16 (Load Word)
			if(MOC)
				next = 5'b10001; //If MOC, go to State 17
			else
				next = 5'b10000; //Else, continue waiting for MOC
			5'b10001: //State 17
				next = 5'b00001;
			5'b10010: //State 18
			case(opcode)
				6'b111111: //Go to State 19 (SD?)
				next = 5'b10011;
				6'b101011: //Go to State 19 (SW)
				next = 5'b10011;
				6'b101001: //Go to State 19 (SH?)
				next = 5'b10011;
				6'b101000: //Go to State 25 (SB)
				next = 5'b11001;
			endcase
			5'b10011: //State 19 (Store Word)
				next = 5'b10100;
			5'b10100: //State 20 (Store Word)
			if(MOC)
				next = 5'b10101; //If MOC, go to State 21
			else
				next = 5'b10100; //Else, continue waiting for MOC
			5'b10101: //State 21
				next = 5'b00001;
			5'b10110: //State 22
			next = 5'b00001;
			5'b10111: //State 23 (Load Byte)
			next = 5'b11000;
			5'b11000: //State 24 (Load Byte)
			if(MOC)
				next = 5'b10001; //If MOC, go to State 17
			else
				next = 5'b11000;
			5'b11001: //State 25 (Store Byte)
			next = 5'b11010;
			5'b11010: //State 26 (Store Byte)
			if(MOC)
				next = 5'b10101; //If MOC, go to State 21
			else
				next = 5'b11010;
		endcase
	end
endmodule

// Control Unit
module ControlUnit(output wire [23:0] signals, output [4:0] state, input [5:0] opcode, input MOC, reset, clock);
	wire [4:0] state, next;
	StateRegister SR(state, next, clock, reset);
	ControlSignalEncoder CSE(signals, state);
	NextStateDecoder NSD(next, state, opcode, MOC, reset);
endmodule


