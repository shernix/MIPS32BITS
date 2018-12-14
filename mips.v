module MipsProcessor(output [31:0] DataOut, input reset, clock);

	//ProgramCounter
	reg [8:0] program_counter = 0;
	wire [8:0] pcOut;
	wire [8:0] branchOut;

	//Control Unit Variables
	wire [22:0] CUOut;

	//Control Unit Signals
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
	ProgramCounter pc(pcOut, branchOut, pcLd, clock);
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
	BranchMagicBox Branching(branchOut, pcOut, imm16, IR25_21, IR20_16, opcode, B, clock);

	initial begin
    // // out = outW;
     $display("time        clk       State              PC           MAR                        IR                        RamOut");
     $monitor("%4d         %b        %b           %b      %d          %b          %b", $time, clock, stateOut, pcOut, marOut, instructionOut, ramDataOut) ;
  end
endmodule //end

//PC module
module ProgramCounter(output reg [8:0] Qs, input [8:0] Qd, input Ld, CLK);
	initial begin
		Qs= 9'd0;
	end

	always@(posedge CLK)
	if (Ld && CLK) begin
		Qs = Qd;
		// $display("PROGRAM COUNTER = ----------> %b", Qs);
	end
	
endmodule

// output reg [8:0] Qs, input [5:0] opcode, input [15:0] imm16, input[4:0] rs, rt, input Ld, CLK

//Brnach Mgix Box 
module BranchMagicBox(output reg[8:0] out, input [8:0] PC, input [15:0] imm16, input [4:0] rs, rt, input [5:0] opcode, input B, CLK);
always @(PC, imm16, B, CLK) 
	if(CLK && B) begin
		case(opcode)
			6'b000100: begin//BEQ
				if(rs == rt)
					out[8:0] =  PC+4+imm16*4;
				else begin 
					out[8:0] =  PC + 9'd4;
				end
			end	
			6'b000111: begin //BGTZ
				if(rs>0)
					out[8:0] =  PC+4+imm16*4;
				else begin
					out[8:0] =  PC + 9'd4;
				end
			end
			6'b000110: begin //BLEZ
				if(rs<0)
					out[8:0] =  PC+4+imm16*4;
				else begin 
					out[8:0] =  PC + 9'd4;
				end
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

	always@(posedge CLK, Ld)
		if (Ld && CLK) begin
			Qs<=Ds;
		end
endmodule

//MAR Module
module MAR(output reg [8:0] Qs, input [8:0] Ds, input Ld, CLK);
	initial begin
		Qs = 9'd0;
	end

	always@(posedge CLK, Ld)
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

	always @(posedge MOV, MemRead, MemWrite) begin//Whenever Enable and/or MOV is active
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
	always@(regIn)
	case (regIn)
		2'b00: data = aluResult;
		2'b01: data = {23'd0, program_counter} + 32'd8;
		2'b10: data = dataFromRam; 
	endcase
endmodule

// Register A_Input multiplexer 
module RegSrcMux(output reg [4:0] data, input [4:0] IR21_25, input [1:0] regSrc);
	reg LO,HI;
	always@(regSrc)
	case (regSrc)
		2'b00: data = IR21_25;
		// 2'b01: data = LO;
		// 2'b10: data = HI;
	endcase
endmodule

//Register Destination Multiplexer
module RegDstMux(output reg [4:0] destination, input [4:0] IR20_16, IR15_11, HI, LO, R_31, input [2:0]regDst);
	always@(regDst)
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
	always@(posedge clock)
	begin
	if(write)
		begin
		registerFile[destination] = dataIn;
		if(~destination)
			registerFile[destination] = 0;
		end
	OA = registerFile[regAddressA];
	OB = registerFile[regAddressB];
	end
endmodule

//ALU Source Multiplexer
module ALUSrcMux(output reg [31:0] data, input [31:0] regData, extended, input [4:0] sa, input [1:0]aluSrc);
	always@(aluSrc)
	case (aluSrc)
		2'b00: data = extended;
		
		2'b01: data = sa;
			
		2'b10: data = regData;
	endcase
endmodule

//16 to 32 Extender
module Extender(output reg [31:0] dataOut, input [15:0] dataIn);
	always@(dataIn) 
	if (dataIn[15])
		dataOut = {16'b1111111111111111, dataIn}; 
	else
		dataOut = {16'b0000000000000000, dataIn}; 
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
	always@(memToReg)
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
module ControlSignalEncoder(output reg [22:0] signals, input [4:0] state);
	/*
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
			signals = 23'b00000000000000000000000;
		5'b00001: //Estado 1 Instruction FETCH... MAR and IR activated ---> Load PC to MAR
			signals = 23'b00000010010000001000110;
		5'b00010: //Estado 2 
			signals = 23'b00000000010000000000110;
		5'b00011: //Estado 3 PC + 4
			signals = 23'b00000000000000001010000;
		5'b00100: //Estado 4 verificar OPCODE
			signals = 23'b00000000010000000000000;
		5'b00101: //Estado 5 (Logic R-TYPE) ADD, ADDU, SUB, SUBU, SLT, SLTU, AND, OR, NOR, XOR, SLLV, SRAV, SRLV
			signals = 23'b01000011010000000000000;
		5'b00110: //Estado 6 ---> ADDI / ADDIU
			signals = 23'b01000010001000110000000;
		5'b00111: //Estado 7 ---> SLTI / SLT
			signals = 23'b01000010000010000000000;
		5'b01000: //Estado 8 ---> ANDI
			signals = 23'b01000010000010100000000;
		5'b01001: //Estado 9 ---> ORI
			signals = 23'b01000010000011000000000;
		5'b01010: //Estado 10 ---> XORI
			signals = 23'b01000010000011100000000;
		5'b01011: //Estado 11 ---> LUI
			signals = 23'b01000010000000100000000;
		5'b01100: //Estado 12  ---> BEQ / B / BGEZ / BGEZAL / BGTZ / BNE
			signals = 23'b00000010000010100001000;
		5'b01101: //Estado 13 ---> J / JAL
			signals = 23'b01000010000011000000000;
		5'b01110: //Estado 14 ---> LW / LH / LHU / LB / LBU ---> calcular eff-address
			signals = 23'b10000000000001101000000;
		5'b01111: //Estado 15 ---> LOAD_INT ---> Tomar eff-address del LOAD y escribir en el Register file el resultado en la direccion RT
			signals = 23'b01110010010000000000010;
		5'b10000: //Estado 16 ---> 
			signals = 23'b01110010010000000000010;
		5'b10001: //Estado 17 ---> 
			signals = 23'b01110010010000000000010;
		5'b10010: //Estado 18 ---> SD / SW / SH / SB ---> calcular eff-address
			signals = 23'b10000000000001101000000;
		5'b10011: //Estado 19  ---> STORE_INT Tomar eff-address del STORE y escribir en el RAM el valor de RT
			signals = 23'b0000001001000000000001;
		5'b10100: //Estado 20 
			signals = 23'b10000000000000110000001;
		5'b10101: //Estado 21 
			signals = 23'b10000000000000110000001;
		5'b10110: //Estado 22 
			signals = 23'b10000000000000110000001;
		5'b10111: //Estado 23 
			signals = 23'b10000000000000110000001;
		5'b11000: //Estado 24 
			signals = 23'b10000000000000110000001;
		5'b11001: //Estado 25 
			signals = 23'b10000000000000110000001;
		5'b11010: //Estado 26
			signals = 23'b10000000000000110000001;
		default: //Undefined
			signals = 23'b00000000000000000000000;
	endcase
endmodule

module NextStateDecoder(output reg [4:0] next, input [4:0] prev, input [5:0] opcode, input MOC, reset);
	always@(prev, opcode)
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
module ControlUnit(output wire [22:0] signals, output [4:0] state, input [5:0] opcode, input MOC, reset, clock);
	wire [4:0] state, next;
	StateRegister SR(state, next, clock, reset);
	ControlSignalEncoder CSE(signals, state);
	NextStateDecoder NSD(next, state, opcode, MOC, reset);
endmodule


