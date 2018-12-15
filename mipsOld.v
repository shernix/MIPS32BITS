module MipsProcessor(output [31:0]DataOut, input reset, clock);

//ProgramCounter
reg [8:0] PC = 0;
//Control Unit Variables
wire [13:0]CUOut;
wire [5:0]CUInput;
wire MOC;

//Control Unit Outputs
wire MDRLd = CUOut[0];
wire MARLd = CUOut[1];
wire MOV = CUOut[2];
wire regWrite = CUOut[3];
wire ALUsrc = CUOut[4];
wire MemWrite = CUOut[5];
wire AluOp2 = CUOut[6];
wire AluOp1 = CUOut[7];
wire AluOp0 = CUOut[8]; 
wire MemToReg = CUOut[9];
wire MemRead = CUOut[10];
wire Branch = CUOut[11];
wire Jump = CUOut[12];
wire RegDst = CUOut[13];



//Register File Variables
wire [31:0] OutRF_InAluA, OutRF_InAluSrcB;//B va para el mux
wire [4:0] outputSelectorA, outputSelectorB;

//ALU Source Mux Variables
wire [31:0]outAluSrc_InAlu;
wire [31:0]singExtended;

//Register Destination Mux Variables
wire [4:0] destination;
wire [4:0] IR20_16, IR15_11;

//ALU Variables
wire [31:0]AluOut;
wire C,V;
wire [5:0] operation;

//MAR Variables
wire [8:0]MAROutput;

//MDR Variables
wire [31:0]MDROuput;

//RAM Variables
wire [31:0]RAMDataOut;
assign DataOut = RAMDataOut;

//Memory to Register Mux Variables
wire [31:0]MemtoRegMuxOut;

//Sign Extender Variables
wire [15:0] dataIn;

//ALU Control Variables
wire [5:0] funct;
wire [31:0] instruction = RAMDataOut;

//Instruction to corresponding variables
assign CUInput = instruction[31:26];
assign outputSelectorA = instruction[25:21];
assign outputSelectorB = instruction[20:16];
assign IR20_16 = instruction[20:16];
assign IR15_11 = instruction[15:11];
assign dataIn = instruction[15:0];
		



//Datpath
RegisterFile RegF(OutRF_InAluA, OutRF_InAluSrcB, MemtoRegMuxOut, destination, outputSelectorA, outputSelectorB, regWrite, clock);
ALUSrcMux ALUsrcMux1(outAluSrc_InAlu, OutRF_InAluSrcB, singExtended, ALUsrc);
RegDstMux RegDstMux1(destination, IR20_16, IR15_11, RegDst);
Alu_32bits alu1(AluOut, C, V, operation, OutRF_InAluA, outAluSrc_InAlu);
MAR mar1(MAROutput, AluOut, MARLd, clock);
MDR mdr1(MDROuput, OutRF_InAluSrcB, MDRLd, clock);
ram512x8 RAM(RAMDataOut, MOC, MOV, MemRead, MemWrite, MAROutput, MDROuput);
MemToRegMux MemToRegMux1(MemtoRegMuxOut, RAMDataOut, AluOut, MemToReg);
Extender singExtender(singExtended, dataIn);
ALUControl ALUControl(operation, funct, AluOp2, AluOp1, AluOp0);
ControlUnit CU(CUOut,CUInput, reset, clock, MOC);
endmodule //end

//MAR Module
module MAR(output reg [8:0] Qs, input [31:0]  Ds, input Ld, CLK);

  initial begin
  	Qs= 32'd0;
		//$display("MARLd ----->  %b", Ld);
  end

always@(posedge CLK)
	if (Ld) begin
		Qs<=Ds;
		$display("MAR ----->  %b", Qs);
		//$display("MARLd ----->  %b", Ld);
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

//Register File
module RegisterFile(output reg [31:0] OA, OB, input [31:0] dataIn, input [4:0] destination, regAddressA, regAddressB, input write, clock);
	reg [31:0] registerFile [31:0];
		initial begin
		registerFile[0] = 32'b00100100000000010000000000101100;
		registerFile[1] = 32'b10010000001000100000000000000000;
		registerFile[2] = 32'b10010000001000110000000000000010;
		registerFile[3] = 32'b00000000000000000010100000100001;
		registerFile[4] = 32'b00000000101000100010100000100001;
		registerFile[5] = 32'b00100100011000111111111111111111;
		registerFile[6] = 32'b00011100011000001111111111111101;
		registerFile[7] = 32'b00000000000000000000000000000000;
		registerFile[8] = 32'b10100000001001010000000000000001;
		registerFile[9] = 32'b00010000000000000000000000000010;
		registerFile[10] = 32'b00000000000000000000000000000000;
		registerFile[11] = 32'b00011001000001010000011100000100;
		registerFile[12] = 32'b00010000000000001111111111111111;
		registerFile[13] = 32'b00000000000000000000000000000000;
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
module ALUSrcMux(output reg [31:0] data, input [31:0] regData, extended, input ALUSrc);
	always@(ALUSrc)
	if(ALUSrc)
		data = extended;
	else
		data = regData;
endmodule

//Register Destination Multiplexer
module RegDstMux(output reg [4:0] destination, input [4:0] IR20_16, IR15_11, input RegDst);
	always@(RegDst)
	if(RegDst)
		destination = IR15_11;
	else
		destination = IR20_16;
endmodule

//ALU
module Alu_32bits(output reg [31:0] Y,output reg C,V, input[5:0]s, input[31:0] A,B);
    integer i;
    integer c = 0; //variable para manejar el conteo de los unos consecutivos.
    integer c2 = 0; //variable para manejar el conteo de los ceros consecutivos.
    integer flag = 0;
    always@(s,A,B)
    begin
    case(s)

    6'b100100:begin //bitwise and
    V = 1'b0;
    C = 1'b0;
    Y = A & B;

    end

    6'b100101:begin //bitwise or
    V = 1'b0;
    C = 1'b0;
    Y = A | B;

    end

    6'b100111:begin //bitwise nor
    V = 1'b0;
    C = 1'b0;
    Y = ~(A | B);

    end

    6'b100110:begin //bitwise ex-or
    V = 1'b0;
    C = 1'b0;
    Y = A ^ B;

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
    end

    6'b000000://shift left logico
    begin
    V = 1'b0;
    assign C = 1'b0;
    {C,Y}=A<<B;
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
    end
endmodule

//Memory with MemRead and MemWrite
module ram512x8 (output reg [31:0] DataOut, output reg MOC,
	input MOV, MemRead, MemWrite, input [8:0] Address, input [31:0] DataIn);

	integer fileIn, code; reg [31:0] data;
		reg[8:0] loadPC;
		reg [7:0] test_ram_out;
	initial begin
		fileIn = $fopen("testcode.txt", "r");
		loadPC = 9'd0;
		//done = 0;
		while (!$feof(fileIn)) begin
				code = $fscanf(fileIn, "%b", data);
				// $display("code = $b, data = %b", code, data);
				RAM.Mem[loadPC] = data;
				test_ram_out = RAM.Mem[loadPC];
				//$display("space=%d, memory_data=%b", loadPC,test_ram_out);
				loadPC = loadPC + 1;
		end
		$fclose(fileIn);
		MOC = 1;
	end

	reg [7:0] Mem[0:511];

	always @(posedge MOV) //Whenever Enable and/or MOV is active
	if(MOV) //If MOV=1, proceed with ReadWrite
		begin
		if(MemRead) //Read Operation (1)
			begin
			//DataOut = {Mem[Address], {Mem[Address+1], {Mem[Address+2], Mem[Address+3]}}}; //{Mem[Address], Mem[Address+1], Mem[Address+2], Mem[Address+3]};
			DataOut = {Mem[Address], Mem[Address+1], Mem[Address+2], Mem[Address+3]};
				$display("instruction ----->  %b", DataOut);
			MOC = 1'b1;
			#2 MOC = 1'b0;
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
			MOC = MOV;
			end
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

//16 to 32 Extender
module Extender(output [31:0] dataOut, input [15:0] dataIn);
	assign dataOut = {16'b0000000000000000, dataIn};
endmodule

//ALU Control
module ALUControl(output reg [5:0] operation, input [5:0] funct, input ALUOP2, ALUOP1, ALUOP0);
	reg [2:0] aluop;
	initial begin
	aluop = {ALUOP2, {ALUOP1, ALUOP0}};
	case(aluop)
		3'b000: //Add
			operation = 6'b100000;
		3'b001: //Sub
			operation = 6'b100010;
		3'b010: //FUNCT
			operation = funct;
		3'b011: //Shift
			operation = 6'b111111;
		3'b100: //SLT
			operation = 6'b101011;
		3'b101: //AND
			operation = 6'b100100;
		3'b110: //OR
			operation = 6'b100101;
		3'b111: //XOR
			operation = 6'b100110;
	endcase
	end
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

		$display("state ----->  %b", state);

		end
endmodule

//Control Signal Encoder
module ControlSignalEncoder(output reg [13:0] signals, input [4:0] state);
	/*
	signals[13] = RegDst
	signals[12] = Jump
	signals[11] = Branch
	signals[10] = MemRead
	signals[9] = MemToReg
	signals[8] = ALUOp0
	signals[7] = AlUOp1
	signals[6] = AlUOp2
	signals[5] = MemWrite
	signals[4] = ALUSrc
	signals[3] = RegWrite
	signals[2] = MOV
	signals[1] = MARLd
	signals[0] = MDRLd
	*/
	always@(state)
	case(state)
		5'b00000: //Estado 0
		signals = 14'b00000000000000;
		5'b00001: //Estado 1 (Fetch 1)
		signals = 14'b00000001000010;
			5'b00010: //Estado 2 (Fetch 2)
			signals = 14'b10001001101100;
			5'b00011: //Estado 3 (Fetch 3)
			signals = 14'b00010001101100;
			5'b00100: //Estado 4
			signals = 14'b00010000000100;
			5'b00101: //Estado 5 (R-Type)
			signals = 14'b10000100001000;
			5'b00110: //Estado 6 (ADDI)
			signals = 14'b00000101011000;
			5'b00111: //Estado 7 (SLTI)
			signals = 14'b00000110011000;
			5'b01000: //Estado 8 (ANDI)
			signals = 14'b10000111011000;
			5'b01001: //Estado 9 (ORI)
			signals = 14'b00000000011000;
			5'b01010: //Estado 10 (XORI)
			signals = 14'b00000001011000;
			5'b01011: //Estado 11 (LUI)
			signals = 14'b00000011011000;
			5'b01100: //Estado 12 (BEQ)
			signals = 14'b00100010000000;
			5'b01101: //Estado 13 (Jump)
			signals = 14'b01000000000000;
		5'b01110: //Estado 14 (Load 1)
		signals = 14'b00000101010010;
		5'b01111: //Estado 15 (Load 2)
		signals = 14'b00010101000100;
		5'b10000: //Estado 16 (Load 3)
		signals = 14'b00010101000110;
		5'b10001: //Estado 17 (Load 4)
		signals = 14'b00001101001000;
		5'b10010: //Estado 18 (Store 1)
		signals = 14'b00000101010011;
		5'b10011: //Estado 19 (Store 2)
		signals = 14'b00000101100100;
		5'b10100: //Estado 20 (Store 3)
		signals = 14'b00000101110111;
		5'b10101: //Estado 21 (Store 4)
		signals = 14'b00000101000000;
		default: //Undefined
		signals = 14'b11111111111111;
	endcase
endmodule

//Next State Decoder
module NextStateDecoder(output reg [4:0] next, input [4:0] prev, input [5:0] opcode, input MOC, input reset);
	always@(prev,opcode, MOC, reset, next)

	if (reset) begin
		next = 5'b00000;
	end else begin
		$display("OpCode ----->  %b", opcode);
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
					6'b000000: //Go to State 5
						//MOC =1;
						next = 5'b00101;
					6'b001000: //Go to State 6
					next = 5'b00110;
					6'b001001: //Go to State 6
					next = 5'b00110;
					6'b001010: //Go to State 7
					next = 5'b00111;
					6'b001011: //Go to State 7
					next = 5'b00111;
					6'b001100: //Go to State 8
					next = 5'b01000;
					6'b001101: //Go to State 9
					next = 5'b01001;
					6'b001110: //Go to State 10
					next = 5'b01010;
					6'b001111: //Go to State 11
					next = 5'b01011;
					6'b000100: //Go to State 12
					next = 5'b01100;
					6'b000001: //Go to State 12
					next = 5'b01100;
					6'b000111: //Go to State 12
					next = 5'b01100;
					6'b000110: //Go to State 12
					next = 5'b01100;
					6'b000101: //Go to State 12
					next = 5'b01100;
					6'b000010: //Go to State 13
					next = 5'b01101;
					6'b000011: //Go to State 13
					next = 5'b01101;
					6'b100011: //Go to State 14
					next = 5'b01110;
					6'b100001: //Go to State 14
					next = 5'b01110;
					6'b100101: //Go to State 14
					next = 5'b01110;
					6'b100000: //Go to State 14
					next = 5'b01110;
					6'b100100: //Go to State 14
					next = 5'b01110;
					6'b111111: //Go to State 18
					next = 5'b10010;
					6'b101011: //Go to State 18
					next = 5'b10010;
					6'b101001: //Go to State 18
					next = 5'b10010;
					6'b101000: //Go to State 18
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
			next = 5'b00001;
			5'b01101: //State 13
			next = 5'b00001;
			5'b01110: //State 14
			next = 5'b01111;
			5'b01111: //State 15
			next = 5'b10000;
			5'b10000: //State 16
			if(MOC)
				next = 5'b10001; //If MOC, go to State 17
			else
				next = 5'b10000; //Else, continue waiting for MOC
			5'b10001: //State 17
				next = 5'b00001;
			5'b10010: //State 18
				next = 5'b10011;
			5'b10011: //State 19
				next = 5'b10100;
			5'b10100: //State 20
			if(MOC)
				next = 5'b10101; //If MOC, go to State 21
			else
				next = 5'b10100; //Else, continue waiting for MOC
			5'b10101: //State 21
				next = 5'b00001;
		endcase
	end

endmodule

// Control Unit
module ControlUnit(output wire [13:0] signals, input [5:0] opcode, input reset, clock, MOC);
	wire [4:0] state, next;
	StateRegister SR(state, next, clock, reset);
	ControlSignalEncoder CSE(signals, state);
	NextStateDecoder NSD(next, state, opcode, MOC, reset);

endmodule