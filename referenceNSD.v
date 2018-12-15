module NextStateDecoder(output reg [4:0] next, input [4:0] prev, input [5:0] opcode, input MOC);
	always@(prev, opcode)
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
			6'b100011: //Go to State 14 (LW)
			next = 5'b01110;
			6'b100001: //Go to State 14 (LH?)
			next = 5'b01110;
			6'b100101: //Go to State 14 (LHU?)
			next = 5'b01110;
			6'b100000: //Go to State 14 (LB)
			next = 5'b01110;
			6'b100100: //Go to State 14 (LBU)
			next = 5'b01110;
			6'b111111: //Go to State 18 (SD?)
			next = 5'b10010;
			6'b101011: //Go to State 18 (SW)
			next = 5'b10010;
			6'b101001: //Go to State 18 (SH?)
			next = 5'b10010;
			6'b101000: //Go to State 18 (SB)
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
endmodule