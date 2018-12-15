module CPU(output reg done, input clk);
wire [31:0] MEM_input, IR, PA, PB, CondA, CondB, RegFAOut, sign_out, zero_out, ALUAMux, ALUBMux, PC_out, IMMExt2_Out, PC_in, ALUMout, ALU_Out, Data_Out, DataMuxOut, Write_Data, PCB_out;									
wire[15:0] sign_in, zero_in;									
wire[8:0] MAR_out;									
wire[4:0] RegC;									
wire[3:0] AluOp;									
wire[1:0] AluinBMux, AluAInput, CondBox;
reg [7:0] test_ram_out;
reg load_PC;
reg[8:0] PC_mar;
integer fi, fo, code, i; reg [31:0] data; //for pre_loading									
wire Reg_Write, MuxInReg, ImmExtenderMux, ImmExtenderMux2, RamEnable, PCinMux, MemReg, InstructionRegInMux, ConInMuxB, ConInMuxA, PCbranch, load_inst, ramDone, aluOverflow, clk, clr, COut, AlmostDone;	
control_unit cu (Reg_Write, MuxInReg, ImmExtenderMux, ImmExtenderMux2, RamEnable, PCinMux, MemReg, InstructionRegInMux, ConInMuxB, ConInMuxA, PCbranch, load_inst, AluOp, AluinBMux, AluAInput, ramDone, aluOverflow, CondBox, IR, clk,clr );
// monitors
initial begin
    $monitor("ram_data_out=%b--PC=%b", Data_Out, PC_out);
    $monitor("memInput=%b, MemReg=%b", MEM_input, MemReg);
    $monitor("IR=%b", IR);
end

//for clk change
always @ (clk) begin
    //$display("PC=%d, MAR=%b, ALU_out=%b", PC_out, MAR_out, ALU_Out);
    //$display("data_out=%b", Data_Out);
    //$display("memInput=%b, MemReg=%b", MEM_input, MemReg);
    //$display("IR=%b", IR);
    //$display("condbox=%b", CondBox);
end

always @(PC_out) begin
    $display("PC=%d", PC_out);
end
//PREGARGAR MEMORIA
initial begin
    fi = $fopen("testcode_mips1.txt", "r");
    PC_mar = 9'd0;
    done = 0;
    while (!$feof(fi)) begin
        code = $fscanf(fi, "%b", data);
        //$display("code = $b, data = %b", code, data);
        ram.Mem[PC_mar] = data;
        #5 test_ram_out = ram.Mem[PC_mar];
        $display("space=%d, memory_data=%b", PC_mar,test_ram_out);
        PC_mar = PC_mar + 1;
    end
    $fclose(fi);
    done = 1;
end

//DATAPATH
register_file rf (PA, PB, IR[25:21], IR[20:16], RegC, Write_Data, Reg_Write, clk);
ALU alu_ (ALU_Out, COut, aluOverflow, ALUAMux, ALUBMux, AluOp);
ram512x8 ram (Data_Out, RamEnable, MAR_out, PB, IR[31:26], AlmostDone, ramDone, load_inst);
//MAGIC BOXES
conditional_box cb (CondBox, CondA, CondB);
sign_extender se (sign_out, sign_in);
zero_extender ze (zero_out, zero_in);
pc_branch pcb(PCB_out, IMMExt2_Out, PCbranch);
// MUXES AND DECODERS
multiplexer2 condmuxA (CondA, CondInMux, PA, 32'd0);
multiplexer2 condmuxB (CondB, CondInMuxB, PB,  32'd0);
multiplexer2 immext2 (IMMExt2_Out, ImmExtenderMux2, sign_out, zero_out) ;
multiplexer2 Data_Mux (DataMuxOut, MemReg,ALUMout, Data_Out);

multiplexer4 ALUA (ALUAMux, AluAInput,PC_out, 32'd1, PA, 32'b0);
multiplexer4 ALUB (ALUBMux, AluinBMux, PB, PCB_out, 32'd0, 32'd4);

multiplexer2_5 regFC (RegC, MuxInReg , IR[20:16], IR[15:11]);

decoder2 instregmux (Write_Data, MEM_input, InstructionRegInMux, DataMuxOut); 
decoder2 PCMux (PC_in, ALUMout, PCinMux, ALU_Out);
decoder2_16 immext (sign_in, zero_in, ImmExtenderMux, IR[15:0]);

// registers
register_32bits InstReg (IR, MEM_input, InstructionRegInMux, clk); 
register_32bits ProgCount (PC_out, PC_in, !PCinMux, clk);
Memory_access_register MAR (MAR_out, ALUMout, RamEnable, clk);

endmodule