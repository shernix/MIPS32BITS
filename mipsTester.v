`include "mips.v"

module mipsTester();
  // initialize clock
  reg clk, reset;
  wire [31:0] instrW, outW;
  wire [8:0] mar;
  reg [31:0] instr, out;

  integer fileIn, code; reg [31:0] data;
	reg [7:0] Mem[0:511];
	reg[8:0] loadPC;
	reg [7:0] test_ram_out;

  initial clk = 0; // initial block executes once during simulation
  always #10 clk = ~clk;
  initial #5500 $stop;

  MipsProcessor mips(outW, reset, clk);

  initial begin
  // reset mips and set to 0 on the next tick
  reset = 1;
  #10 reset = 0;
  end

  initial begin
    while (!$feof(fileIn)) begin
				code = $fscanf(fileIn, "%b", data);
				// $display("code = $b, data = %b", code, data);
				Mem[loadPC] = data;
				test_ram_out = Mem[loadPC];
				//$display("space=%d, memory_data=%b", loadPC, test_ram_out);
				loadPC = loadPC + 1;
		end
  end



endmodule
