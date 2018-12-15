`include "mips.v"

module mipsTester();
  // initialize clock
  reg clk, reset;
  wire [31:0] instrW, outW;
  wire [8:0] mar;
  reg [31:0] instr, out;


  initial clk = 0; // initial block executes once during simulation
  always #10 clk = ~clk;
  initial #10000 $stop;

  MipsProcessor mips(outW, reset, clk);

  initial begin
  // reset mips and set to 0 on the next tick
  reset = 1;
  #10 reset = 0;
  end

  initial begin
    // // out = outW;
    // $display("time        clk                        out         ");
    // $monitor("%4d         %b                     %b     ", $time, clk, outW) ;
  end



endmodule
