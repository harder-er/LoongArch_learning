 module mips_tb();
    
   reg clk, reset;
    
   mips U_MIPS(
      .clk(clk), .reset(reset)
   );
    
   initial begin
      $readmemh( "code.txt" , U_MIPS.U_IM.imem ) ;
      //$monitor("PC = 0x%8X, IR = 0x%8X", U_MIPS.U_PC.PC, U_MIPS.instr ); 
      clk = 1 ;
      reset = 0 ;
      #5 ;
      reset = 1 ;
      #20 ;
      reset = 0 ;
   end
   
   always
	   #(50) clk = ~clk;
   
endmodule
