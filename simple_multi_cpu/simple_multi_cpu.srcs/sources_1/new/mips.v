module mips( clk, reset );
   input   clk;
   input   reset;
   
   wire               zero          ;
   wire   [31:0]      PC            ;
   wire   [31:0]      ALUOut        ;
   // wire   [1 :0]      ALUOp         ;
   wire   [31:0]      pc_im_mux_out;
   wire   [4 :0]      ir_reg_mux_out;
   wire   [31:0]      im_reg_mux_out;
   wire   [31:0]      alu_pc_mux_out;
   wire   [31:0]      reg_alu_mux_out;
   wire   [31:0]      reg_alu_mux2_out;
   // wire   [31:0]      muxout1  ;
   wire   [31:0]      im_dout,dm_dout;
   wire   [31:0]      busA,busB;
   wire   [31:0]      in,IRout;
   //wire   [31:0]      rdata1,rdata2;

   wire   [31:0]      Imm32   ;
   wire   [2 :0]      opcode  ;   
   wire               PCWr    ;
   wire               PCWrCond;
   wire               IorD    ;
   wire               MemWr   ;
   wire               IRWr    ;
   wire               RegDst  ;
   wire               RegWr   ;
   wire               ALUSelA ;
   wire   [1 :0]      ALUSelB ;
   wire               PCSrc   ;
   wire               BrWr    ;
   wire   [1 :0]      ExtOp   ;
   wire               MemtoReg;
   wire   [1 :0]      ALUOp   ;
   wire   [31:0]      tar_out ;

   PC U_PC(
      .PCWr  ( PCWr |( PCWrCond & zero )),
      .NPC   ( alu_pc_mux_out   ),
      .clk   ( clk   ),
      .reset ( reset ),
      .PC    ( PC    )
   );

   // im_4k U_IM ( 
   //    .addr(pc_im_mux_out[11:2]), 
   //    .dout(im_dout)
   // );
   im_4k U_IM(
      .clk   ( clk   ),
      .RAdr  ( pc_im_mux_out[11:2]  ),
      .WrAdr ( ALUOut[11:2] ),
      .MemWr ( MemWr ),
      .Din   ( busB   ),
      .Dout  ( im_dout  )
   );

   // dm_4k U_DM ( 
   //    .addr(ALUOut[11:2]),
   //    .din(busB), 
   //    .DMWr(MemWr), 
   //    .clk(clk), 
   //    .dout(dm_dout)
   // );
   
   Alu U_Alu(
      .A      ( reg_alu_mux_out      ),
      .B      ( reg_alu_mux2_out      ),
      .ALUOp  ( ALUOp  ),
      .ALUOut ( ALUOut ),
      .zero   ( zero   )
   );

   target U_target(
      .clk    ( clk    ),
      .reset  ( reset  ),
      .tar_in ( ALUOut ),
      .BrWr   ( BrWr   ),
      .tar_out  ( tar_out  )
   );


   IR U_IR(
      .clk  ( clk      ),
      .IRWr ( IRWr     ),
      .in   ( im_dout  ),
      .IRout( IRout    )
   );

   regfile U_regfile(
      .clk    ( clk    ),
      .raddr1 ( IRout[25:21] ),
      .rdata1 ( busA ),
      .raddr2 ( IRout[20:16] ),
      .rdata2 ( busB ),
      .RFWr   ( RegWr   ),
      .waddr  ( ir_reg_mux_out ),
      .wdata  ( im_reg_mux_out  )
   );

   ctrl U_ctrl(
      .clk      ( clk      ),
      .reset    ( reset    ),
      .opcode   ( IRout[31:26]   ),
      .PCWr     ( PCWr     ),
      .PCWrCond ( PCWrCond ),
      .IorD     ( IorD     ),
      .MemWr    ( MemWr    ),
      .IRWr     ( IRWr     ),
      .RegDst   ( RegDst   ),
      .RegWr    ( RegWr    ),
      .ALUSelA  ( ALUSelA  ),
      .ALUSelB  ( ALUSelB  ),
      .PCSrc    ( PCSrc    ),
      .BrWr     ( BrWr     ),
      .ExtOp    ( ExtOp    ),
      .MemtoReg ( MemtoReg ),
      .ALUOp    ( ALUOp    )
   );

   mutiplexer32_1 U_PC_IM(
      .s  ( IorD  ),
      .d0 ( PC ),
      .d1 ( ALUOut ),
      .out32_1  ( pc_im_mux_out )
   );

   mutiplexer5_1 U_IR_Regfile(
      .s  ( RegDst  ),
      .d0 ( IRout[20:16] ),
      .d1 ( IRout[15:11] ),
      .out5_1  (  ir_reg_mux_out )
   );

   mutiplexer32_1 U_IM_Regfile(
      .s  (  MemtoReg ),
      .d0 ( ALUOut ),
      .d1 ( im_dout ),
      .out32_1  (  im_reg_mux_out )
   );

   mutiplexer32_1 U_Regfile_Alu_1(
      .s  ( ALUSelA  ),
      .d0 ( PC ),
      .d1 ( busA ),
      .out32_1  ( reg_alu_mux_out  )
   );

   EXT U_EXT(
      .Imm16 ( IRout[15:0] ),
      .ExtOp ( ExtOp ),
      .Imm32 ( Imm32 )
   );

   mutiplexer32_1 U_Alu_PC(
      .s  ( PCSrc  ),
      .d0 ( ALUOut ),
      .d1 ( tar_out ),
      .out32_1  ( alu_pc_mux_out  )
   );

   mutiplexer32_2 U_Regfile_Alu_2(
      .s  ( ALUSelB  ),
      .d0 ( 32'h4 ),
      .d1 ( busB ),
      .d2 ( {Imm32[29:0],2'b00} ),
      .d3 ( Imm32 ),
      .out32_2  ( reg_alu_mux2_out  )
   );

endmodule