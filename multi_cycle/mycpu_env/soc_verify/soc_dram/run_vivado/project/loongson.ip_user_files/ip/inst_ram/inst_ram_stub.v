// Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2019.2 (lin64) Build 2708876 Wed Nov  6 21:39:14 MST 2019
// Date        : Tue Jan 23 12:12:29 2024
// Host        : zgh-Legion-Y7000-2019-PG0 running 64-bit Ubuntu 22.04.3 LTS
// Command     : write_verilog -force -mode synth_stub
//               /home/zgh/learn_files/LoongArch_learning/exp6/mycpu_env/soc_verify/soc_dram/rtl/xilinx_ip/inst_ram/inst_ram_stub.v
// Design      : inst_ram
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7a200tfbg676-1
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* x_core_info = "dist_mem_gen_v8_0_13,Vivado 2019.2" *)
module inst_ram(a, d, clk, we, spo)
/* synthesis syn_black_box black_box_pad_pin="a[14:0],d[31:0],clk,we,spo[31:0]" */;
  input [14:0]a;
  input [31:0]d;
  input clk;
  input we;
  output [31:0]spo;
endmodule
