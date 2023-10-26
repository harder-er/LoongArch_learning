#!/bin/sh

# 
# Vivado(TM)
# runme.sh: a Vivado-generated Runs Script for UNIX
# Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
# 

if [ -z "$PATH" ]; then
  PATH=/home/tools/Vivado/2019.2/ids_lite/ISE/bin/lin64:/home/tools/Vivado/2019.2/bin
else
  PATH=/home/tools/Vivado/2019.2/ids_lite/ISE/bin/lin64:/home/tools/Vivado/2019.2/bin:$PATH
fi
export PATH

if [ -z "$LD_LIBRARY_PATH" ]; then
  LD_LIBRARY_PATH=
else
  LD_LIBRARY_PATH=:$LD_LIBRARY_PATH
fi
export LD_LIBRARY_PATH

HD_PWD='/home/zgh/learn_files/vivado_ex/simple_multi_cpu/simple_multi_cpu.runs/synth_1'
cd "$HD_PWD"

HD_LOG=runme.log
/bin/touch $HD_LOG

ISEStep="./ISEWrap.sh"
EAStep()
{
     $ISEStep $HD_LOG "$@" >> $HD_LOG 2>&1
     if [ $? -ne 0 ]
     then
         exit
     fi
}

EAStep vivado -log mips.vds -m64 -product Vivado -mode batch -messageDb vivado.pb -notrace -source mips.tcl
