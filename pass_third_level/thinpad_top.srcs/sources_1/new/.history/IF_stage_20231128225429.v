`include "mycpu.h"

module if_stage(
    input                          clk            ,
    input                          reset          ,
    //allwoin
    input                          ds_allowin     ,

    //  input          wire                ms_to_fs_ld_valid,
    //  brbus
    input  [`BR_BUS_WD       -1:0] br_bus         ,
    input                         if_stall       ,
    output                          if_to_ds_stall,
    //to ds
    output                         fs_to_ds_valid ,
    output [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus   ,
    // inst sram interface
    output        inst_sram_en   ,
    output [ 3:0] inst_sram_we  ,
    output [31:0] inst_sram_addr ,
    output [31:0] inst_sram_wdata,
    input  [31:0] inst_sram_rdata
);
assign if_to_ds_stall = if_stall;
reg         fs_valid;//表示当前阶段是否有效
wire        fs_ready_go;
wire        fs_allowin; //并确定 IF 阶段是否接受取指令的请求
wire        to_fs_valid;//表示是否可以将指令传递到下一个阶段


wire [31:0] seq_pc;
wire [31:0] nextpc;

wire        br_taken;
wire [31:0] br_target;

assign {br_taken,br_target} = br_bus;  

wire [31:0] fs_inst;
reg  [31:0] fs_pc;
assign fs_to_ds_bus = {fs_inst ,
                       fs_pc   };

// pre-IF stage
assign to_fs_valid  = ~reset;
// assign seq_pc       = if_stall?seq_pc:fs_pc + 3'h4;
assign seq_pc = fs_pc + 3'h4;
reg br_TAKEN;
reg [31:0] Nextpc;
always @(posedge clk) begin
    if (reset) 
        br_TAKEN <= 1'b0;
        Nextpc <= 32'h0;
    end else if (if_stall&&br_taken) begin
        br_TAKEN <= 1'b1;
        Nextpc <= nextpc;
    end else begin
        br_TAKEN <= 1'b0;
        Nextpc <= br_target;
    end
end


// assign nextpc       = br_taken||br_TAKEN ? br_target : seq_pc; 
assign nextpc = br_TAKEN?Nextpc:br_taken?br_target:seq_pc;
// IF stage
// assign fs_ready_go    = ms_to_fs_ld_valid?1'b0:1'b1;
assign fs_ready_go    = ~if_stall;
assign fs_allowin     = !fs_valid || (fs_ready_go && ds_allowin);
assign fs_to_ds_valid =  fs_valid && fs_ready_go;


always @(posedge clk) begin
    if (reset) 
        fs_valid <= 1'b0;
    else if (fs_allowin) 
        fs_valid <= to_fs_valid;
    
    if (reset) 
        fs_pc <= 32'h7fff_fffc;
        // fs_pc <= 32'h8000_0000;  
    else if (to_fs_valid && fs_allowin) 
        fs_pc <= nextpc;
end
// 
assign inst_sram_en    = to_fs_valid && fs_allowin&&~if_stall;
assign inst_sram_we   = 4'h0;
assign inst_sram_addr  = fs_pc;
assign inst_sram_wdata = 32'b0;

assign fs_inst         = inst_sram_rdata;

endmodule