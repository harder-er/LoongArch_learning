module mycpu_top(
    input         clk,
    input         resetn,
    // inst sram interface
    // output        inst_sram_en,
    // output [ 3:0] inst_sram_we,
    // output [19:0] inst_sram_addr,
    // // output [31:0] inst_sram_wdata,
    // input  [31:0] inst_sram_rdata,

    // data sram interface
    // output        data_sram_en,
    // output [ 3:0] data_sram_we,
    // output [19:0] data_sram_addr,
    // output [31:0] data_sram_wdata,
    // input  [31:0] data_sram_rdata 

      //BaseRAM信号
    (*mark_debug = "true"*)inout wire[31:0] base_ram_data,  //BaseRAM数据，低8位与CPLD串口控制器共享
    (*mark_debug = "true"*)output wire[19:0] base_ram_addr, //BaseRAM地址
    (*mark_debug = "true"*)output wire[3:0] base_ram_be_n,  //BaseRAM字节使能，低有效。如果不使用字节使能，请保持为0
    (*mark_debug = "true"*)output wire base_ram_ce_n,       //BaseRAM片选，低有效
    (*mark_debug = "true"*)output wire base_ram_oe_n,       //BaseRAM读使能，低有效
    (*mark_debug = "true"*)output wire base_ram_we_n,       //BaseRAM写使能，低有效

      //ExtRAM信号
    (*mark_debug = "true"*)inout wire[31:0] ext_ram_data,  //ExtRAM数据
    (*mark_debug = "true"*)output wire[19:0] ext_ram_addr, //ExtRAM地址
    (*mark_debug = "true"*)output wire[3:0] ext_ram_be_n,  //ExtRAM字节使能，低有效。如果不使用字节使能，请保持为0
    (*mark_debug = "true"*)output wire ext_ram_ce_n,       //ExtRAM片选，低有效
    (*mark_debug = "true"*)output wire ext_ram_oe_n,       //ExtRAM读使能，低有效
    (*mark_debug = "true"*)output wire ext_ram_we_n        //ExtRAM写使能，低有效  
    // // trace debug interface
    // output [31:0] debug_wb_pc,
    // output [ 3:0] debug_wb_rf_we,
    // output [ 4:0] debug_wb_rf_wnum,
    // output [31:0] debug_wb_rf_wdata
);
reg         reset;
always @(posedge clk) reset <= ~resetn;

wire         ds_allowin;
wire         es_allowin;
wire         ms_allowin;
wire         ws_allowin;
wire         fs_to_ds_valid;
wire         ds_to_es_valid;
wire         es_to_ms_valid;
wire         ms_to_ws_valid;
wire [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus;
wire [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus;
wire [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus;
wire [`MS_TO_WS_BUS_WD -1:0] ms_to_ws_bus;
wire [`WS_TO_RF_BUS_WD -1:0] ws_to_rf_bus;
wire [`BR_BUS_WD       -1:0] br_bus;
wire [ 4                 :0] WB_dest,EXE_dest,MEM_dest;    
wire                         exe_gr_we,mem_gr_we,wb_gr_we;
wire [31                 :0] EXE_result,MEM_result,WB_result;
wire                         es_load_valid;
wire [31                 :0] data_sram_wdata;
reg  [31                 :0] data_sram_rdata;
assign base_ram_ce_n = 1'b0;
assign base_ram_we_n = 1'b1;

assign ext_ram_ce_n  = 1'b0;// 片选信号，低电平有效 

// assign ext_ram_oe_n  = ~es_load_valid;
assign ext_ram_oe_n  = ~ext_ram_we_n;
always @(ext_ram_oe_n) begin
    if (~ext_ram_oe_n) 
        data_sram_rdata <= ext_ram_data;
end
// assign data_sram_rdata  = ext_ram_data;
assign ext_ram_data  = (~ext_ram_we_n)?data_sram_wdata:32'bz;
// IF stage
if_stage if_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    //allowin
    .ds_allowin     (ds_allowin     ),
    //brbus
    .br_bus         (br_bus         ),
    //outputs
    .fs_to_ds_valid (fs_to_ds_valid ),
    .fs_to_ds_bus   (fs_to_ds_bus   ),
    // inst sram interface
    .inst_sram_en   (base_ram_oe_n  ),
    .inst_sram_we   (base_ram_be_n  ),
    .inst_sram_addr (base_ram_addr  ),
    // .inst_sram_wdata(inst_sram_wdata),
    .inst_sram_rdata(base_ram_data  )
);
// ID stage
id_stage id_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    //allowin
    .es_allowin     (es_allowin     ),
    .ds_allowin     (ds_allowin     ),
    //from fs
    .fs_to_ds_valid (fs_to_ds_valid ),
    .fs_to_ds_bus   (fs_to_ds_bus   ),
    //to es
    .ds_to_es_valid (ds_to_es_valid ),
    .ds_to_es_bus   (ds_to_es_bus   ),
    //to fs
    .br_bus         (br_bus         ),
    // .ext_ram_re     (ext_ram_re     ),
    //to rf: for write back
    .ws_to_rf_bus   (ws_to_rf_bus   ), 
    .WB_dest        (WB_dest        ),
    .MEM_dest       (MEM_dest       ),
    .EXE_dest       (EXE_dest       ),
    .mem_gr_we      (mem_gr_we      ),
    .exe_gr_we      (exe_gr_we      ),
    .wb_gr_we       (wb_gr_we       ),
    .es_load_valid  (es_load_valid  ),
    .EXE_result     (EXE_result     ),
    .MEM_result     (MEM_result     ),
    .WB_result      (WB_result      )
);
// EXE stage
exe_stage exe_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    //allowin
    .ms_allowin     (ms_allowin     ),
    .es_allowin     (es_allowin     ),
    //from ds
    .ds_to_es_valid (ds_to_es_valid ),
    .ds_to_es_bus   (ds_to_es_bus   ),
    //to ms
    .es_to_ms_valid (es_to_ms_valid ),
    .es_to_ms_bus   (es_to_ms_bus   ),
    // data sram interface
    .data_sram_en   (ext_ram_we_n   ),
    .data_sram_we   (ext_ram_be_n   ),
    .data_sram_addr (ext_ram_addr   ),
    .data_sram_wdata(data_sram_wdata   ), 
    .EXE_dest       (EXE_dest       ),
    .exe_gr_we      (exe_gr_we      ),
    .es_load_valid  (es_load_valid  ),
    .EXE_result     (EXE_result     )
);
// MEM stage
mem_stage mem_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    //allowin
    .ws_allowin     (ws_allowin     ),
    .ms_allowin     (ms_allowin     ),
    //from es
    .es_to_ms_valid (es_to_ms_valid ),
    .es_to_ms_bus   (es_to_ms_bus   ),
    //to ws
    .ms_to_ws_valid (ms_to_ws_valid ),
    .ms_to_ws_bus   (ms_to_ws_bus   ),
    //from data-sram
    // .data_sram_we   (ext_ram_oe_n   ),
    .data_sram_rdata(data_sram_rdata   ),
    .MEM_dest       (MEM_dest       ),
    .mem_gr_we      (mem_gr_we      ),
    .MEM_result     (MEM_result     )
);
// WB stage
wb_stage wb_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    //allowin
    .ws_allowin     (ws_allowin     ),
    //from ms
    .ms_to_ws_valid (ms_to_ws_valid ),
    .ms_to_ws_bus   (ms_to_ws_bus   ),
    //to rf: for write back
    .ws_to_rf_bus   (ws_to_rf_bus   ),
    // //trace debug interface
    // .debug_wb_pc      (debug_wb_pc      ),
    // .debug_wb_rf_we  (debug_wb_rf_we  ),
    // .debug_wb_rf_wnum (debug_wb_rf_wnum ),
    // .debug_wb_rf_wdata(debug_wb_rf_wdata),
    .WB_dest          (WB_dest          ),
    .wb_gr_we         (wb_gr_we         ),
    .WB_result        (WB_result        )
);

endmodule
