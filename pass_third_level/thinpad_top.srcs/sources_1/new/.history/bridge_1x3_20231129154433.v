//*************************************************************************
//   > File Name   : bridge_1x3.v
//   > Description : bridge between cpu_data and data ram, confreg
//   
//     master:    cpu_data
//                 /  |  \
//     1 x 3      /   |   \  
//     bridge:   /    |    \                    
//              /     |     \       
//     slave: inst   data  uart
//
//   > Author      : zgh
//   > Date        : 2023-11-19
//*************************************************************************
`define CONF_ADDR_INST 32'h0000_0000
`define CONF_ADDR_DATA 32'h0040_0000
`define CONF_ADDR_MASK 32'hffc0_0000
module bridge_1x3(                                 
    input                           clk,          // clock 
    input                           resetn,       // reset, active low
    // master : cpu data  // slave : inst ram 
    input                          inst_sram_en,     // access inst_sram enable
    input [3                   :0] inst_sram_wen,    // write enable 
    input [31                  :0] inst_sram_addr,   // address
    input [31                  :0] inst_sram_wdata,  // inst in
    output[31                  :0] inst_sram_rdata,  // inst out
    // slave : data ram 
    input                           data_sram_en,     // access data_sram enable
    input  [3                   :0] data_sram_wen,    // write enable 
    input  [31                  :0] data_sram_addr,   // address
    input  [31                  :0] data_sram_wdata,  // data in
    output [31                  :0] data_sram_rdata,  // data out
    input                           es_load_valid,
    input                           if_stall,
    // slave : confreg 
    output                          conf_en,          // access confreg enable 
    output [3                   :0] conf_wen,         // access confreg enable 
    output [31                  :0] conf_addr,        // address
    output [31                  :0] conf_wdata,       // write data
    input  [31                  :0] conf_rdata  ,      // read data
    //to memory 
    output                          is_base_ext,
    output [3                   :0] mem_be,
    output [19                  :0] mem_addr,
    output [31                  :0] mem_rdata,mem_wdata,
    output                          mem_oe,mem_we
);
    // wire   es_load_valid;
    reg sel_inst_r; // reg of sel_inst
    reg sel_data_r; // reg of sel_dram 
    reg sel_conf_r; // reg of sel_conf 
    wire sel_inst;  // cpu data is from inst ram
    wire sel_data;  // cpu data is from data ram
    wire sel_conf;  // cpu data is from confreg
    assign sel_inst = inst_sram_en&(data_sram_addr & `CONF_ADDR_MASK) == `CONF_ADDR_INST;
    assign sel_data = (es_load_VALID|data_sram_EN)&(data_sram_addr & `CONF_ADDR_MASK) == `CONF_ADDR_DATA;
    
    // assign sel_conf = ~sel_inst & ~sel_data&(data_sram_addr == 32'h1fd003fc||data_sram_addr == 32'h1fd003f8);
    reg data_sram_EN,es_load_VALID;
    reg [31:0]      data_sram_WDATA;
    reg [31:0]      data_sram_ADDR;
    reg [3:0]       data_sram_WEN;
    always @(posedge clk) begin
        if (!resetn) begin
            data_sram_EN <= 1'b0;
            es_load_VALID <= 1'b0;
            data_sram_WDATA <= 32'h0;
            data_sram_ADDR <= 20'h0;
            data_sram_WEN <= 4'hf;
        end else begin
            data_sram_EN <= data_sram_en;
            es_load_VALID <= es_load_valid;
            data_sram_WDATA <= data_sram_wdata;
            data_sram_WEN <= data_sram_wen;
            data_sram_ADDR <= data_sram_addr;
        end
    end
    assign is_base_ext = ((es_load_VALID|data_sram_EN)&sel_inst)|inst_sram_en;

    assign mem_be = es_load_VALID?4'hf:
                    data_sram_EN?data_sram_WEN:
                /*inst_sram_en*/                4'hf;
    assign mem_oe = inst_sram_en||es_load_VALID;
    assign mem_we = data_sram_EN;

    
    assign mem_addr = !resetn?20'h0:
                        es_load_VALID||data_sram_EN?data_sram_ADDR[21:2]:inst_sram_addr[21:2];
    assign inst_sram_rdata = mem_rdata;
   
    assign mem_wdata = data_sram_WDATA;
    reg IF_stall;
    assign if_stall = IF_stall;

    always @(posedge clk) begin
        if (!resetn) 
            IF_stall <= 1'b0;
        else if (data_sram_en || es_load_valid) 
            IF_stall <= 1'b1;
        else 
            IF_stall <= 1'b0;
    end
    // confreg
    assign conf_en    = (data_sram_EN | es_load_VALID)& sel_conf_r;
    assign conf_wen   = data_sram_WEN;
    assign conf_addr  = data_sram_ADDR;
    assign conf_wdata = data_sram_WDATA;

    always @ (posedge clk)
    begin
        if (!resetn)
        begin
            sel_inst_r <= 1'b0;
            sel_data_r <= 1'b0;
            sel_conf_r <= 1'b0;
            // cpu_data_wen <= 4'h0;
            // cpu_data_addr <= 32'h0;
            // cpu_data_rdata <= 32'h0;
            // cpu_data_wdata <= 32'h0;
        end
        else
        begin
            sel_inst_r <= sel_inst;
            sel_data_r <= sel_data;
            sel_conf_r <= sel_conf;
        end
    end
    reg [31:0]data_sram_RDATA;
    always @(posedge clk) begin
        if (!resetn) data_sram_RDATA <= 32'h0;
        else if (sel_data||sel_inst) 
            data_sram_RDATA <= mem_rdata;
        else if (sel_conf_r) 
            data_sram_RDATA <= conf_rdata;
        else 
            data_sram_RDATA <= 32'h0;
    end
    assign data_sram_rdata = data_sram_RDATA;
    // assign data_sram_rdata = sel_data_r||sel_inst_r?mem_rdata:
    //                             sel_conf_r?conf_rdata:32'h0;

    // assign data_sram_rdata   = {32{sel_inst_r}} & inst_sram_rdata
    //                         | {32{sel_data_r}} & data_sram_rdata
    //                         | {32{sel_conf_r}} & conf_rdata;

endmodule