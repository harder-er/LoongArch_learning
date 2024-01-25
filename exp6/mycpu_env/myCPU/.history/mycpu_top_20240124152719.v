module mycpu_top(
    input  wire        clk,
    input  wire        resetn,
    // inst sram interface
    output wire        inst_sram_we,
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input  wire [31:0] inst_sram_rdata,
    // data sram interface
    output wire        data_sram_we,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata,
    input  wire [31:0] data_sram_rdata,
    // trace debug interface
    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_we,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata
);
reg         reset;
always @(posedge clk) reset <= ~resetn;

reg         valid;
always @(posedge clk) begin
    if (reset) 
        valid <= 1'b0;
    else 
        valid <= 1'b1;
end

reg  [31:0] seq_pc;
reg  [31:0] nextpc;
reg        br_taken;
reg [31:0] br_target;
reg  [31:0] inst;
reg  [31:0] pc;

reg [11:0] alu_op;
reg        load_op;
reg        src1_is_pc;
reg        src2_is_imm;
reg        res_from_mem;
reg        dst_is_r1;
reg        gr_we;
reg        mem_we;
reg        src_reg_is_rd;
reg [4: 0] dest;
reg [31:0] rj_value;
reg [31:0] rkd_value;
reg [31:0] imm;
reg [31:0] br_offs;
reg [31:0] jirl_offs;

reg [ 5:0] op_31_26;
reg [ 3:0] op_25_22;
reg [ 1:0] op_21_20;
reg [ 4:0] op_19_15;
reg [ 4:0] rd;
reg [ 4:0] rj;
reg [ 4:0] rk;
reg [11:0] i12;
reg [19:0] i20;
reg [15:0] i16;
reg [25:0] i26;

reg [63:0] op_31_26_d;
reg [15:0] op_25_22_d;
reg [ 3:0] op_21_20_d;
reg [31:0] op_19_15_d;

reg        inst_add_w;
reg        inst_sub_w;
reg        inst_slt;
reg        inst_sltu;
reg        inst_nor;
reg        inst_and;
reg        inst_or;
reg        inst_xor;
reg        inst_slli_w;
reg        inst_srli_w;
reg        inst_srai_w;
reg        inst_addi_w;
reg        inst_ld_w;
reg        inst_st_w;
reg        inst_jirl;
reg        inst_b;
reg        inst_bl;
reg        inst_beq;
reg        inst_bne;
reg        inst_lu12i_w;

reg        need_ui5;
reg        need_si12;
reg        need_si16;
reg        need_si20;
reg        need_si26;
reg        src2_is_4;

reg [ 4:0] rf_raddr1;
reg [31:0] rf_rdata1;
reg [ 4:0] rf_raddr2;
reg [31:0] rf_rdata2;
reg        rf_we;
reg [ 4:0] rf_waddr;
reg [31:0] rf_wdata;

reg [31:0] alu_src1   ;
reg [31:0] alu_src2   ;
reg [31:0] alu_result ;

reg [31:0] mem_result,final_result;



parameter IF  = 3'd0;
parameter ID  = 3'd1;
parameter EXE = 3'd2;
parameter MEM = 3'd3;
parameter WB  = 3'd4;
wire [2:0] cur_state,next_state;

always @(posedge clk) begin
    if (reset) 
        cur_state = 3'd0;
    else 
        cur_state = next_state;
end

always @(cur_state,next_state,inst_add_w,inst_sub_w,inst_slt,inst_sltu,inst_nor,
inst_and,inst_or,inst_xor,inst_slli_w,inst_srli_w,inst_srai_w,inst_addi_w,inst_ld_w,
inst_st_w,inst_jirl,inst_b,inst_bl,inst_beq,inst_bne,inst_lu12i_w) begin
    case (cur_state) 
        IF:
            next_state <= ID;
        ID:begin
            if (inst_b | inst_beq | inst_bne) 
                next_state <= IF;
            else 
                next_state <= EXE;            
        end
        EXE:begin
            if (inst_st_w | inst_ld_w) 
                next_state <= MEM;
            else 
                next_state <= WB;
        end
        MEM:begin
            if (inst_ld_w)
                next_state <= WB;
            else 
                next_state <= IF;
        end
        WB:begin
            next_state <= IF; 
        end
            
        default: next_state = next_state;
    endcase
end

decoder_6_64 u_dec0(.in(op_31_26 ), .out(op_31_26_d ));
decoder_4_16 u_dec1(.in(op_25_22 ), .out(op_25_22_d ));
decoder_2_4  u_dec2(.in(op_21_20 ), .out(op_21_20_d ));
decoder_5_32 u_dec3(.in(op_19_15 ), .out(op_19_15_d ));

always @(posedge clk) begin
    if (reset) begin
        rf_we <= 1'b0;
        pc <= 32'h1bff_fffc; 
    end
    else begin
        case (cur_state)
            IF: begin
                rf_we           <= 1'b0;
                seq_pc       = pc + 3'h4;
                pc              <= nextpc;
                nextpc       = br_taken ? br_target : seq_pc;
                inst            <= inst_sram_rdata;
                inst_sram_addr  <= pc;
            end
            ID: begin
                rf_we = 1'b0;
                op_31_26  = inst[31:26];
                op_25_22  = inst[25:22];
                op_21_20  = inst[21:20];
                op_19_15  = inst[19:15];

                rd   = inst[ 4: 0];
                rj   = inst[ 9: 5];
                rk   = inst[14:10];

                i12  = inst[21:10];
                i20  = inst[24: 5];
                i16  = inst[25:10];
                i26  = {inst[ 9: 0], inst[25:10]};


                inst_add_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h00];
                inst_sub_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h02];
                inst_slt    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h04];
                inst_sltu   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h05];
                inst_nor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h08];
                inst_and    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h09];
                inst_or     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0a];
                inst_xor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0b];
                inst_slli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h01];
                inst_srli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h09];
                inst_srai_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h11];
                inst_addi_w = op_31_26_d[6'h00] & op_25_22_d[4'ha];
                inst_ld_w   = op_31_26_d[6'h0a] & op_25_22_d[4'h2];
                inst_st_w   = op_31_26_d[6'h0a] & op_25_22_d[4'h6];
                inst_jirl   = op_31_26_d[6'h13];
                inst_b      = op_31_26_d[6'h14];
                inst_bl     = op_31_26_d[6'h15];
                inst_beq    = op_31_26_d[6'h16];
                inst_bne    = op_31_26_d[6'h17];
                inst_lu12i_w= op_31_26_d[6'h05] & ~inst[25];

                alu_op[ 0] = inst_add_w | inst_addi_w | inst_ld_w | inst_st_w
                    | inst_jirl | inst_bl;
                alu_op[ 1] = inst_sub_w;
                alu_op[ 2] = inst_slt;
                alu_op[ 3] = inst_sltu;
                alu_op[ 4] = inst_and;
                alu_op[ 5] = inst_nor;
                alu_op[ 6] = inst_or;
                alu_op[ 7] = inst_xor;
                alu_op[ 8] = inst_slli_w;
                alu_op[ 9] = inst_srli_w;
                alu_op[10] = inst_srai_w;
                alu_op[11] = inst_lu12i_w;
                need_ui5   =  inst_slli_w | inst_srli_w | inst_srai_w;
                need_si12  =  inst_addi_w | inst_ld_w | inst_st_w;
                need_si16  =  inst_jirl | inst_beq | inst_bne;
                need_si20  =  inst_lu12i_w;
                need_si26  =  inst_b | inst_bl;
                src2_is_4  =  inst_jirl | inst_bl;

                imm = src2_is_4 ? 32'h4                      :
                        need_si20 ? {i20[19:0], 12'b0}         :
                /*need_ui5 || need_si12*/{{20{i12[11]}}, i12[11:0]} ;

                br_offs = need_si26 ? {{ 4{i26[25]}}, i26[25:0], 2'b0} :
                                         {{14{i16[15]}}, i16[15:0], 2'b0} ;

                jirl_offs = {{14{i16[15]}}, i16[15:0], 2'b0};

                src_reg_is_rd = inst_beq | inst_bne | inst_st_w;

                src1_is_pc    = inst_jirl | inst_bl |inst_b;

                src2_is_imm   = inst_slli_w |
                       inst_srli_w |
                       inst_srai_w |
                       inst_addi_w |
                       inst_ld_w   |
                       inst_st_w   |
                       inst_lu12i_w|
                       inst_jirl   |
                       inst_bl     |
                       inst_b;

                res_from_mem  = inst_ld_w;
                dst_is_r1     = inst_bl;
                gr_we         = ~inst_st_w & ~inst_beq & ~inst_bne & ~inst_b ;
                mem_we        = inst_st_w;
                dest          = dst_is_r1 ? 5'd1 : rd;  
                rf_raddr1 = rj;
                rf_raddr2 = src_reg_is_rd ? rd :rk;
                regfile u_regfile(
                    .clk    (clk      ),
                    .raddr1 (rf_raddr1),
                    .rdata1 (rf_rdata1),
                    .raddr2 (rf_raddr2),
                    .rdata2 (rf_rdata2),
                    .we     (rf_we    ),
                    .waddr  (rf_waddr ),
                    .wdata  (rf_wdata )
                    );
                rj_value  = rf_rdata1;
                rkd_value = rf_rdata2;

                rj_eq_rd = (rj_value == rkd_value);
                br_taken = (   inst_beq  &&  rj_eq_rd
                        || inst_bne  && !rj_eq_rd
                        || inst_jirl
                        || inst_bl
                        || inst_b
                        ) && valid;

                br_target = (inst_beq || inst_bne || inst_bl || inst_b) ? (pc + br_offs) :
                                                      /*inst_jirl*/ (rj_value + jirl_offs);

                alu_src1 = src1_is_pc  ? pc[31:0] : rj_value;
                alu_src2 = src2_is_imm ? imm : rkd_value;
            end
            EXE: begin
                rf_we = 1'b0;
                alu u_alu(
                .alu_op     (alu_op    ),
                .alu_src1   (alu_src1  ),
                .alu_src2   (alu_src2  ),
                .alu_result (alu_result)
                );
                data_sram_we    = mem_we && valid;
                data_sram_addr  = alu_result;
                data_sram_wdata = rkd_value;
            end
            MEM: begin
                rf_we = 1'b0;
                mem_result   = data_sram_rdata;
                final_result = res_from_mem ? mem_result : alu_result;
            end
            WB : begin
                rf_we = gr_we&&valid;
                rf_waddr = dest;
                rf_wdata = final_result;
            end
            default:  rf_we = 1'b0;
        endcase
    end
end
assign inst_sram_we    = 1'b0;

assign inst_sram_wdata = 32'b0;

//assign rf_we    = gr_we && valid;


// debug info generate
assign debug_wb_pc       = pc;
assign debug_wb_rf_we   = {4{rf_we}};
assign debug_wb_rf_wnum  = dest;
assign debug_wb_rf_wdata = final_result;

endmodule
