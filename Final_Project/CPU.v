//############################################################################
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//   (C) Copyright Laboratory System Integration and Silicon Implementation
//   All Right Reserved
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   ICLAB 2021 Final Project: Customized ISA Processor 
//   Author              : Hsi-Hao Huang
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   File Name   : CPU.v
//   Module Name : CPU.v
//   Release version : V1.0 (Release Date: 2021-May)
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//############################################################################

module CPU(

				clk,
			  rst_n,
  
		   IO_stall,

         awid_m_inf,
       awaddr_m_inf,
       awsize_m_inf,
      awburst_m_inf,
        awlen_m_inf,
      awvalid_m_inf,
      awready_m_inf,
                    
        wdata_m_inf,
        wlast_m_inf,
       wvalid_m_inf,
       wready_m_inf,
                    
          bid_m_inf,
        bresp_m_inf,
       bvalid_m_inf,
       bready_m_inf,
                    
         arid_m_inf,
       araddr_m_inf,
        arlen_m_inf,
       arsize_m_inf,
      arburst_m_inf,
      arvalid_m_inf,
                    
      arready_m_inf, 
          rid_m_inf,
        rdata_m_inf,
        rresp_m_inf,
        rlast_m_inf,
       rvalid_m_inf,
       rready_m_inf 

);
// Input port
input  wire clk, rst_n;
// Output port
output reg  IO_stall;

parameter ID_WIDTH = 4 , ADDR_WIDTH = 32, DATA_WIDTH = 16, DRAM_NUMBER=2, WRIT_NUMBER=1;

// AXI Interface wire connecttion for pseudo DRAM read/write
/* Hint:
  your AXI-4 interface could be designed as convertor in submodule(which used reg for output signal),
  therefore I declared output of AXI as wire in CPU
*/



// axi write address channel 
output  wire [WRIT_NUMBER * ID_WIDTH-1:0]        awid_m_inf;
output  wire [WRIT_NUMBER * ADDR_WIDTH-1:0]    awaddr_m_inf;
output  wire [WRIT_NUMBER * 3 -1:0]            awsize_m_inf;
output  wire [WRIT_NUMBER * 2 -1:0]           awburst_m_inf;
output  wire [WRIT_NUMBER * 7 -1:0]             awlen_m_inf;
output  wire [WRIT_NUMBER-1:0]                awvalid_m_inf;
input   wire [WRIT_NUMBER-1:0]                awready_m_inf;
// axi write data channel 
output  wire [WRIT_NUMBER * DATA_WIDTH-1:0]     wdata_m_inf;
output  wire [WRIT_NUMBER-1:0]                  wlast_m_inf;
output  wire [WRIT_NUMBER-1:0]                 wvalid_m_inf;
input   wire [WRIT_NUMBER-1:0]                 wready_m_inf;
// axi write response channel
input   wire [WRIT_NUMBER * ID_WIDTH-1:0]         bid_m_inf;
input   wire [WRIT_NUMBER * 2 -1:0]             bresp_m_inf;
input   wire [WRIT_NUMBER-1:0]             	   bvalid_m_inf;
output  wire [WRIT_NUMBER-1:0]                 bready_m_inf;
// -----------------------------
// axi read address channel 
output  wire [DRAM_NUMBER * ID_WIDTH-1:0]       arid_m_inf;
output  wire [DRAM_NUMBER * ADDR_WIDTH-1:0]   araddr_m_inf;
output  wire [DRAM_NUMBER * 7 -1:0]            arlen_m_inf;
output  wire [DRAM_NUMBER * 3 -1:0]           arsize_m_inf;
output  wire [DRAM_NUMBER * 2 -1:0]          arburst_m_inf;
output  wire [DRAM_NUMBER-1:0]               arvalid_m_inf;
input   wire [DRAM_NUMBER-1:0]               arready_m_inf;
// -----------------------------
// axi read data channel 
input   wire [DRAM_NUMBER * ID_WIDTH-1:0]         rid_m_inf;
input   wire [DRAM_NUMBER * DATA_WIDTH-1:0]     rdata_m_inf;
input   wire [DRAM_NUMBER * 2 -1:0]             rresp_m_inf;
input   wire [DRAM_NUMBER-1:0]                  rlast_m_inf;
input   wire [DRAM_NUMBER-1:0]                 rvalid_m_inf;
output  wire [DRAM_NUMBER-1:0]                 rready_m_inf;
// -----------------------------

//
//
// 
/* Register in each core:
  There are sixteen registers in your CPU. You should not change the name of those registers.
  TA will check the value in each register when your core is not busy.
  If you change the name of registers below, you must get the fail in this lab.
*/

reg signed [15:0] core_r0 , core_r1 , core_r2 , core_r3 ;
reg signed [15:0] core_r4 , core_r5 , core_r6 , core_r7 ;
reg signed [15:0] core_r8 , core_r9 , core_r10, core_r11;
reg signed [15:0] core_r12, core_r13, core_r14, core_r15;


//###########################################
//
// Wrtie down your design below
//
//###########################################

// state parameter
parameter PREPARE = 'd0, FETCH_INST_DRAM = 'd1, READ_INST_CACHE = 'd2, READ_INST_CACHE_D = 'd3, ID_STAGE = 'd4, ID_STAGE_D1 = 'd5, EXE_STAGE = 'd6, PREPARE_DATA = 'd7;
parameter FETCH_DATA_DRAM = 'd8, READ_DATA_CACHE = 'd9, READ_DATA_CACHE_D1 = 'd10, READ_DATA_CACHE_D2 = 'd11, DRAM_WRITE = 'd12, WB_STAGE = 'd13, OUTPUT = 'd14;


//ISA parameter
parameter ADD_OR_SUB = 3'b000;
parameter SLT_OR_MULT = 3'b001;
parameter LOAD = 3'b010;
parameter STORE = 3'b011;
parameter BEQ = 3'b100;
parameter JUMP = 3'b101;

//####################################################
//               reg & wire
//####################################################

reg [3:0] current_state, next_state;

//cache
reg [7:0] cache_addr;
reg [15:0] cache_din, cache_din_d1;

wire [15:0] cache_dout;
reg cache_WEB, cache_WEB_d1, cache_WEB_d2;

wire [7:0] inst_cache_addr;
wire [7:0] data_cache_addr;

reg cache_addr_first;

//cpu
reg signed [15:0] pc;
reg [15:0] cache_idx_lb_r, cache_idx_ub_r; // cache idx (lower bound/upper bound)
reg [15:0] cache_idx_lb, cache_idx_ub;

reg [15:0] inst;
wire [2:0] opcode;
wire [3:0] rs_idx, rt_idx, rd_idx;
wire inst_func;
reg signed [4:0] imme;
wire [12:0] jump_addr;
//ID stage
reg signed [15:0] rs_data, rt_data; 
//EXE stage
reg signed [15:0] data_wb; // data_wb: write_back_data
reg [3:0] reg_wb;
wire signed [15:0] OFFSET;
reg [15:0] dc; // data counter 
wire [15:0] dc_t;
reg [15:0] data_idx_lb_r, data_idx_ub_r;
reg [15:0] data_idx_lb, data_idx_ub;
// MEM stage
reg [1:0] read_dcache_finish;
wire load_finsih;
// flag
wire data_cache_miss_flag;
wire inst_cache_miss_flag;


//####################################################
//               CACHE
//####################################################
CACHE_256 ISNT_DATA_CACHE(
	.A0(cache_addr[0]), .A1(cache_addr[1]), .A2(cache_addr[2]), .A3(cache_addr[3]), .A4(cache_addr[4]), .A5(cache_addr[5]), .A6(cache_addr[6]), .A7(cache_addr[7]), 
	.DO0(cache_dout[0]), .DO1(cache_dout[1]), .DO2(cache_dout[2]), .DO3(cache_dout[3]), .DO4(cache_dout[4]), .DO5(cache_dout[5]), .DO6(cache_dout[6]), .DO7(cache_dout[7]), .DO8(cache_dout[8]), .DO9(cache_dout[9]), .DO10(cache_dout[10]), .DO11(cache_dout[11]), .DO12(cache_dout[12]), .DO13(cache_dout[13]), .DO14(cache_dout[14]), .DO15(cache_dout[15]),
	.DI0(cache_din[0]), .DI1(cache_din[1]), .DI2(cache_din[2]), .DI3(cache_din[3]), .DI4(cache_din[4]), .DI5(cache_din[5]), .DI6(cache_din[6]), .DI7(cache_din[7]), .DI8(cache_din[8]), .DI9(cache_din[9]), .DI10(cache_din[10]), .DI11(cache_din[11]), .DI12(cache_din[12]), .DI13(cache_din[13]), .DI14(cache_din[14]), .DI15(cache_din[15]),
	.CK(clk), .WEB(cache_WEB), .OE(1'b1), .CS(1'b1) );


always@(posedge clk, negedge rst_n)begin
	if(!rst_n) cache_addr_first <= 'd1;
	else begin
		case(current_state)
		PREPARE: cache_addr_first <= 'd1;
		FETCH_INST_DRAM: if(rvalid_m_inf[1]) cache_addr_first <= 'd0;
						 else cache_addr_first <= cache_addr_first;
		PREPARE_DATA: cache_addr_first <= 'd1;
		FETCH_DATA_DRAM: if(rvalid_m_inf[0]) cache_addr_first <= 'd0;
					     else cache_addr_first <= cache_addr_first;
		default: cache_addr_first <= cache_addr_first;
		endcase
	end
end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) cache_addr <= 'd0;
	else begin
		case(current_state)
		PREPARE: cache_addr <= 'd0;
		FETCH_INST_DRAM: if(rvalid_m_inf[1] && !cache_addr_first) cache_addr <= cache_addr + 'd1;
						 else cache_addr <= cache_addr;
		READ_INST_CACHE: cache_addr <= (pc - cache_idx_lb_r) >> 1; // inst cache address
		PREPARE_DATA: cache_addr <= 'd128;
		FETCH_DATA_DRAM: if(rvalid_m_inf[0] && !cache_addr_first) cache_addr <= cache_addr + 'd1;
						 else cache_addr <= cache_addr;
		READ_DATA_CACHE: cache_addr <= ((dc - data_idx_lb_r) >> 1) + 'd128;
		READ_DATA_CACHE_D1: cache_addr <= ((dc - data_idx_lb_r) >> 1) + 'd128;
		READ_DATA_CACHE_D2: cache_addr <= ((dc - data_idx_lb_r) >> 1) + 'd128;
		DRAM_WRITE: cache_addr <= dc; // need check
		default: cache_addr <= cache_addr;
		endcase
	end
end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) cache_din <= 'd0;
	else begin
		case(current_state)
		FETCH_INST_DRAM: if(rvalid_m_inf[1]) cache_din <= rdata_m_inf[31:16];
						else cache_din <= 'd0;
		FETCH_DATA_DRAM: if(rvalid_m_inf[0]) cache_din <= rdata_m_inf[15:0];
						else cache_din <= 'd0;
		READ_DATA_CACHE:if(opcode == STORE) cache_din <= rt_data;
						else cache_din <= 'd0;
		default: cache_din <= 'd0;
		endcase
	end

end


always@(posedge clk, negedge rst_n)begin
	if(!rst_n) cache_WEB <= 'd1;
	else begin
		case(current_state)
		FETCH_INST_DRAM: if(rvalid_m_inf[1]) cache_WEB <= 'd0;
						else cache_WEB <= 'd1;
		FETCH_DATA_DRAM: if(rvalid_m_inf[0]) cache_WEB <= 'd0;
						else cache_WEB <= 'd1;
		READ_DATA_CACHE:if(opcode == STORE) cache_WEB <= 'd0;
						else cache_WEB <= 'd1;
		default: cache_WEB <= 'd1;
		endcase
	end
end



//####################################################
//               CPU
//####################################################

//FSM
//current_state
always@(posedge clk, negedge rst_n)begin
	if(!rst_n) current_state <= PREPARE;
	else current_state <= next_state;
end
//next_state
always@(*)begin
	case(current_state)
	PREPARE: next_state = FETCH_INST_DRAM;
	FETCH_INST_DRAM: if(rlast_m_inf[1]) next_state = READ_INST_CACHE;
					 else next_state = current_state;
	READ_INST_CACHE: next_state = READ_INST_CACHE_D;
	READ_INST_CACHE_D: next_state = ID_STAGE;
	ID_STAGE: next_state = ID_STAGE_D1;
	ID_STAGE_D1: next_state = EXE_STAGE;
	EXE_STAGE:begin 
		case(opcode)
		ADD_OR_SUB: next_state = WB_STAGE;
		SLT_OR_MULT: next_state = WB_STAGE;
		LOAD: if(data_cache_miss_flag) next_state = PREPARE_DATA;
			  else next_state = READ_DATA_CACHE;
		STORE: next_state = DRAM_WRITE;
		default: next_state = OUTPUT;
		endcase
	end
	PREPARE_DATA: next_state = FETCH_DATA_DRAM;
	FETCH_DATA_DRAM: if(rlast_m_inf[0]) next_state = READ_DATA_CACHE;
					 else next_state = current_state;
	READ_DATA_CACHE: next_state = READ_DATA_CACHE_D1;
	READ_DATA_CACHE_D1: next_state = READ_DATA_CACHE_D2;
	READ_DATA_CACHE_D2: if(opcode == LOAD) next_state = WB_STAGE;
						else if(opcode == STORE) next_state = OUTPUT;
						else next_state = current_state;
	DRAM_WRITE: if(bvalid_m_inf && ( (dc < data_idx_lb_r) | (dc > data_idx_ub_r))) next_state = OUTPUT;
				else if(bvalid_m_inf) next_state = READ_DATA_CACHE;
				else next_state = current_state;
	WB_STAGE: next_state = OUTPUT;
	OUTPUT: if(inst_cache_miss_flag) next_state = PREPARE;
			else next_state = READ_INST_CACHE;
	default: next_state = current_state;
	endcase
end
// prepare idx bound for cache


// IF_STAGE

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) pc <= 'h1000;
	else begin
		case(current_state)
		ID_STAGE: pc <= pc + 2;
		EXE_STAGE: 
			if (opcode == BEQ && rs_data == rt_data ) pc <= pc + imme*2;
			else if(opcode == JUMP) pc <= {3'b000, jump_addr};
			else pc <= pc;
		default: pc <= pc;
		endcase
	end
end


always@(posedge clk, negedge rst_n)begin
	if(!rst_n)begin
		cache_idx_lb_r <= 'h1000;
		cache_idx_ub_r <= 'h10fe;
	end
	else begin
		cache_idx_lb_r <= cache_idx_lb;
		cache_idx_ub_r <= cache_idx_ub;
	end
end

always@(*)begin
	case(current_state)
	PREPARE:begin
		if(pc < 'h107c)begin cache_idx_lb = 'h1000; cache_idx_ub = 'h10fe; end
		else if(pc > 'h1f7c)begin cache_idx_lb = 'h1f00; cache_idx_ub = 'h1ffe; end
		else begin cache_idx_lb = pc - 'd124; cache_idx_ub = pc + 'd130; end
	end
	default:begin cache_idx_lb = cache_idx_lb_r; cache_idx_ub = cache_idx_ub_r; end
	endcase
end

//ID Stage

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) inst <= 'd0;
	else begin
		case(current_state)
		ID_STAGE: inst <= cache_dout;
		default: inst <= inst;
		endcase

	end
end

//  Dont use below variable before EXE stage
assign opcode 	 = inst[15:13];
assign rs_idx 	 = inst[12:9];
assign rt_idx 	 = inst[8:5];
assign rd_idx 	 = inst[4:1];
assign inst_func = inst[0];
assign jump_addr = inst[12:0];

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) rs_data <= 'd0;  // used for R-type
	else begin
		case(current_state)
		ID_STAGE_D1:begin
			case(inst[12:9])
			'd0 :rs_data <= core_r0;
			'd1 :rs_data <= core_r1;
			'd2 :rs_data <= core_r2;
			'd3 :rs_data <= core_r3;
			'd4 :rs_data <= core_r4;
			'd5 :rs_data <= core_r5;
			'd6 :rs_data <= core_r6;
			'd7 :rs_data <= core_r7;
			'd8 :rs_data <= core_r8;
			'd9 :rs_data <= core_r9;
			'd10:rs_data <= core_r10;
			'd11:rs_data <= core_r11;
			'd12:rs_data <= core_r12;
			'd13:rs_data <= core_r13;
			'd14:rs_data <= core_r14;
			default:rs_data <= core_r15;
			endcase
		end
		default: rs_data <= rs_data;
		endcase
	end
end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) rt_data <= 'd0;  // used for R-type
	else begin
		case(current_state)
		ID_STAGE_D1:begin
			case(inst[8:5])
			'd0 :rt_data <= core_r0;
			'd1 :rt_data <= core_r1;
			'd2 :rt_data <= core_r2;
			'd3 :rt_data <= core_r3;
			'd4 :rt_data <= core_r4;
			'd5 :rt_data <= core_r5;
			'd6 :rt_data <= core_r6;
			'd7 :rt_data <= core_r7;
			'd8 :rt_data <= core_r8;
			'd9 :rt_data <= core_r9;
			'd10:rt_data <= core_r10;
			'd11:rt_data <= core_r11;
			'd12:rt_data <= core_r12;
			'd13:rt_data <= core_r13;
			'd14:rt_data <= core_r14;
			default:rt_data <= core_r15;
			endcase
		end
		default: rt_data <= rt_data;
		endcase
	end
end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) imme <= 0;
	else begin
		case(current_state)
		ID_STAGE:begin
			imme <= cache_dout[4:0];
		end
		default: imme <= imme;
		endcase
	end	
end

//EXE_Stage
always@(posedge clk, negedge rst_n)begin
	if(!rst_n) data_wb <= 0;
	else begin
		case(current_state)
		EXE_STAGE:
			case(opcode)
			ADD_OR_SUB: if(inst_func == 0) data_wb <= rs_data + rt_data;
						else data_wb <= rs_data - rt_data;
			SLT_OR_MULT: 
				if(inst_func == 0)begin
					if(rs_data < rt_data) data_wb <= 1;
					else data_wb <= 0;
				end
				else data_wb <= rs_data * rt_data;
			default: data_wb <= data_wb;
			endcase
		READ_DATA_CACHE:data_wb <= cache_dout;
		READ_DATA_CACHE_D1: data_wb <= cache_dout;
		READ_DATA_CACHE_D2: data_wb <= cache_dout;
		endcase
	end
end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) reg_wb <= 'd0;
	else begin
		if(opcode[1]) reg_wb <= rt_idx; // lw or sw
		else reg_wb <= rd_idx;
	end
end

// calcualte dram address
assign OFFSET = 'h1000;
assign dc_t = ( (rs_data + imme)*2 ) + OFFSET;
assign data_cache_miss_flag = ( (dc_t < data_idx_lb_r) | (dc_t > data_idx_ub_r) );
always@(posedge clk, negedge rst_n)begin
	if(!rst_n) dc <= 'd0;
	else begin
		case(current_state)
		EXE_STAGE: begin
			case(opcode)
			LOAD: dc <= dc_t;
			STORE: dc <= dc_t;
			JUMP: dc <= {3'b100, jump_addr}; // need check
			default: dc <= dc;
			endcase
		end
		default: dc <= dc;
		endcase
	end
end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n)begin
		data_idx_lb_r <= 'h2345;
		data_idx_ub_r <= 'h0000;
	end
	else begin
		data_idx_lb_r <= data_idx_lb;
		data_idx_ub_r <= data_idx_ub;
	end
end

always@(*)begin
	case(current_state)
	PREPARE_DATA:begin
		if(dc < 'h107c)begin data_idx_lb = 'h1000; data_idx_ub = 'h10fe; end
		else if(dc > 'h1f7c)begin data_idx_lb = 'h1f00; data_idx_ub = 'h1ffe; end
		else begin data_idx_lb = dc - 'd124; data_idx_ub = dc + 'd130; end
	end
	default:begin data_idx_lb = data_idx_lb_r; data_idx_ub = data_idx_ub_r; end
	endcase
end



//WB_Stage

assign inst_cache_miss_flag = ( (pc < cache_idx_lb_r) | (pc > cache_idx_ub_r) );

always@(posedge clk, negedge rst_n)begin
	if(!rst_n)begin
		core_r0  <= 0; 
		core_r1  <= 0;
		core_r2  <= 0;
		core_r3  <= 0;
		core_r4  <= 0; 
		core_r5  <= 0; 
		core_r6  <= 0; 
		core_r7  <= 0;
		core_r8  <= 0; 
		core_r9  <= 0; 
		core_r10 <= 0; 
		core_r11 <= 0;
		core_r12 <= 0; 
		core_r13 <= 0; 
		core_r14 <= 0; 
		core_r15 <= 0;
	end
	else begin
		case(current_state)
		WB_STAGE:begin
			case(reg_wb)
			'd0 :core_r0 <= data_wb;
			'd1 :core_r1 <= data_wb;
			'd2 :core_r2 <= data_wb;
			'd3 :core_r3 <= data_wb;
			'd4 :core_r4 <= data_wb;
			'd5 :core_r5 <= data_wb;
			'd6 :core_r6 <= data_wb;
			'd7 :core_r7 <= data_wb;
			'd8 :core_r8 <= data_wb;
			'd9 :core_r9 <= data_wb;
			'd10:core_r10 <= data_wb;
			'd11:core_r11 <= data_wb;
			'd12:core_r12 <= data_wb;
			'd13:core_r13 <= data_wb;
			'd14:core_r14 <= data_wb;
			default:core_r15 <= data_wb;
			endcase
		end

		endcase
	end

end

//####################################################
//               AXI
//####################################################

// read channel const
assign arid_m_inf    	= 8'b0000_0000;
assign arsize_m_inf  	= 6'b001_001;
assign arburst_m_inf 	= 4'b01_01;
assign arlen_m_inf 		= 14'b1111111_1111111;

// read channel (inst)
READ_INST_DRAM AXI_READ_INST (.clk(clk), .rst_n(rst_n), .current_state(current_state), .cache_idx_lb(cache_idx_lb), .arready_m_inf(arready_m_inf[1]),
							  .rlast_m_inf(rlast_m_inf[1]), .araddr_m_inf(araddr_m_inf[63:32]), .arvalid_m_inf(arvalid_m_inf[1]), .rready_m_inf(rready_m_inf[1]));
// read channel (data)
READ_DATA_DRAM AXI_READ_DATA (.clk(clk), .rst_n(rst_n), .current_state(current_state), .data_idx_lb(data_idx_lb), .arready_m_inf(arready_m_inf[0]),
							  .rlast_m_inf(rlast_m_inf[0]), .araddr_m_inf(araddr_m_inf[31:0]), .arvalid_m_inf(arvalid_m_inf[0]), .rready_m_inf(rready_m_inf[0]));


//write channel const
reg [WRIT_NUMBER * ADDR_WIDTH-1:0] awaddr_m_inf_r;
reg [WRIT_NUMBER-1:0] awvalid_m_inf_r;
reg [WRIT_NUMBER * DATA_WIDTH-1:0] wdata_m_inf_r;
reg [WRIT_NUMBER-1:0] wvalid_m_inf_r;
reg [WRIT_NUMBER-1:0] wlast_m_inf_r;
reg [WRIT_NUMBER-1:0] bready_m_inf_r;

assign awaddr_m_inf = awaddr_m_inf_r;
assign awvalid_m_inf = awvalid_m_inf_r;
assign wdata_m_inf = wdata_m_inf_r;
assign wvalid_m_inf = wvalid_m_inf_r;
assign wlast_m_inf = wlast_m_inf_r;
assign bready_m_inf = bready_m_inf_r;

assign awlen_m_inf = 'd0;
assign awid_m_inf = 'd0;
assign awsize_m_inf = 3'b001;
assign awburst_m_inf = 2'd1;


always@(posedge clk, negedge rst_n)begin
	if(!rst_n) awaddr_m_inf_r <= 'd0;
	else begin
		case(current_state)
		EXE_STAGE: if(opcode == STORE) awaddr_m_inf_r <= dc_t;
				   else awaddr_m_inf_r <= awaddr_m_inf_r;
		DRAM_WRITE: if(awready_m_inf) awaddr_m_inf_r <= 'd0;
				   else awaddr_m_inf_r <= awaddr_m_inf_r;
		default: awaddr_m_inf_r <= awaddr_m_inf_r;
		endcase
	end
end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) awvalid_m_inf_r <= 'd0;
	else begin
		case(current_state)
		EXE_STAGE: if(opcode == STORE) awvalid_m_inf_r <= 'd1;
				   else awvalid_m_inf_r <= awvalid_m_inf_r;
		DRAM_WRITE: if(awready_m_inf) awvalid_m_inf_r <= 'd0;
				   else awvalid_m_inf_r <= awvalid_m_inf_r;
		default: awvalid_m_inf_r <= awvalid_m_inf_r;
		endcase
	end
end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) wdata_m_inf_r <= 'd0;
	else if(awready_m_inf) wdata_m_inf_r <= rt_data;
	else if(wready_m_inf) wdata_m_inf_r <= 'd0;
	
end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) wvalid_m_inf_r <= 'd0;
	else if(awready_m_inf) wvalid_m_inf_r <= 'd1;
	else if(wready_m_inf) wvalid_m_inf_r <= 'd0;
end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) wlast_m_inf_r <= 'd0;
	else if(awready_m_inf) wlast_m_inf_r <= 'd1;
	else if(wready_m_inf) wlast_m_inf_r <= 'd0;
end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) bready_m_inf_r <= 'd0;
	else if(awready_m_inf) bready_m_inf_r <= 'd1;
	else if(bvalid_m_inf) bready_m_inf_r <= 'd0;
end
//####################################################
//               OUTPUT
//####################################################

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) IO_stall <= 'd1;
	else begin
		case(current_state)
		OUTPUT: IO_stall <= 'd0;
		default: IO_stall <= 'd1;
		endcase
	end
end



endmodule


module READ_INST_DRAM(
	clk, rst_n, current_state, cache_idx_lb, arready_m_inf, rlast_m_inf, 
	araddr_m_inf, arvalid_m_inf, rready_m_inf
);
input clk, rst_n;
input [3:0] current_state;
input arready_m_inf, rlast_m_inf;
input [15:0] cache_idx_lb;
output reg [31:0] araddr_m_inf;
output reg arvalid_m_inf, rready_m_inf;

parameter PREPARE = 'd0;

// read channel (inst)
always@(posedge clk, negedge rst_n)begin
	if(!rst_n) araddr_m_inf <= 'd0;
	else begin
		case(current_state)
		PREPARE:begin
			araddr_m_inf <= {16'd0, cache_idx_lb};
		end
		default: araddr_m_inf <= 'd0;
		endcase
	end
end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) arvalid_m_inf <= 'd0;
	else begin
		case(current_state)
		PREPARE:begin
			arvalid_m_inf <= 'd1;
		end
		default: arvalid_m_inf <= 'd0;
		endcase
	end
end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) rready_m_inf <= 'd0;
	else if(arready_m_inf) rready_m_inf <= 'd1;
	else if(rlast_m_inf) rready_m_inf <= 'd0;
end

endmodule

module READ_DATA_DRAM(
	clk, rst_n, current_state, data_idx_lb, arready_m_inf, rlast_m_inf,
	araddr_m_inf, arvalid_m_inf, rready_m_inf
);

input clk, rst_n;
input [3:0] current_state;
input arready_m_inf, rlast_m_inf;
input [15:0] data_idx_lb;
output reg [31:0] araddr_m_inf;
output reg arvalid_m_inf, rready_m_inf;

parameter PREPARE_DATA = 'd7;

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) araddr_m_inf <= 'd0;
	else begin
		case(current_state)
		PREPARE_DATA:begin
			araddr_m_inf <= {16'd0, data_idx_lb};
		end
		default: araddr_m_inf <= 'd0;
		endcase

	end
end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) arvalid_m_inf <= 'd0;
	else begin
		case(current_state)
		PREPARE_DATA:begin
			arvalid_m_inf <= 'd1;
		end
		default: arvalid_m_inf <= 'd0;
		endcase
	end
end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) rready_m_inf <= 'd0;
	else if(arready_m_inf) rready_m_inf <= 'd1;
	else if(rlast_m_inf) rready_m_inf <= 'd0;
end

endmodule