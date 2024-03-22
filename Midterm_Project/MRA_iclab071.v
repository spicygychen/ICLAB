//############################################################################
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//   (C) Copyright Si2 LAB @NYCU ED430
//   All Right Reserved
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   ICLAB 2023 Fall
//   Midterm Proejct            : MRA  
//   Author                     : Lin-Hung, Lai
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   File Name   : MRA.v
//   Module Name : MRA
//   Release version : V2.0 (Release Date: 2023-10)
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//############################################################################

module MRA(
	// CHIP IO
	clk            	,	
	rst_n          	,	
	in_valid       	,	
	frame_id        ,	
	net_id         	,	  
	loc_x          	,	  
    loc_y         	,
	cost	 		,		
	busy         	,

    // AXI4 IO
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
	   rready_m_inf,
	
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
	   bready_m_inf 
);

// ===============================================================
//  					Parameter Declaration 
// ===============================================================
parameter ID_WIDTH = 4 , ADDR_WIDTH = 32, DATA_WIDTH = 128;

// ===============================================================
//  					Input / Output 
// ===============================================================

// << CHIP io port with system >>
input 			  	clk,rst_n;
input 			   	in_valid;
input  [4:0] 		frame_id;
input  [3:0]       	net_id;     
input  [5:0]       	loc_x; 
input  [5:0]       	loc_y; 
output reg [13:0] 	cost;
output reg          busy;       
  
// AXI Interface wire connecttion for pseudo DRAM read/write
/* Hint:
       Your AXI-4 interface could be designed as a bridge in submodule,
	   therefore I declared output of AXI as wire.  
	   Ex: AXI4_interface AXI4_INF(...);
*/

// ------------------------
// <<<<< AXI READ >>>>>
// ------------------------
// (1)	axi read address channel 
output wire [ID_WIDTH-1:0]      arid_m_inf;
output wire [1:0]            arburst_m_inf;
output wire [2:0]             arsize_m_inf;
output wire [7:0]              arlen_m_inf;
output wire                  arvalid_m_inf;
input  wire                  arready_m_inf;
output wire [ADDR_WIDTH-1:0]  araddr_m_inf;
// ------------------------
// (2)	axi read data channel 
input  wire [ID_WIDTH-1:0]       rid_m_inf;
input  wire                   rvalid_m_inf;
output wire                   rready_m_inf;
input  wire [DATA_WIDTH-1:0]   rdata_m_inf;
input  wire                    rlast_m_inf;
input  wire [1:0]              rresp_m_inf;
// ------------------------
// <<<<< AXI WRITE >>>>>
// ------------------------
// (1) 	axi write address channel 
output wire [ID_WIDTH-1:0]      awid_m_inf;
output wire [1:0]            awburst_m_inf;
output wire [2:0]             awsize_m_inf;
output wire [7:0]              awlen_m_inf;
output wire                  awvalid_m_inf;
input  wire                  awready_m_inf;
output wire [ADDR_WIDTH-1:0]  awaddr_m_inf;
// -------------------------
// (2)	axi write data channel 
output wire                   wvalid_m_inf;
input  wire                   wready_m_inf;
output wire [DATA_WIDTH-1:0]   wdata_m_inf;
output wire                    wlast_m_inf;
// -------------------------
// (3)	axi write response channel 
input  wire  [ID_WIDTH-1:0]      bid_m_inf;
input  wire                   bvalid_m_inf;
output wire                   bready_m_inf;
input  wire  [1:0]             bresp_m_inf;
// -----------------------------

// ===============================================================
//  					Reg / Wire 
// ===============================================================

reg [2:0] cur_state, next_state;
//state_flag
wire state_LMAP_READ, state_FILL_PATH, state_WAIT_WEIGHT, state_RETRACE, state_CLEAR_MAP, state_DRAM_WRITE, state_OUTPUT, state_IDLE;
// input_reg
reg [4:0] frame_id_r;
reg [3:0] net_id_r [0:14];
reg [5:0] loc_x_r[0:1][0:14]; 
reg [5:0] loc_y_r[0:1][0:14];
reg frame_id_ok;
// counter
wire [3:0] total_net;
reg [3:0] in_cnt;
reg [1:0] path_one_two;
reg [1:0] cnt_to_3;
reg [3:0] current_net;
reg [6:0] dram_cnt;

// flag
reg sink_flag;
reg wmap_read_flag;
reg laddr_finish, waddr_finish;
reg state_fill;
reg path_flag, source_encode_flag;
reg fill_finish;
reg retrace_finish;
wire back_to_source;
wire go_to_dram_state;
reg sram_write_flag;
reg dram_write_flag;
reg bready_pulldown;
// fill path
reg [1:0] path_map [0:63][0:63];
wire [1:0] fill_num;
reg [1:0] last_fill_num;
wire on_source;
reg on_source_d;
// retrace
reg rlast_d;
reg [5:0] retrace_x, retrace_y;
wire retrace_down, retrace_up, retrace_right, retrace_left;
wire [5:0] retrace_x_sub, retrace_x_add, retrace_y_sub, retrace_y_add;
wire dont_read_sink_w;
//cost
reg [13:0] total_cost;
//sram
reg [6:0] l_sram_addr;
wire [6:0] w_sram_addr;
reg [127:0] l_sram_di; 
wire [127:0] routed_data;
wire [127:0] w_sram_di;
wire [127:0] l_sram_do, w_sram_do;
wire L_WEB, W_WEB;

reg [6:0] sram_addr_r;
reg [6:0] sram_retrace_addr;
wire [6:0] retrace_y_mult2;
// ===============================================================
//  					Parameter / Integer 
// ===============================================================
//input
parameter SOURCE = 1'd0;
parameter SINK = 1'd1;
//state
parameter IDLE = 3'd0;
parameter LMAP_READ = 3'd1;
parameter FILL_PATH = 3'd2;
parameter RETRACE = 3'd3;
parameter WAIT_WEIGHT = 3'd4;
parameter DRAM_WRITE = 3'd5;
parameter CLEAR_MAP = 3'd6;
parameter OUTPUT = 3'd7;
//fill path
parameter EMPTY = 2'b00;
parameter MACRO = 2'b01;
parameter PATH_ONE = 2'b10;
parameter PATH_TWO = 2'b11;

integer i,j;
// ===============================================================
//  					FSM
// ===============================================================
always@(posedge clk, negedge rst_n)begin
	if(!rst_n) cur_state <= IDLE;
	else cur_state <= next_state;
end

always@(*)begin
	case(cur_state)
	IDLE: if(in_valid) next_state = LMAP_READ;
		else next_state = cur_state;
	LMAP_READ: if(rlast_m_inf) next_state = FILL_PATH;
		else next_state = cur_state;
	FILL_PATH: if(on_source )begin
			if(current_net==0) next_state = WAIT_WEIGHT;
			else next_state = RETRACE;
		end
		else next_state = cur_state;
	WAIT_WEIGHT: if(rlast_m_inf) next_state = RETRACE;
		else next_state = cur_state;
	RETRACE: if(back_to_source) next_state = CLEAR_MAP;
		else next_state = cur_state;
	CLEAR_MAP:if(retrace_finish) next_state = DRAM_WRITE;
		else next_state = FILL_PATH;
	DRAM_WRITE: if(bready_m_inf && bvalid_m_inf) next_state = OUTPUT;
		else next_state = cur_state;
	OUTPUT: next_state = IDLE;
	default: next_state = cur_state;
	endcase

end

// ===============================================================
//  					Input block
// ===============================================================
always@(posedge clk, negedge rst_n)begin
	if(!rst_n) frame_id_r <= 'd0;
	else begin
		if(in_valid) frame_id_r <= frame_id;
		else frame_id_r <= frame_id_r;
	end
end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) frame_id_ok <= 'd0;
	else begin
		if(in_valid) frame_id_ok <= 'd1;
		else frame_id_ok <= frame_id_ok;
	end
end


always@(posedge clk, negedge rst_n)begin
	if(!rst_n)begin
		for(i=0;i<15;i=i+1)begin
			net_id_r[i] <= 'd0;
		end
	end
	else begin
		if(in_valid && !sink_flag) net_id_r[in_cnt] <= net_id;
		else net_id_r <= net_id_r;
	end
end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n)begin
		for(i=0;i<2;i=i+1)begin
			for(j=0;j<15;j=j+1)begin
				loc_x_r[i][j] <= 'd0;
				loc_y_r[i][j] <= 'd0;
			end
		end
	end
	else begin
		if(in_valid)begin
			loc_x_r[sink_flag][in_cnt] <= loc_x;
			loc_y_r[sink_flag][in_cnt] <= loc_y;
		end
		else begin loc_x_r <= loc_x_r; loc_y_r <= loc_y_r; end
	end

end




// ===============================================================
//  					AXI4 Read
// ===============================================================

reg [ADDR_WIDTH-1:0] araddr_r;
reg arvalid_r;


assign arid_m_inf = 'b0;
assign arburst_m_inf = 2'b01;
assign arsize_m_inf = 3'b100;
assign arlen_m_inf = 8'd127;
assign araddr_m_inf = araddr_r;
assign arvalid_m_inf = arvalid_r;


always@(*)begin
	if(state_LMAP_READ)begin
		if(laddr_finish == 0) araddr_r = {{16'h0001}, frame_id_r , 11'b0};
		else araddr_r = 0;
	end
	else if(wmap_read_flag)begin
		if(waddr_finish == 0) araddr_r = {{16'h0002}, frame_id_r , 11'b0};
		else araddr_r = 0;
	end
	else araddr_r = 0;

end

always@(*)begin
	if(state_LMAP_READ)begin
		if(laddr_finish == 0) arvalid_r = 1;
		else arvalid_r = 0;
	end
	else if(wmap_read_flag)begin
		if(waddr_finish == 0) arvalid_r = 1;
		else arvalid_r = 0;
	end
	else arvalid_r = 0;

end

// always@(posedge clk, negedge rst_n)begin
// 	if(!rst_n) araddr_r <= 'd0;
// 	else begin
// 		if(state_LMAP_READ)begin
// 			if(arready_m_inf | laddr_finish) araddr_r <= 'd0;
// 		 	else araddr_r <= {{16'h0001}, frame_id_r , 11'b0};
// 		end
// 		else if(wmap_read_flag)begin
// 			if(arready_m_inf | waddr_finish) araddr_r <= 'd0; 
// 		 	else araddr_r <= {{16'h0002}, frame_id_r , 11'b0};
// 		end
// 		else araddr_r <= araddr_r;
// 		// case(cur_state)
// 		// LMAP_READ: if(arready_m_inf | laddr_finish) araddr_r <= 'd0;
// 		// 	else araddr_r <= {{16'b 0000_0000_0000_0001}, frame_id_r , 11'b0};
// 		// WMAP_READ: if(arready_m_inf | waddr_finish) araddr_r <= 'd0; 
// 		// 	else araddr_r <= {{16'b 0000_0000_0000_0010}, frame_id_r , 11'b0};
// 		// default: araddr_r <= araddr_r;
// 		// endcase

// 	end
// end

// always@(posedge clk, negedge rst_n)begin
// 	if(!rst_n) arvalid_r <= 'b0;
// 	else begin
// 		if(state_LMAP_READ)begin
// 			if(arready_m_inf | laddr_finish) arvalid_r <= 'd0;
// 		 	else arvalid_r <= 'd1;
// 		end
// 		else if(wmap_read_flag)begin
// 			if(arready_m_inf | waddr_finish) arvalid_r <= 'd0;
// 		 	else arvalid_r <= 'd1;
// 		end
// 		else arvalid_r <= arvalid_r;
// 		// case(cur_state)
// 		// LMAP_READ: if(arready_m_inf | laddr_finish) arvalid_r <= 'd0;
// 		// 	else arvalid_r <= 'd1;
// 		// WMAP_READ: if(arready_m_inf | waddr_finish) arvalid_r <= 'd0;
// 		// 	else arvalid_r <= 'd1;
// 		// default: arvalid_r <= arvalid_r;
// 		// endcase

// 	end
// end

// ===============================================================
//  					AXI4 Write
// ===============================================================
reg awaddr_finish;
reg [127:0] dram_data_in;
reg wlast_d;
reg awvalid;
assign awid_m_inf    = 4'd0;
assign awburst_m_inf = 2'd1;
assign awsize_m_inf  = 3'b100;
assign awlen_m_inf   = 8'd127;

assign awaddr_m_inf = (awvalid )?{{16'b 0000_0000_0000_0001}, frame_id_r , 11'b0}:0;
assign awvalid_m_inf = (awvalid )?'b1:'b0;

always@(posedge clk, negedge rst_n)begin
	if(!rst_n )dram_write_flag <= 'd0;
	else begin
		if(state_CLEAR_MAP) dram_write_flag <= 'd1;
		else dram_write_flag <= 0;
	end

end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) awvalid <= 'd0;
	else begin
		if(state_DRAM_WRITE && dram_write_flag) awvalid <= 1;
		else if(awvalid_m_inf && awready_m_inf) awvalid <= 0;
		else awvalid <= awvalid;
	end
end


//write address

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) awaddr_finish <= 'd0;
	else begin
		if(wlast_d) awaddr_finish <= 'd0;
		else if(awready_m_inf) awaddr_finish <= 'd1;
		else awaddr_finish <= awaddr_finish;
	end

end

//write data
always@(posedge clk, negedge rst_n)begin
	if(!rst_n) wlast_d <= 'd0;
	else begin
		if(dram_cnt == 127) wlast_d <= 'd1;
		else wlast_d <= 'd0;
	end
end

assign wdata_m_inf = (awaddr_finish)?l_sram_do:0;
assign wvalid_m_inf = (awaddr_finish)?1:0;
assign wlast_m_inf = (wlast_d)?1:0;
// //resp

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) bready_pulldown <= 'd1;
	else begin 
		if (bvalid_m_inf) bready_pulldown <= 'd0;
		else if(awready_m_inf) bready_pulldown <= 'd1;
		else bready_pulldown <= bready_pulldown;
	end
end
 
 assign bready_m_inf = (bready_pulldown)?1:0;


// ===============================================================
//  					SRAM
// ===============================================================

// reg [6:0] l_sram_addr;
// wire [6:0] w_sram_addr;
// reg [127:0] l_sram_di; 
// wire [127:0] routed_data;
// wire [127:0] w_sram_di;
// wire [127:0] l_sram_do, w_sram_do;
// wire L_WEB, W_WEB;

// reg [6:0] sram_addr_r;
// reg [6:0] sram_retrace_addr;
// wire [6:0] retrace_y_mult2;
assign L_WEB = ((rvalid_m_inf && (cur_state==LMAP_READ) ) || sram_write_flag) ?1'b0:1'b1;
assign W_WEB = (rvalid_m_inf && wmap_read_flag )?1'b0:1'b1;
// assign l_sram_addr = (state_RETRACE)?sram_retrace_addr:sram_addr_r;
assign w_sram_addr = (state_RETRACE)?sram_retrace_addr:sram_addr_r;
// assign l_sram_di = (rvalid_m_inf && (cur_state==LMAP_READ))?rdata_m_inf:'d0;
assign w_sram_di = (rvalid_m_inf && wmap_read_flag)?rdata_m_inf:'d0;

always@(*)begin
	if(rvalid_m_inf && (cur_state==LMAP_READ)) l_sram_di = rdata_m_inf;
	else if(sram_write_flag) l_sram_di = routed_data;
	else l_sram_di = 0;

end

always@(*)begin
	if(state_RETRACE) l_sram_addr = sram_retrace_addr;
	else if(state_DRAM_WRITE ) begin
		if(wready_m_inf ==  0)l_sram_addr = 0;
		else l_sram_addr = dram_cnt;
	end
	else l_sram_addr = sram_addr_r;
	
end


assign routed_data[3:0] = (retrace_x[4:0] == 0) ? net_id_r[current_net]: l_sram_do[3:0];
assign routed_data[7:4] = (retrace_x[4:0] == 1) ? net_id_r[current_net]: l_sram_do[7:4];
assign routed_data[11:8] = (retrace_x[4:0] == 2) ? net_id_r[current_net]: l_sram_do[11:8];
assign routed_data[15:12] = (retrace_x[4:0] == 3) ? net_id_r[current_net]: l_sram_do[15:12];
assign routed_data[19:16] = (retrace_x[4:0] == 4) ? net_id_r[current_net]: l_sram_do[19:16];
assign routed_data[23:20] = (retrace_x[4:0] == 5) ? net_id_r[current_net]: l_sram_do[23:20];
assign routed_data[27:24] = (retrace_x[4:0] == 6) ? net_id_r[current_net]: l_sram_do[27:24];
assign routed_data[31:28] = (retrace_x[4:0] == 7) ? net_id_r[current_net]: l_sram_do[31:28];
assign routed_data[35:32] = (retrace_x[4:0] == 8) ? net_id_r[current_net]: l_sram_do[35:32];
assign routed_data[39:36] = (retrace_x[4:0] == 9) ? net_id_r[current_net]: l_sram_do[39:36];
assign routed_data[43:40] = (retrace_x[4:0] == 10) ? net_id_r[current_net]: l_sram_do[43:40];
assign routed_data[47:44] = (retrace_x[4:0] == 11) ? net_id_r[current_net]: l_sram_do[47:44];
assign routed_data[51:48] = (retrace_x[4:0] == 12) ? net_id_r[current_net]: l_sram_do[51:48];
assign routed_data[55:52] = (retrace_x[4:0] == 13) ? net_id_r[current_net]: l_sram_do[55:52];
assign routed_data[59:56] = (retrace_x[4:0] == 14) ? net_id_r[current_net]: l_sram_do[59:56];
assign routed_data[63:60] = (retrace_x[4:0] == 15) ? net_id_r[current_net]: l_sram_do[63:60];
assign routed_data[67:64] = (retrace_x[4:0] == 16) ? net_id_r[current_net]: l_sram_do[67:64];
assign routed_data[71:68] = (retrace_x[4:0] == 17) ? net_id_r[current_net]: l_sram_do[71:68];
assign routed_data[75:72] = (retrace_x[4:0] == 18) ? net_id_r[current_net]: l_sram_do[75:72];
assign routed_data[79:76] = (retrace_x[4:0] == 19) ? net_id_r[current_net]: l_sram_do[79:76];
assign routed_data[83:80] = (retrace_x[4:0] == 20) ? net_id_r[current_net]: l_sram_do[83:80];
assign routed_data[87:84] = (retrace_x[4:0] == 21) ? net_id_r[current_net]: l_sram_do[87:84];
assign routed_data[91:88] = (retrace_x[4:0] == 22) ? net_id_r[current_net]: l_sram_do[91:88];
assign routed_data[95:92] = (retrace_x[4:0] == 23) ? net_id_r[current_net]: l_sram_do[95:92];
assign routed_data[99:96] = (retrace_x[4:0] == 24) ? net_id_r[current_net]: l_sram_do[99:96];
assign routed_data[103:100] = (retrace_x[4:0] == 25) ? net_id_r[current_net]: l_sram_do[103:100];
assign routed_data[107:104] = (retrace_x[4:0] == 26) ? net_id_r[current_net]: l_sram_do[107:104];
assign routed_data[111:108] = (retrace_x[4:0] == 27) ? net_id_r[current_net]: l_sram_do[111:108];
assign routed_data[115:112] = (retrace_x[4:0] == 28) ? net_id_r[current_net]: l_sram_do[115:112];
assign routed_data[119:116] = (retrace_x[4:0] == 29) ? net_id_r[current_net]: l_sram_do[119:116];
assign routed_data[123:120] = (retrace_x[4:0] == 30) ? net_id_r[current_net]: l_sram_do[123:120];
assign routed_data[127:124] = (retrace_x[4:0] == 31) ? net_id_r[current_net]: l_sram_do[127:124];


assign retrace_y_mult2 = retrace_y<<1;
always@(*)begin
	if(retrace_x<32) sram_retrace_addr = retrace_y_mult2;
	else sram_retrace_addr = retrace_y_mult2+1;
end


always@(posedge clk, negedge rst_n)begin
	if(!rst_n) sram_addr_r <= 'd0;
	else begin
		if(rvalid_m_inf)begin
			if(sram_addr_r == 'd127) sram_addr_r <= 'd0;
			else sram_addr_r <= sram_addr_r + 'd1;
		end
		else sram_addr_r <= sram_addr_r;
	end
end

SRAM_128 LMAP (.A0(l_sram_addr[0]),.A1(l_sram_addr[1]),.A2(l_sram_addr[2]),.A3(l_sram_addr[3]),.A4(l_sram_addr[4]),.A5(l_sram_addr[5]),.A6(l_sram_addr[6]),
				 .DO0(l_sram_do[0]),.DO1(l_sram_do[1]),.DO2(l_sram_do[2]),.DO3(l_sram_do[3]),.DO4(l_sram_do[4]),.DO5(l_sram_do[5]),.DO6(l_sram_do[6]),.DO7(l_sram_do[7]),.DO8(l_sram_do[8]),.DO9(l_sram_do[9]),.DO10(l_sram_do[10]),.DO11(l_sram_do[11]),.DO12(l_sram_do[12]),.DO13(l_sram_do[13]),.DO14(l_sram_do[14]),.DO15(l_sram_do[15]),
                 .DO16(l_sram_do[16]),.DO17(l_sram_do[17]),.DO18(l_sram_do[18]),.DO19(l_sram_do[19]),.DO20(l_sram_do[20]),.DO21(l_sram_do[21]),.DO22(l_sram_do[22]),.DO23(l_sram_do[23]),.DO24(l_sram_do[24]),.DO25(l_sram_do[25]),.DO26(l_sram_do[26]),.DO27(l_sram_do[27]),.DO28(l_sram_do[28]),.DO29(l_sram_do[29]),.DO30(l_sram_do[30]),.DO31(l_sram_do[31]),
                 .DO32(l_sram_do[32]),.DO33(l_sram_do[33]),.DO34(l_sram_do[34]),.DO35(l_sram_do[35]),.DO36(l_sram_do[36]),.DO37(l_sram_do[37]),.DO38(l_sram_do[38]),.DO39(l_sram_do[39]),.DO40(l_sram_do[40]),.DO41(l_sram_do[41]),.DO42(l_sram_do[42]),.DO43(l_sram_do[43]),.DO44(l_sram_do[44]),.DO45(l_sram_do[45]),.DO46(l_sram_do[46]),.DO47(l_sram_do[47]),
                 .DO48(l_sram_do[48]),.DO49(l_sram_do[49]),.DO50(l_sram_do[50]),.DO51(l_sram_do[51]),.DO52(l_sram_do[52]),.DO53(l_sram_do[53]),.DO54(l_sram_do[54]),.DO55(l_sram_do[55]),.DO56(l_sram_do[56]),.DO57(l_sram_do[57]),.DO58(l_sram_do[58]),.DO59(l_sram_do[59]),.DO60(l_sram_do[60]),.DO61(l_sram_do[61]),.DO62(l_sram_do[62]),.DO63(l_sram_do[63]),
                 .DO64(l_sram_do[64]),.DO65(l_sram_do[65]),.DO66(l_sram_do[66]),.DO67(l_sram_do[67]),.DO68(l_sram_do[68]),.DO69(l_sram_do[69]),.DO70(l_sram_do[70]),.DO71(l_sram_do[71]),.DO72(l_sram_do[72]),.DO73(l_sram_do[73]),.DO74(l_sram_do[74]),.DO75(l_sram_do[75]),.DO76(l_sram_do[76]),.DO77(l_sram_do[77]),.DO78(l_sram_do[78]),.DO79(l_sram_do[79]),
                 .DO80(l_sram_do[80]),.DO81(l_sram_do[81]),.DO82(l_sram_do[82]),.DO83(l_sram_do[83]),.DO84(l_sram_do[84]),.DO85(l_sram_do[85]),.DO86(l_sram_do[86]),.DO87(l_sram_do[87]),.DO88(l_sram_do[88]),.DO89(l_sram_do[89]),.DO90(l_sram_do[90]),.DO91(l_sram_do[91]),.DO92(l_sram_do[92]),.DO93(l_sram_do[93]),.DO94(l_sram_do[94]),.DO95(l_sram_do[95]),
                 .DO96(l_sram_do[96]),.DO97(l_sram_do[97]),.DO98(l_sram_do[98]),.DO99(l_sram_do[99]),.DO100(l_sram_do[100]),.DO101(l_sram_do[101]),.DO102(l_sram_do[102]),.DO103(l_sram_do[103]),.DO104(l_sram_do[104]),.DO105(l_sram_do[105]),.DO106(l_sram_do[106]),.DO107(l_sram_do[107]),.DO108(l_sram_do[108]),.DO109(l_sram_do[109]),.DO110(l_sram_do[110]),
                 .DO111(l_sram_do[111]),.DO112(l_sram_do[112]),.DO113(l_sram_do[113]),.DO114(l_sram_do[114]),.DO115(l_sram_do[115]),.DO116(l_sram_do[116]),.DO117(l_sram_do[117]),.DO118(l_sram_do[118]),.DO119(l_sram_do[119]),.DO120(l_sram_do[120]),.DO121(l_sram_do[121]),.DO122(l_sram_do[122]),.DO123(l_sram_do[123]),.DO124(l_sram_do[124]),
                 .DO125(l_sram_do[125]),.DO126(l_sram_do[126]),.DO127(l_sram_do[127]),.DI0(l_sram_di[0]),.DI1(l_sram_di[1]),.DI2(l_sram_di[2]),.DI3(l_sram_di[3]),.DI4(l_sram_di[4]),.DI5(l_sram_di[5]),.DI6(l_sram_di[6]),.DI7(l_sram_di[7]),.DI8(l_sram_di[8]),.DI9(l_sram_di[9]),.DI10(l_sram_di[10]),.DI11(l_sram_di[11]),.DI12(l_sram_di[12]),.DI13(l_sram_di[13]),.DI14(l_sram_di[14]),
                 .DI15(l_sram_di[15]),.DI16(l_sram_di[16]),.DI17(l_sram_di[17]),.DI18(l_sram_di[18]),.DI19(l_sram_di[19]),.DI20(l_sram_di[20]),.DI21(l_sram_di[21]),.DI22(l_sram_di[22]),.DI23(l_sram_di[23]),.DI24(l_sram_di[24]),.DI25(l_sram_di[25]),.DI26(l_sram_di[26]),.DI27(l_sram_di[27]),.DI28(l_sram_di[28]),.DI29(l_sram_di[29]),.DI30(l_sram_di[30]),
                 .DI31(l_sram_di[31]),.DI32(l_sram_di[32]),.DI33(l_sram_di[33]),.DI34(l_sram_di[34]),.DI35(l_sram_di[35]),.DI36(l_sram_di[36]),.DI37(l_sram_di[37]),.DI38(l_sram_di[38]),.DI39(l_sram_di[39]),.DI40(l_sram_di[40]),.DI41(l_sram_di[41]),.DI42(l_sram_di[42]),.DI43(l_sram_di[43]),.DI44(l_sram_di[44]),.DI45(l_sram_di[45]),.DI46(l_sram_di[46]),
                 .DI47(l_sram_di[47]),.DI48(l_sram_di[48]),.DI49(l_sram_di[49]),.DI50(l_sram_di[50]),.DI51(l_sram_di[51]),.DI52(l_sram_di[52]),.DI53(l_sram_di[53]),.DI54(l_sram_di[54]),.DI55(l_sram_di[55]),.DI56(l_sram_di[56]),.DI57(l_sram_di[57]),.DI58(l_sram_di[58]),.DI59(l_sram_di[59]),.DI60(l_sram_di[60]),.DI61(l_sram_di[61]),.DI62(l_sram_di[62]),
                 .DI63(l_sram_di[63]),.DI64(l_sram_di[64]),.DI65(l_sram_di[65]),.DI66(l_sram_di[66]),.DI67(l_sram_di[67]),.DI68(l_sram_di[68]),.DI69(l_sram_di[69]),.DI70(l_sram_di[70]),.DI71(l_sram_di[71]),.DI72(l_sram_di[72]),.DI73(l_sram_di[73]),.DI74(l_sram_di[74]),.DI75(l_sram_di[75]),.DI76(l_sram_di[76]),.DI77(l_sram_di[77]),.DI78(l_sram_di[78]),
                 .DI79(l_sram_di[79]),.DI80(l_sram_di[80]),.DI81(l_sram_di[81]),.DI82(l_sram_di[82]),.DI83(l_sram_di[83]),.DI84(l_sram_di[84]),.DI85(l_sram_di[85]),.DI86(l_sram_di[86]),.DI87(l_sram_di[87]),.DI88(l_sram_di[88]),.DI89(l_sram_di[89]),.DI90(l_sram_di[90]),.DI91(l_sram_di[91]),.DI92(l_sram_di[92]),.DI93(l_sram_di[93]),.DI94(l_sram_di[94]),
                 .DI95(l_sram_di[95]),.DI96(l_sram_di[96]),.DI97(l_sram_di[97]),.DI98(l_sram_di[98]),.DI99(l_sram_di[99]),.DI100(l_sram_di[100]),.DI101(l_sram_di[101]),.DI102(l_sram_di[102]),.DI103(l_sram_di[103]),.DI104(l_sram_di[104]),.DI105(l_sram_di[105]),.DI106(l_sram_di[106]),.DI107(l_sram_di[107]),.DI108(l_sram_di[108]),.DI109(l_sram_di[109]),
                 .DI110(l_sram_di[110]),.DI111(l_sram_di[111]),.DI112(l_sram_di[112]),.DI113(l_sram_di[113]),.DI114(l_sram_di[114]),.DI115(l_sram_di[115]),.DI116(l_sram_di[116]),.DI117(l_sram_di[117]),.DI118(l_sram_di[118]),.DI119(l_sram_di[119]),.DI120(l_sram_di[120]),.DI121(l_sram_di[121]),.DI122(l_sram_di[122]),.DI123(l_sram_di[123]),
                 .DI124(l_sram_di[124]),.DI125(l_sram_di[125]),.DI126(l_sram_di[126]),.DI127(l_sram_di[127]),.CK(clk),.WEB(L_WEB),.OE(1'b1), .CS(1'b1));

SRAM_128 WMAP (.A0(w_sram_addr[0]),.A1(w_sram_addr[1]),.A2(w_sram_addr[2]),.A3(w_sram_addr[3]),.A4(w_sram_addr[4]),.A5(w_sram_addr[5]),.A6(w_sram_addr[6]),
				 .DO0(w_sram_do[0]),.DO1(w_sram_do[1]),.DO2(w_sram_do[2]),.DO3(w_sram_do[3]),.DO4(w_sram_do[4]),.DO5(w_sram_do[5]),.DO6(w_sram_do[6]),.DO7(w_sram_do[7]),.DO8(w_sram_do[8]),.DO9(w_sram_do[9]),.DO10(w_sram_do[10]),.DO11(w_sram_do[11]),.DO12(w_sram_do[12]),.DO13(w_sram_do[13]),.DO14(w_sram_do[14]),.DO15(w_sram_do[15]),
                 .DO16(w_sram_do[16]),.DO17(w_sram_do[17]),.DO18(w_sram_do[18]),.DO19(w_sram_do[19]),.DO20(w_sram_do[20]),.DO21(w_sram_do[21]),.DO22(w_sram_do[22]),.DO23(w_sram_do[23]),.DO24(w_sram_do[24]),.DO25(w_sram_do[25]),.DO26(w_sram_do[26]),.DO27(w_sram_do[27]),.DO28(w_sram_do[28]),.DO29(w_sram_do[29]),.DO30(w_sram_do[30]),.DO31(w_sram_do[31]),
                 .DO32(w_sram_do[32]),.DO33(w_sram_do[33]),.DO34(w_sram_do[34]),.DO35(w_sram_do[35]),.DO36(w_sram_do[36]),.DO37(w_sram_do[37]),.DO38(w_sram_do[38]),.DO39(w_sram_do[39]),.DO40(w_sram_do[40]),.DO41(w_sram_do[41]),.DO42(w_sram_do[42]),.DO43(w_sram_do[43]),.DO44(w_sram_do[44]),.DO45(w_sram_do[45]),.DO46(w_sram_do[46]),.DO47(w_sram_do[47]),
                 .DO48(w_sram_do[48]),.DO49(w_sram_do[49]),.DO50(w_sram_do[50]),.DO51(w_sram_do[51]),.DO52(w_sram_do[52]),.DO53(w_sram_do[53]),.DO54(w_sram_do[54]),.DO55(w_sram_do[55]),.DO56(w_sram_do[56]),.DO57(w_sram_do[57]),.DO58(w_sram_do[58]),.DO59(w_sram_do[59]),.DO60(w_sram_do[60]),.DO61(w_sram_do[61]),.DO62(w_sram_do[62]),.DO63(w_sram_do[63]),
                 .DO64(w_sram_do[64]),.DO65(w_sram_do[65]),.DO66(w_sram_do[66]),.DO67(w_sram_do[67]),.DO68(w_sram_do[68]),.DO69(w_sram_do[69]),.DO70(w_sram_do[70]),.DO71(w_sram_do[71]),.DO72(w_sram_do[72]),.DO73(w_sram_do[73]),.DO74(w_sram_do[74]),.DO75(w_sram_do[75]),.DO76(w_sram_do[76]),.DO77(w_sram_do[77]),.DO78(w_sram_do[78]),.DO79(w_sram_do[79]),
                 .DO80(w_sram_do[80]),.DO81(w_sram_do[81]),.DO82(w_sram_do[82]),.DO83(w_sram_do[83]),.DO84(w_sram_do[84]),.DO85(w_sram_do[85]),.DO86(w_sram_do[86]),.DO87(w_sram_do[87]),.DO88(w_sram_do[88]),.DO89(w_sram_do[89]),.DO90(w_sram_do[90]),.DO91(w_sram_do[91]),.DO92(w_sram_do[92]),.DO93(w_sram_do[93]),.DO94(w_sram_do[94]),.DO95(w_sram_do[95]),
                 .DO96(w_sram_do[96]),.DO97(w_sram_do[97]),.DO98(w_sram_do[98]),.DO99(w_sram_do[99]),.DO100(w_sram_do[100]),.DO101(w_sram_do[101]),.DO102(w_sram_do[102]),.DO103(w_sram_do[103]),.DO104(w_sram_do[104]),.DO105(w_sram_do[105]),.DO106(w_sram_do[106]),.DO107(w_sram_do[107]),.DO108(w_sram_do[108]),.DO109(w_sram_do[109]),.DO110(w_sram_do[110]),
                 .DO111(w_sram_do[111]),.DO112(w_sram_do[112]),.DO113(w_sram_do[113]),.DO114(w_sram_do[114]),.DO115(w_sram_do[115]),.DO116(w_sram_do[116]),.DO117(w_sram_do[117]),.DO118(w_sram_do[118]),.DO119(w_sram_do[119]),.DO120(w_sram_do[120]),.DO121(w_sram_do[121]),.DO122(w_sram_do[122]),.DO123(w_sram_do[123]),.DO124(w_sram_do[124]),
                 .DO125(w_sram_do[125]),.DO126(w_sram_do[126]),.DO127(w_sram_do[127]),.DI0(w_sram_di[0]),.DI1(w_sram_di[1]),.DI2(w_sram_di[2]),.DI3(w_sram_di[3]),.DI4(w_sram_di[4]),.DI5(w_sram_di[5]),.DI6(w_sram_di[6]),.DI7(w_sram_di[7]),.DI8(w_sram_di[8]),.DI9(w_sram_di[9]),.DI10(w_sram_di[10]),.DI11(w_sram_di[11]),.DI12(w_sram_di[12]),.DI13(w_sram_di[13]),.DI14(w_sram_di[14]),
                 .DI15(w_sram_di[15]),.DI16(w_sram_di[16]),.DI17(w_sram_di[17]),.DI18(w_sram_di[18]),.DI19(w_sram_di[19]),.DI20(w_sram_di[20]),.DI21(w_sram_di[21]),.DI22(w_sram_di[22]),.DI23(w_sram_di[23]),.DI24(w_sram_di[24]),.DI25(w_sram_di[25]),.DI26(w_sram_di[26]),.DI27(w_sram_di[27]),.DI28(w_sram_di[28]),.DI29(w_sram_di[29]),.DI30(w_sram_di[30]),
                 .DI31(w_sram_di[31]),.DI32(w_sram_di[32]),.DI33(w_sram_di[33]),.DI34(w_sram_di[34]),.DI35(w_sram_di[35]),.DI36(w_sram_di[36]),.DI37(w_sram_di[37]),.DI38(w_sram_di[38]),.DI39(w_sram_di[39]),.DI40(w_sram_di[40]),.DI41(w_sram_di[41]),.DI42(w_sram_di[42]),.DI43(w_sram_di[43]),.DI44(w_sram_di[44]),.DI45(w_sram_di[45]),.DI46(w_sram_di[46]),
                 .DI47(w_sram_di[47]),.DI48(w_sram_di[48]),.DI49(w_sram_di[49]),.DI50(w_sram_di[50]),.DI51(w_sram_di[51]),.DI52(w_sram_di[52]),.DI53(w_sram_di[53]),.DI54(w_sram_di[54]),.DI55(w_sram_di[55]),.DI56(w_sram_di[56]),.DI57(w_sram_di[57]),.DI58(w_sram_di[58]),.DI59(w_sram_di[59]),.DI60(w_sram_di[60]),.DI61(w_sram_di[61]),.DI62(w_sram_di[62]),
                 .DI63(w_sram_di[63]),.DI64(w_sram_di[64]),.DI65(w_sram_di[65]),.DI66(w_sram_di[66]),.DI67(w_sram_di[67]),.DI68(w_sram_di[68]),.DI69(w_sram_di[69]),.DI70(w_sram_di[70]),.DI71(w_sram_di[71]),.DI72(w_sram_di[72]),.DI73(w_sram_di[73]),.DI74(w_sram_di[74]),.DI75(w_sram_di[75]),.DI76(w_sram_di[76]),.DI77(w_sram_di[77]),.DI78(w_sram_di[78]),
                 .DI79(w_sram_di[79]),.DI80(w_sram_di[80]),.DI81(w_sram_di[81]),.DI82(w_sram_di[82]),.DI83(w_sram_di[83]),.DI84(w_sram_di[84]),.DI85(w_sram_di[85]),.DI86(w_sram_di[86]),.DI87(w_sram_di[87]),.DI88(w_sram_di[88]),.DI89(w_sram_di[89]),.DI90(w_sram_di[90]),.DI91(w_sram_di[91]),.DI92(w_sram_di[92]),.DI93(w_sram_di[93]),.DI94(w_sram_di[94]),
                 .DI95(w_sram_di[95]),.DI96(w_sram_di[96]),.DI97(w_sram_di[97]),.DI98(w_sram_di[98]),.DI99(w_sram_di[99]),.DI100(w_sram_di[100]),.DI101(w_sram_di[101]),.DI102(w_sram_di[102]),.DI103(w_sram_di[103]),.DI104(w_sram_di[104]),.DI105(w_sram_di[105]),.DI106(w_sram_di[106]),.DI107(w_sram_di[107]),.DI108(w_sram_di[108]),.DI109(w_sram_di[109]),
                 .DI110(w_sram_di[110]),.DI111(w_sram_di[111]),.DI112(w_sram_di[112]),.DI113(w_sram_di[113]),.DI114(w_sram_di[114]),.DI115(w_sram_di[115]),.DI116(w_sram_di[116]),.DI117(w_sram_di[117]),.DI118(w_sram_di[118]),.DI119(w_sram_di[119]),.DI120(w_sram_di[120]),.DI121(w_sram_di[121]),.DI122(w_sram_di[122]),.DI123(w_sram_di[123]),
                 .DI124(w_sram_di[124]),.DI125(w_sram_di[125]),.DI126(w_sram_di[126]),.DI127(w_sram_di[127]),.CK(clk),.WEB(W_WEB),.OE(1'b1), .CS(1'b1));

// ===============================================================
//  					Counter
// ===============================================================

assign total_net = in_cnt-1;

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) in_cnt <= 'd0;
	else begin
		if(state_OUTPUT) in_cnt <= 'd0;
		else if(in_valid && sink_flag) in_cnt <= in_cnt + 'd1;
		else in_cnt <= in_cnt;
	end
end


always@(posedge clk, negedge rst_n)begin
	if(!rst_n)cnt_to_3 <= 'd0;
	else begin
		if(state_OUTPUT) cnt_to_3 <= 'd0;
		else if(state_FILL_PATH)begin
			if(source_encode_flag==0) cnt_to_3 <= cnt_to_3;
			else if(on_source) cnt_to_3 <= cnt_to_3 - 'd2;
			else  cnt_to_3 <= cnt_to_3 + 'd1;
		end
		else if(state_RETRACE)begin
			if(back_to_source) cnt_to_3 <= 'd0;
			else if(sram_write_flag == 0) cnt_to_3 <= cnt_to_3;
			else  cnt_to_3 <= cnt_to_3-'d1;
		end
		else cnt_to_3 <= cnt_to_3;


	end

end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n)current_net <= 'd0;
	else begin
		if(state_IDLE) current_net <= 'd0;
		else if(state_RETRACE)begin
			if(back_to_source) current_net <= current_net + 'd1;
			else current_net <= current_net;
		end
		else current_net <= current_net;

	end

end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) dram_cnt <= 'd1;
	else begin
		if(state_IDLE) dram_cnt <= 'd1;
		else if(wready_m_inf) dram_cnt <= dram_cnt + 1;
		else dram_cnt <= dram_cnt;
	end
end

// ===============================================================
//  					Flag
// ===============================================================

assign state_FILL_PATH = (cur_state == FILL_PATH);
assign state_LMAP_READ = (cur_state == LMAP_READ);
assign state_WAIT_WEIGHT = (cur_state == WAIT_WEIGHT);
assign state_RETRACE = (cur_state == RETRACE);
assign state_CLEAR_MAP = (cur_state == CLEAR_MAP);
assign state_DRAM_WRITE = (cur_state == DRAM_WRITE);
assign state_OUTPUT = (cur_state == OUTPUT);
assign state_IDLE = (cur_state == IDLE);

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) sink_flag <= 'b0;
	else begin
		if(in_valid)begin
			if(sink_flag) sink_flag <='b0;
			else sink_flag <= 'b1;
		end
		else sink_flag <= sink_flag;
	end
end

always@(posedge clk, negedge rst_n)begin // need reset after finish 1 pattern
	if(!rst_n) laddr_finish <= 'b0;
	else begin
		if(cur_state == IDLE) laddr_finish <= 'b0;
		else 
			if(arready_m_inf) laddr_finish <= 'b1;
			else laddr_finish <= laddr_finish;
	end
end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) wmap_read_flag <= 'b0;
	else begin
		if(state_LMAP_READ && rlast_m_inf) wmap_read_flag <= 'b1;
		else if(rlast_m_inf) wmap_read_flag <= 'b0;
		else wmap_read_flag <= wmap_read_flag;
	end
end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) waddr_finish <= 'b0;
	else begin
		if(state_IDLE) waddr_finish <= 'b0;
		else if(wmap_read_flag && arready_m_inf) waddr_finish <= 'b1;
		else waddr_finish <= waddr_finish;
		// case(cur_state)
		// IDLE: waddr_finish <= 'b0;
		// WMAP_READ: if(arready_m_inf) waddr_finish <= 'b1;
		// 	else waddr_finish <= waddr_finish;
		// default: waddr_finish <= waddr_finish;
		// endcase
	end
end




always@(posedge clk, negedge rst_n)begin
	if(!rst_n) source_encode_flag <= 'b0;
	else begin
		if(state_FILL_PATH) source_encode_flag <= 'b1;
		else if(state_RETRACE) source_encode_flag <= 'b0;
		else source_encode_flag <= source_encode_flag;
	end
end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) sram_write_flag <= 'd0;
	else begin
		if(state_RETRACE) begin
			sram_write_flag <= ~sram_write_flag;
		end
		else sram_write_flag <= 0;

	end

end

assign retrace_finish = (current_net == in_cnt);





// ===============================================================
//  					Fill path
// ===============================================================
wire [5:0] path_i;
assign path_i = sram_addr_r>>1;
assign fill_num = (cnt_to_3[1])?PATH_TWO:PATH_ONE;
assign on_source = path_map[loc_y_r[SINK][current_net]][loc_x_r[SINK][current_net]][1] && state_FILL_PATH;

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) on_source_d <= 'd0;
	else on_source_d <= on_source;
end

always@(posedge clk /*, negedge rst_n*/)begin
	// if(!rst_n)begin
	// 	for(i=0;i<64;i=i+1)begin
	// 		for(j=0;j<64;j=j+1)begin
	// 			path_map[i][j] <= 'd0;
	// 		end
	// 	end
	// end
	// else begin
		if(state_LMAP_READ)begin
			if(rvalid_m_inf)begin
				if(sram_addr_r[0] == 'b0)begin
					path_map[path_i][0] <= (rdata_m_inf[3:0]) ? MACRO : EMPTY;
					path_map[path_i][1] <= (rdata_m_inf[7:4]) ? MACRO : EMPTY;
					path_map[path_i][2] <= (rdata_m_inf[11:8]) ? MACRO : EMPTY;
					path_map[path_i][3] <= (rdata_m_inf[15:12]) ? MACRO : EMPTY;
					path_map[path_i][4] <= (rdata_m_inf[19:16]) ? MACRO : EMPTY;
					path_map[path_i][5] <= (rdata_m_inf[23:20]) ? MACRO : EMPTY;
					path_map[path_i][6] <= (rdata_m_inf[27:24]) ? MACRO : EMPTY;
					path_map[path_i][7] <= (rdata_m_inf[31:28]) ? MACRO : EMPTY;
					path_map[path_i][8] <= (rdata_m_inf[35:32]) ? MACRO : EMPTY;
					path_map[path_i][9] <= (rdata_m_inf[39:36]) ? MACRO : EMPTY;
					path_map[path_i][10] <= (rdata_m_inf[43:40]) ? MACRO : EMPTY;
					path_map[path_i][11] <= (rdata_m_inf[47:44]) ? MACRO : EMPTY;
					path_map[path_i][12] <= (rdata_m_inf[51:48]) ? MACRO : EMPTY;
					path_map[path_i][13] <= (rdata_m_inf[55:52]) ? MACRO : EMPTY;
					path_map[path_i][14] <= (rdata_m_inf[59:56]) ? MACRO : EMPTY;
					path_map[path_i][15] <= (rdata_m_inf[63:60]) ? MACRO : EMPTY;
					path_map[path_i][16] <= (rdata_m_inf[67:64]) ? MACRO : EMPTY;
					path_map[path_i][17] <= (rdata_m_inf[71:68]) ? MACRO : EMPTY;
					path_map[path_i][18] <= (rdata_m_inf[75:72]) ? MACRO : EMPTY;
					path_map[path_i][19] <= (rdata_m_inf[79:76]) ? MACRO : EMPTY;
					path_map[path_i][20] <= (rdata_m_inf[83:80]) ? MACRO : EMPTY;
					path_map[path_i][21] <= (rdata_m_inf[87:84]) ? MACRO : EMPTY;
					path_map[path_i][22] <= (rdata_m_inf[91:88]) ? MACRO : EMPTY;
					path_map[path_i][23] <= (rdata_m_inf[95:92]) ? MACRO : EMPTY;
					path_map[path_i][24] <= (rdata_m_inf[99:96]) ? MACRO : EMPTY;
					path_map[path_i][25] <= (rdata_m_inf[103:100]) ? MACRO : EMPTY;
					path_map[path_i][26] <= (rdata_m_inf[107:104]) ? MACRO : EMPTY;
					path_map[path_i][27] <= (rdata_m_inf[111:108]) ? MACRO : EMPTY;
					path_map[path_i][28] <= (rdata_m_inf[115:112]) ? MACRO : EMPTY;
					path_map[path_i][29] <= (rdata_m_inf[119:116]) ? MACRO : EMPTY;
					path_map[path_i][30] <= (rdata_m_inf[123:120]) ? MACRO : EMPTY;
					path_map[path_i][31] <= (rdata_m_inf[127:124]) ? MACRO : EMPTY;
				end
				else begin
					path_map[path_i][32] <= (rdata_m_inf[3:0]) ? MACRO : EMPTY;
					path_map[path_i][33] <= (rdata_m_inf[7:4]) ? MACRO : EMPTY;
					path_map[path_i][34] <= (rdata_m_inf[11:8]) ? MACRO : EMPTY;
					path_map[path_i][35] <= (rdata_m_inf[15:12]) ? MACRO : EMPTY;
					path_map[path_i][36] <= (rdata_m_inf[19:16]) ? MACRO : EMPTY;
					path_map[path_i][37] <= (rdata_m_inf[23:20]) ? MACRO : EMPTY;
					path_map[path_i][38] <= (rdata_m_inf[27:24]) ? MACRO : EMPTY;
					path_map[path_i][39] <= (rdata_m_inf[31:28]) ? MACRO : EMPTY;
					path_map[path_i][40] <= (rdata_m_inf[35:32]) ? MACRO : EMPTY;
					path_map[path_i][41] <= (rdata_m_inf[39:36]) ? MACRO : EMPTY;
					path_map[path_i][42] <= (rdata_m_inf[43:40]) ? MACRO : EMPTY;
					path_map[path_i][43] <= (rdata_m_inf[47:44]) ? MACRO : EMPTY;
					path_map[path_i][44] <= (rdata_m_inf[51:48]) ? MACRO : EMPTY;
					path_map[path_i][45] <= (rdata_m_inf[55:52]) ? MACRO : EMPTY;
					path_map[path_i][46] <= (rdata_m_inf[59:56]) ? MACRO : EMPTY;
					path_map[path_i][47] <= (rdata_m_inf[63:60]) ? MACRO : EMPTY;
					path_map[path_i][48] <= (rdata_m_inf[67:64]) ? MACRO : EMPTY;
					path_map[path_i][49] <= (rdata_m_inf[71:68]) ? MACRO : EMPTY;
					path_map[path_i][50] <= (rdata_m_inf[75:72]) ? MACRO : EMPTY;
					path_map[path_i][51] <= (rdata_m_inf[79:76]) ? MACRO : EMPTY;
					path_map[path_i][52] <= (rdata_m_inf[83:80]) ? MACRO : EMPTY;
					path_map[path_i][53] <= (rdata_m_inf[87:84]) ? MACRO : EMPTY;
					path_map[path_i][54] <= (rdata_m_inf[91:88]) ? MACRO : EMPTY;
					path_map[path_i][55] <= (rdata_m_inf[95:92]) ? MACRO : EMPTY;
					path_map[path_i][56] <= (rdata_m_inf[99:96]) ? MACRO : EMPTY;
					path_map[path_i][57] <= (rdata_m_inf[103:100]) ? MACRO : EMPTY;
					path_map[path_i][58] <= (rdata_m_inf[107:104]) ? MACRO : EMPTY;
					path_map[path_i][59] <= (rdata_m_inf[111:108]) ? MACRO : EMPTY;
					path_map[path_i][60] <= (rdata_m_inf[115:112]) ? MACRO : EMPTY;
					path_map[path_i][61] <= (rdata_m_inf[119:116]) ? MACRO : EMPTY;
					path_map[path_i][62] <= (rdata_m_inf[123:120]) ? MACRO : EMPTY;
					path_map[path_i][63] <= (rdata_m_inf[127:124]) ? MACRO : EMPTY;
				end
			end
			else path_map <= path_map;
		end
		
		else if(state_FILL_PATH)begin
			if(!source_encode_flag)begin
				path_map[loc_y_r[SOURCE][current_net]][loc_x_r[SOURCE][current_net]] <= PATH_TWO;
				path_map[loc_y_r[SINK][current_net]][loc_x_r[SINK][current_net]] <= EMPTY;
			end
			else begin
				for(i=1;i<63;i=i+1)
					for(j=1;j<63;j=j+1)
						if(path_map[i][j] == EMPTY && (path_map[i-1][j][1] || path_map[i+1][j][1] || path_map[i][j-1][1] || path_map[i][j+1][1]))
							path_map[i][j] <= fill_num;
							
				for(j=1;j<63;j=j+1) begin // left bound
					if(path_map[0][j] == EMPTY && (path_map[0][j-1][1] || path_map[0][j+1][1] || path_map[1][j][1]))
						path_map[0][j] <= fill_num;
					if(path_map[63][j] == EMPTY && (path_map[63][j-1][1] || path_map[63][j+1][1] || path_map[62][j][1]))
						path_map[63][j] <= fill_num;
				end

				
				for(i=1;i<63;i=i+1) begin // bottom bound
					if(path_map[i][0] == EMPTY && (path_map[i-1][0][1] || path_map[i+1][0][1] || path_map[i][1][1]))
						path_map[i][0] <= fill_num;
					if(path_map[i][63] == EMPTY && (path_map[i-1][63][1] || path_map[i+1][63][1] || path_map[i][62][1]))
						path_map[i][63] <= fill_num;
				end


				if(path_map[0][0] == EMPTY && (path_map[0][1][1] || path_map[1][0][1])) // left bottom corner
					path_map[0][0] <= fill_num;

					
				if(path_map[0][63] == EMPTY && (path_map[0][62][1] || path_map[1][63][1])) // left top corner
					path_map[0][63] <= fill_num;

					
				if(path_map[63][0] == EMPTY && (path_map[62][0][1] || path_map[63][1][1])) // right bottom corner
					path_map[63][0] <= fill_num;

				if(path_map[63][63] == EMPTY && (path_map[62][63][1] || path_map[63][62][1])) // right top corner
					path_map[63][63] <= fill_num;

			end

		end
		else if(state_RETRACE)begin
			path_map[retrace_y][retrace_x] <= MACRO;
		end
		else if(state_CLEAR_MAP)begin
			for(i=0;i<64;i=i+1)begin
				for(j=0;j<64;j=j+1)begin
					if(path_map[i][j] == MACRO) path_map[i][j] <= path_map[i][j];
					else path_map[i][j] <=EMPTY;
				end
			end
		end
		else path_map <= path_map;


	 // end
end

// always@(posedge clk, negedge rst_n)begin
// 	if(!rst_n) last_fill_num <= 'd2;
// 	else begin
// 		case(cur_state)
// 		FILL_PATH:if(path_map[loc_y_r[SINK][0]][loc_x_r[SINK][0]][1])begin last_fill_num <= cnt_to_3-1;end
// 		else last_fill_num <= last_fill_num;
// 		default: last_fill_num <= last_fill_num;
// 		endcase

// 	end

// end
// ===============================================================
//  					Retrace
// ===============================================================
assign dont_read_sink_w = (retrace_x == loc_x_r[SINK][current_net] && retrace_y == loc_y_r[SINK][current_net]) && sram_write_flag == 1;
assign back_to_source = (retrace_x == loc_x_r[SOURCE][current_net] && retrace_y == loc_y_r[SOURCE][current_net]) && sram_write_flag == 1;
assign retrace_down = path_map[retrace_y_add][retrace_x] == fill_num;
assign retrace_up = path_map[retrace_y_sub][retrace_x] == fill_num;
assign retrace_right = path_map[retrace_y][retrace_x_add] == fill_num;
assign retrace_left = path_map[retrace_y][retrace_x_sub] == fill_num;
assign retrace_x_add = retrace_x+1;
assign retrace_x_sub = retrace_x-1;
assign retrace_y_add = retrace_y+1;
assign retrace_y_sub = retrace_y-1;

// always@(posedge clk, negedge rst_n)begin
// 	if(!rst_n)rlast_d <= 'd0;
// 	else rlast_d <= rlast_m_inf;
// end



always@(posedge clk, negedge rst_n)begin
	if(!rst_n)retrace_x <= 'd0;
	else begin
		if(on_source || rlast_m_inf) retrace_x <= loc_x_r[SINK][current_net];
		else if(state_RETRACE)begin
			if(sram_write_flag == 0)begin
				retrace_x <= retrace_x;
			end
			else begin
				if(retrace_down && retrace_y!='d63) retrace_x <= retrace_x;
				else if(retrace_up && retrace_y!='d0) retrace_x <= retrace_x;
				else if(retrace_right && retrace_x!='d63) retrace_x <= retrace_x_add;
				else retrace_x <= retrace_x_sub;
			end
		end
		else retrace_x <= retrace_x;


	end

end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n)retrace_y <= 'd0;
	else begin
		if(on_source || rlast_m_inf) retrace_y <= loc_y_r[SINK][current_net];
		else if(state_RETRACE)begin
			if(sram_write_flag == 0)begin
				retrace_y <= retrace_y;
			end
			else begin
				if(retrace_down && retrace_y!='d63) retrace_y <= retrace_y_add;
				else if(retrace_up && retrace_y!='d0) retrace_y <= retrace_y_sub;
				else if(retrace_right && retrace_x!='d63) retrace_y <= retrace_y;
				else retrace_y <= retrace_y;
			end
		end
		else retrace_y <= retrace_y;

	end

end


wire [4:0] target_cost;
assign target_cost = (back_to_source || dont_read_sink_w)?0:w_sram_do[retrace_x[4:0]*4 +: 4];

// always@(posedge clk, negedge rst_n)begin
// 	if(!rst_n) total_cost <= 'd0;
// 	else begin
// 		if(state_RETRACE && sram_write_flag)begin
// 			total_cost <= total_cost + target_cost;
// 		end
// 		else total_cost <= total_cost;

// 	end

// end


// always@(posedge clk, negedge rst_n)begin
// 	if(rst_n)

// end
// ===============================================================
//  					Output
// ===============================================================

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) cost <= 'd0;
	else begin
		if(!rst_n) cost <= 'd0;
		else begin
			if(state_IDLE) cost <= 'd0;
			else if(state_RETRACE && sram_write_flag)begin
				cost <= cost + target_cost;
			end
			else cost <= cost;

		end

	end
end

always@(posedge clk, negedge rst_n)begin
	if(!rst_n) busy <= 'd0;
	else begin
		case(cur_state)
		LMAP_READ: if(!in_valid) busy <= 'b1;
			else busy <= busy;
		OUTPUT: busy <= 0;
		default: busy <= busy;
		endcase
	end
end



endmodule
