module SMC(
  // Input signals
    mode,
    W_0, V_GS_0, V_DS_0,
    W_1, V_GS_1, V_DS_1,
    W_2, V_GS_2, V_DS_2,
    W_3, V_GS_3, V_DS_3,
    W_4, V_GS_4, V_DS_4,
    W_5, V_GS_5, V_DS_5,   
  // Output signals
    out_n
);

//================================================================
//   INPUT AND OUTPUT DECLARATION                         
//================================================================
input [2:0] W_0, V_GS_0, V_DS_0;
input [2:0] W_1, V_GS_1, V_DS_1;
input [2:0] W_2, V_GS_2, V_DS_2;
input [2:0] W_3, V_GS_3, V_DS_3;
input [2:0] W_4, V_GS_4, V_DS_4;
input [2:0] W_5, V_GS_5, V_DS_5;
input [1:0] mode;
output [7:0] out_n;         					


//================================================================
//    Wire & Registers 
//================================================================
wire [5:0] cal_r0, cal_r1, cal_r2, cal_r3, cal_r4, cal_r5;
wire [7:0] s_in0, s_in1, s_in2, s_in3, s_in4, s_in5;
wire [7:0] sorted_0, sorted_1, sorted_2, sorted_3, sorted_4, sorted_5;



//================================================================
//    DESIGN
//================================================================

// --------------------------------------------------
// write your design here
// --------------------------------------------------

/*Calculate Id or gm*/

Idgm_calculator cal0(
    .vgs0(V_GS_0), .vds0(V_DS_0),
    .vgs1(V_GS_1), .vds1(V_DS_1),
    .vgs2(V_GS_2), .vds2(V_DS_2),
    .vgs3(V_GS_3), .vds3(V_DS_3),
    .vgs4(V_GS_4), .vds4(V_DS_4),
    .vgs5(V_GS_5), .vds5(V_DS_5),
    .idgm_mode(mode[0]),
    .out0(cal_r0), .out1(cal_r1), .out2(cal_r2),
    .out3(cal_r3), .out4(cal_r4), .out5(cal_r5)
);





assign s_in0 = (cal_r0*W_0);
assign s_in1 = (cal_r1*W_1);
assign s_in2 = (cal_r2*W_2);
assign s_in3 = (cal_r3*W_3);
assign s_in4 = (cal_r4*W_4);
assign s_in5 = (cal_r5*W_5);

/*Sort*/
Sort s1(
    .in0(s_in0), .in1(s_in1), .in2(s_in2), .in3(s_in3), .in4(s_in4), .in5(s_in5),
    .s_out0(sorted_0), .s_out1(sorted_1), .s_out2(sorted_2),
    .s_out3(sorted_3), .s_out4(sorted_4), .s_out5(sorted_5)
);

mm_cal m0(/*.w0(W_0), .w1(W_1), .w2(W_2), .w3(W_3), .w4(W_4), .w5(W_5),*/
      .mm_in0(sorted_0), .mm_in1(sorted_1), .mm_in2(sorted_2), .mm_in3(sorted_3),
	  .mm_in4(sorted_4), .mm_in5(sorted_5),.mm_mode(mode),  .mm_out(out_n));




endmodule


//================================================================
//   SUB MODULE
//================================================================

/*Calculate Id or gm*/
module Idgm_calculator(
  //Input
  vgs0, vds0,
  vgs1, vds1,
  vgs2, vds2,
  vgs3, vds3,
  vgs4, vds4,
  vgs5, vds5,
  idgm_mode,
  //Output
  out0, out1, out2,
  out3, out4, out5
);
  input [2:0]vgs0, vds0;
  input [2:0]vgs1, vds1;
  input [2:0]vgs2, vds2;
  input [2:0]vgs3, vds3;
  input [2:0]vgs4, vds4;
  input [2:0]vgs5, vds5;
  input idgm_mode;

  output [5:0]out0, out1, out2;
  output [5:0]out3, out4, out5;


  wire [5:0]dc0, dc1, dc2, dc3, dc4, dc5;
  wire [5:0]gm0, gm1, gm2, gm3, gm4, gm5;
  wire [2:0] vgs_m0, vgs_m1, vgs_m2, vgs_m3, vgs_m4, vgs_m5;

  assign vgs_m0 = vgs0 - 1;
  assign vgs_m1 = vgs1 - 1;
  assign vgs_m2 = vgs2 - 1;
  assign vgs_m3 = vgs3 - 1;
  assign vgs_m4 = vgs4 - 1;
  assign vgs_m5 = vgs5 - 1;


  drain_gm_cal d0( .V_GS(vgs0), .V_DS(vds0), .vgs_minus(vgs_m0), .drain_out(dc0), .gm_out(gm0));
  drain_gm_cal d1( .V_GS(vgs1), .V_DS(vds1), .vgs_minus(vgs_m1), .drain_out(dc1), .gm_out(gm1));
  drain_gm_cal d2( .V_GS(vgs2), .V_DS(vds2), .vgs_minus(vgs_m2), .drain_out(dc2), .gm_out(gm2));
  drain_gm_cal d3( .V_GS(vgs3), .V_DS(vds3), .vgs_minus(vgs_m3), .drain_out(dc3), .gm_out(gm3));
  drain_gm_cal d4( .V_GS(vgs4), .V_DS(vds4), .vgs_minus(vgs_m4), .drain_out(dc4), .gm_out(gm4));
  drain_gm_cal d5( .V_GS(vgs5), .V_DS(vds5), .vgs_minus(vgs_m5), .drain_out(dc5), .gm_out(gm5));

  assign {out0, out1} = idgm_mode ? {dc0, dc1} : {gm0, gm1};
  assign {out2, out3} = idgm_mode ? {dc2, dc3} : {gm2, gm3};
  assign {out4, out5} = idgm_mode ? {dc4, dc5} : {gm4, gm5};


endmodule

// calculate id and gm
module drain_gm_cal(
  //input
  V_GS,
  V_DS,
  vgs_minus,
  //output
  drain_out,
  gm_out
);
  
  input [2:0]V_GS;
  input [2:0]V_DS;
  input [2:0]vgs_minus;

  output [5:0]drain_out, gm_out;

  wire ts_mode;
  wire [5:0]tri_dc, sat_dc;
  wire [5:0]tri_gm, sat_gm;
  wire [4:0] vds_sq, vgsm_mul_vds;

  //mul msq0 (.mul_in(V_DS), .mul_out(vds_sq) );
  //mul_gd mgd0 (.vgs_in(vgs_minus), .vds_in(V_DS), .mul_out(vgsm_mul_vds));

  assign ts_mode = vgs_minus > V_DS;



  //assign tri_dc = (( (vgsm_mul_vds)<<1 ) - vds_sq );

  cal_tri_id ctd0(.vgsm_in(vgs_minus), .vds_in(V_DS), .dc_out(tri_dc));
  
  assign sat_dc = (vgs_minus)*(vgs_minus);

  assign tri_gm = ((V_DS)<<1);
  assign sat_gm = ((vgs_minus)<<1);

  assign {drain_out, gm_out} = ts_mode ? {tri_dc, tri_gm} : {sat_dc, sat_gm};



endmodule

module Sort(
    in0, in1, in2, in3, in4, in5,
    s_out0, s_out1, s_out2, s_out3, s_out4, s_out5
);

  input [7:0]in0, in1, in2, in3, in4, in5;

  output [7:0]s_out0, s_out1, s_out2, s_out3, s_out4, s_out5;


  wire [7:0] p0_0, p0_1, p0_2, p0_3, p0_4, p0_5;
  wire [7:0] p1_0, p1_1, p1_2, p1_3, p1_4, p1_5;
  wire [7:0] p2_0, p2_1, p2_2, p2_3, p2_4, p2_5;
  wire [7:0] p3_0, p3_1, p3_2, p3_3, p3_4, p3_5;
  wire [7:0] p4_0, p4_1, p4_2, p4_3, p4_4, p4_5;

  /*---------p0--------*/
  assign p0_0 = in0 >= in5 ? in0 : in5;
  assign p0_5 = in0 >= in5 ? in5 : in0;
  assign p0_1 = in1 >= in3 ? in1 : in3;
  assign p0_3 = in1 >= in3 ? in3 : in1;
  assign p0_2 = in2 >= in4 ? in2 : in4;
  assign p0_4 = in2 >= in4 ? in4 : in2;
  /*---------p1--------*/
  assign p1_1 = p0_1 >= p0_2 ? p0_1 : p0_2;
  assign p1_2 = p0_1 >= p0_2 ? p0_2 : p0_1;
  assign p1_3 = p0_3 >= p0_4 ? p0_3 : p0_4;
  assign p1_4 = p0_3 >= p0_4 ? p0_4 : p0_3;
  assign p1_0 = p0_0;
  assign p1_5 = p0_5;
  /*---------p2--------*/
  assign p2_0 = p1_0 >= p1_3 ? p1_0 : p1_3;
  assign p2_3 = p1_0 >= p1_3 ? p1_3 : p1_0;
  assign p2_2 = p1_2 >= p1_5 ? p1_2 : p1_5;
  assign p2_5 = p1_2 >= p1_5 ? p1_5 : p1_2;
  assign p2_1 = p1_1;
  assign p2_4 = p1_4;

  /*---------p3--------*/
  assign p3_0 = p2_0 >= p2_1 ? p2_0 : p2_1;
  assign p3_1 = p2_0 >= p2_1 ? p2_1 : p2_0;
  assign p3_2 = p2_2 >= p2_3 ? p2_2 : p2_3;
  assign p3_3 = p2_2 >= p2_3 ? p2_3 : p2_2;
  assign p3_4 = p2_4 >= p2_5 ? p2_4 : p2_5;
  assign p3_5 = p2_4 >= p2_5 ? p2_5 : p2_4;
  /*---------p4--------*/
  assign p4_1 = p3_1 >= p3_2 ? p3_1 : p3_2;
  assign p4_2 = p3_1 >= p3_2 ? p3_2 : p3_1;
  assign p4_3 = p3_3 >= p3_4 ? p3_3 : p3_4;
  assign p4_4 = p3_3 >= p3_4 ? p3_4 : p3_3;
  assign p4_0 = p3_0;
  assign p4_5 = p3_5;
  /*------output-------*/
  assign s_out0 = p4_0;
  assign s_out1 = p4_1;
  assign s_out2 = p4_2;
  assign s_out3 = p4_3;
  assign s_out4 = p4_4;
  assign s_out5 = p4_5;

endmodule

module mm_cal(
  //input
  mm_in0, mm_in1, mm_in2, mm_in3, mm_in4, mm_in5,
  /*w0, w1, w2, w3, w4, w5,*/
  mm_mode,
  //output
  mm_out
);

input [7:0] mm_in0, mm_in1, mm_in2, mm_in3, mm_in4, mm_in5;
/*input [2:0] w0, w1, w2, w3, w4, w5;*/
input [1:0] mm_mode;
output [7:0] mm_out;

wire [7:0] tmp_Iavg,  out_sum;

wire [7:0] out_n0, out_n1, out_n2;
wire [6:0] f_out_n0, f_out_n1, f_out_n2;
wire [6:0] tmp_out;

assign out_n0 = mm_mode[1] ? mm_in0 : mm_in3;
assign out_n1 = mm_mode[1] ? mm_in1 : mm_in4;
assign out_n2 = mm_mode[1] ? mm_in2 : mm_in5;

divide_by_3_lut dv00 (.div3_in(out_n0), .div3_out(f_out_n0));
divide_by_3_lut dv01 (.div3_in(out_n1), .div3_out(f_out_n1));
divide_by_3_lut dv02 (.div3_in(out_n2), .div3_out(f_out_n2));

assign out_sum = f_out_n0 + f_out_n1 + f_out_n2;


assign tmp_Iavg = mm_mode[0] ? ((4*out_sum)+(f_out_n2-f_out_n0))>>2 : out_sum;


divide_by_3_lut dv03 (.div3_in(tmp_Iavg), .div3_out(tmp_out));
assign mm_out = {0, tmp_out};

/*div3_out dv0(.div3_in(tmp_Iavg), .div3_out(mm_out));*/




endmodule


module divide_by_3_lut (
    div3_in,
    div3_out
);

  input [7:0] div3_in;
  output [6:0] div3_out;
  reg [6:0] y;

  assign div3_out = y;

  always @(*) begin
      case(div3_in)
        8'd3: y = 7'd1;
        8'd4: y = 7'd1;
        8'd5: y = 7'd1;
        8'd6: y = 7'd2;
        8'd7: y = 7'd2;
        8'd8: y = 7'd2;
        8'd9: y = 7'd3;
        8'd10: y = 7'd3;
        8'd11: y = 7'd3;
        8'd12: y = 7'd4;
        8'd13: y = 7'd4;
        8'd14: y = 7'd4;
        8'd15: y = 7'd5;
        8'd16: y = 7'd5;
        8'd17: y = 7'd5;
        8'd18: y = 7'd6;
        8'd19: y = 7'd6;
        8'd20: y = 7'd6;
        8'd21: y = 7'd7;
        8'd22: y = 7'd7;
        8'd23: y = 7'd7;
        8'd24: y = 7'd8;
        8'd25: y = 7'd8;
        8'd26: y = 7'd8;
        8'd27: y = 7'd9;
        8'd28: y = 7'd9;
        8'd29: y = 7'd9;
        8'd30: y = 7'd10;
        8'd31: y = 7'd10;
        8'd32: y = 7'd10;
        8'd33: y = 7'd11;
        8'd34: y = 7'd11;
        8'd35: y = 7'd11;
        8'd36: y = 7'd12;
        8'd37: y = 7'd12;
        8'd38: y = 7'd12;
        8'd39: y = 7'd13;
        8'd40: y = 7'd13;
        8'd41: y = 7'd13;
        8'd42: y = 7'd14;
        8'd43: y = 7'd14;
        8'd44: y = 7'd14;
        8'd45: y = 7'd15;
        8'd46: y = 7'd15;
        8'd47: y = 7'd15;
        8'd48: y = 7'd16;
        8'd49: y = 7'd16;
        8'd50: y = 7'd16;
        8'd51: y = 7'd17;
        8'd52: y = 7'd17;
        8'd53: y = 7'd17;
        8'd54: y = 7'd18;
        8'd55: y = 7'd18;
        8'd56: y = 7'd18;
        8'd57: y = 7'd19;
        8'd58: y = 7'd19;
        8'd59: y = 7'd19;
        8'd60: y = 7'd20;
        8'd61: y = 7'd20;
        8'd62: y = 7'd20;
        8'd63: y = 7'd21;
        8'd64: y = 7'd21;
        8'd65: y = 7'd21;
        8'd66: y = 7'd22;
        8'd67: y = 7'd22;
        8'd68: y = 7'd22;
        8'd69: y = 7'd23;
        8'd70: y = 7'd23;
        8'd71: y = 7'd23;
        8'd72: y = 7'd24;
        8'd73: y = 7'd24;
        8'd74: y = 7'd24;
        8'd75: y = 7'd25;
        8'd76: y = 7'd25;
        8'd77: y = 7'd25;
        8'd78: y = 7'd26;
        8'd79: y = 7'd26;
        8'd80: y = 7'd26;
        8'd81: y = 7'd27;
        8'd82: y = 7'd27;
        8'd83: y = 7'd27;
        8'd84: y = 7'd28;
        8'd85: y = 7'd28;
        8'd86: y = 7'd28;
        8'd87: y = 7'd29;
        8'd88: y = 7'd29;
        8'd89: y = 7'd29;
        8'd90: y = 7'd30;
        8'd91: y = 7'd30;
        8'd92: y = 7'd30;
        8'd93: y = 7'd31;
        8'd94: y = 7'd31;
        8'd95: y = 7'd31;
        8'd96: y = 7'd32;
        8'd97: y = 7'd32;
        8'd98: y = 7'd32;
        8'd99: y = 7'd33;
        8'd100: y = 7'd33;
        8'd101: y = 7'd33;
        8'd102: y = 7'd34;
        8'd103: y = 7'd34;
        8'd104: y = 7'd34;
        8'd105: y = 7'd35;
        8'd106: y = 7'd35;
        8'd107: y = 7'd35;
        8'd108: y = 7'd36;
        8'd109: y = 7'd36;
        8'd110: y = 7'd36;
        8'd111: y = 7'd37;
        8'd112: y = 7'd37;
        8'd113: y = 7'd37;
        8'd114: y = 7'd38;
        8'd115: y = 7'd38;
        8'd116: y = 7'd38;
        8'd117: y = 7'd39;
        8'd118: y = 7'd39;
        8'd119: y = 7'd39;
        8'd120: y = 7'd40;
        8'd121: y = 7'd40;
        8'd122: y = 7'd40;
        8'd123: y = 7'd41;
        8'd124: y = 7'd41;
        8'd125: y = 7'd41;
        8'd126: y = 7'd42;
        8'd127: y = 7'd42;
        8'd128: y = 7'd42;
        8'd129: y = 7'd43;
        8'd130: y = 7'd43;
        8'd131: y = 7'd43;
        8'd132: y = 7'd44;
        8'd133: y = 7'd44;
        8'd134: y = 7'd44;
        8'd135: y = 7'd45;
        8'd136: y = 7'd45;
        8'd137: y = 7'd45;
        8'd138: y = 7'd46;
        8'd139: y = 7'd46;
        8'd140: y = 7'd46;
        8'd141: y = 7'd47;
        8'd142: y = 7'd47;
        8'd143: y = 7'd47;
        8'd144: y = 7'd48;
        8'd145: y = 7'd48;
        8'd146: y = 7'd48;
        8'd147: y = 7'd49;
        8'd148: y = 7'd49;
        8'd149: y = 7'd49;
        8'd150: y = 7'd50;
        8'd151: y = 7'd50;
        8'd152: y = 7'd50;
        8'd153: y = 7'd51;
        8'd154: y = 7'd51;
        8'd155: y = 7'd51;
        8'd156: y = 7'd52;
        8'd157: y = 7'd52;
        8'd158: y = 7'd52;
        8'd159: y = 7'd53;
        8'd160: y = 7'd53;
        8'd161: y = 7'd53;
        8'd162: y = 7'd54;
        8'd163: y = 7'd54;
        8'd164: y = 7'd54;
        8'd165: y = 7'd55;
        8'd166: y = 7'd55;
        8'd167: y = 7'd55;
        8'd168: y = 7'd56;
        8'd169: y = 7'd56;
        8'd170: y = 7'd56;
        8'd171: y = 7'd57;
        8'd172: y = 7'd57;
        8'd173: y = 7'd57;
        8'd174: y = 7'd58;
        8'd175: y = 7'd58;
        8'd176: y = 7'd58;
        8'd177: y = 7'd59;
        8'd178: y = 7'd59;
        8'd179: y = 7'd59;
        8'd180: y = 7'd60;
        8'd181: y = 7'd60;
        8'd182: y = 7'd60;
        8'd183: y = 7'd61;
        8'd184: y = 7'd61;
        8'd185: y = 7'd61;
        8'd186: y = 7'd62;
        8'd187: y = 7'd62;
        8'd188: y = 7'd62;
        8'd189: y = 7'd63;
        8'd190: y = 7'd63;
        8'd191: y = 7'd63;
        8'd192: y = 7'd64;
        8'd193: y = 7'd64;
        8'd194: y = 7'd64;
        8'd195: y = 7'd65;
        8'd196: y = 7'd65;
        8'd197: y = 7'd65;
        8'd198: y = 7'd66;
        8'd199: y = 7'd66;
        8'd200: y = 7'd66;
        8'd201: y = 7'd67;
        8'd202: y = 7'd67;
        8'd203: y = 7'd67;
        8'd204: y = 7'd68;
        8'd205: y = 7'd68;
        8'd206: y = 7'd68;
        8'd207: y = 7'd69;
        8'd208: y = 7'd69;
        8'd209: y = 7'd69;
        8'd210: y = 7'd70;
        8'd211: y = 7'd70;
        8'd212: y = 7'd70;
        8'd213: y = 7'd71;
        8'd214: y = 7'd71;
        8'd215: y = 7'd71;
        8'd216: y = 7'd72;
        8'd217: y = 7'd72;
        8'd218: y = 7'd72;
        8'd219: y = 7'd73;
        8'd220: y = 7'd73;
        8'd221: y = 7'd73;
        8'd222: y = 7'd74;
        8'd223: y = 7'd74;
        8'd224: y = 7'd74;
        8'd225: y = 7'd75;
        8'd226: y = 7'd75;
        8'd227: y = 7'd75;
        8'd228: y = 7'd76;
        8'd229: y = 7'd76;
        8'd230: y = 7'd76;
        8'd231: y = 7'd77;
        8'd232: y = 7'd77;
        8'd233: y = 7'd77;
        8'd234: y = 7'd78;
        8'd235: y = 7'd78;
        8'd236: y = 7'd78;
        8'd237: y = 7'd79;
        8'd238: y = 7'd79;
        8'd239: y = 7'd79;
        8'd240: y = 7'd80;
        8'd241: y = 7'd80;
        8'd242: y = 7'd80;
        8'd243: y = 7'd81;
        8'd244: y = 7'd81;
        8'd245: y = 7'd81;
        8'd246: y = 7'd82;
        8'd247: y = 7'd82;
        8'd248: y = 7'd82;
        8'd249: y = 7'd83;
        8'd250: y = 7'd83;
        8'd251: y = 7'd83;
        8'd252: y = 7'd84;
        /*8'd253: y = 7'd84;
        8'd254: y = 7'd84;
        8'd255: y = 7'd85;*/
        default: y = 7'd0;
      endcase
  end
endmodule

module mul(mul_in, mul_out);
  input [2:0] mul_in;
  output [4:0] mul_out;
  reg [4:0] y;
  always@(*)begin
    case(mul_in)
      3'd2:y = 5'd4;
      3'd3:y = 5'd9;
      3'd4:y = 5'd16;
      3'd5:y = 5'd25;
      default:y = 5'd1;
    endcase
  
  end
  
  assign mul_out = y;


endmodule

module mul_gd(vgs_in, vds_in, mul_out);
  input [2:0] vgs_in, vds_in;
  output [4:0] mul_out;
  reg [4:0] y;
  always@(*)begin
    case({vgs_in, vds_in})
      {3'd6, 3'd5}:y = 5'd30;
      
      {3'd6, 3'd4}:y = 5'd24;
      {3'd5, 3'd4}:y = 5'd20;
      
      {3'd6, 3'd3}:y = 5'd18;
      {3'd5, 3'd3}:y = 5'd15;
      {3'd4, 3'd3}:y = 5'd12;
      
      {3'd6, 3'd2}:y = 5'd12;
      {3'd5, 3'd2}:y = 5'd10;
      {3'd4, 3'd2}:y = 5'd8;
      {3'd3, 3'd2}:y = 5'd6;
    
      default:y = {2'b00, vgs_in};
    endcase
  
  end
  assign mul_out = y;
  
endmodule

module cal_tri_id (vgsm_in, vds_in, dc_out);
  input [2:0] vgsm_in, vds_in;
  output [5:0] dc_out;
  reg [5:0] y;

  always @(*) begin
    case({vgsm_in, vds_in})
      {3'd6, 3'd5}:y = 6'd35;
      
      {3'd6, 3'd4}:y = 6'd32;
      {3'd5, 3'd4}:y = 6'd24;
      
      {3'd6, 3'd3}:y = 6'd27;
      {3'd5, 3'd3}:y = 6'd21;
      {3'd4, 3'd3}:y = 6'd15;
      
      {3'd6, 3'd2}:y = 6'd20;
      {3'd5, 3'd2}:y = 6'd16;
      {3'd4, 3'd2}:y = 6'd12;
      {3'd3, 3'd2}:y = 6'd8;

      {3'd6, 3'd1}:y = 6'd11;
      {3'd5, 3'd1}:y = 6'd9;
      {3'd4, 3'd1}:y = 6'd7;
      {3'd3, 3'd1}:y = 6'd5;
      {3'd2, 3'd1}:y = 6'd3;
    
      default:y = 6'd0;
    endcase
  end

  assign dc_out = y;

endmodule