module CC(
    //Input Port
    clk,
    rst_n,
	in_valid,
	mode,
    xi,
    yi,

    //Output Port
    out_valid,
	xo,
	yo
    );

input               clk, rst_n, in_valid;
input       [1:0]   mode;
input       [7:0]   xi, yi;  

output reg          out_valid;
output reg  [7:0]   xo, yo;
//==============================================//
//             Parameter and Integer            //
//==============================================//

parameter IDLE  = 3'd0; 
parameter INPUT = 3'd1;
parameter MODE0 = 3'd2;
parameter CAL0 = 3'd3;
parameter MODE1 = 3'd4;
parameter MODE2 = 3'd5;
parameter OUTPUT = 3'd6;
parameter CAL2 = 3'd7;

integer i;

//==============================================//
//                 reg declaration              //
//==============================================//

reg [2:0] current_state, next_state;

//INPUT 
reg signed [7:0] in_x [0:3] ;
reg signed [7:0] in_y [0:3] ;
reg [1:0] cnt;

//MODE0
reg o_flag; // output finish
reg signed [7:0] xcnt, ycnt;
reg signed [7:0] cur_lb, cur_rb, next_lb, next_rb;
wire signed [31:0] c1, c2;
wire signed [7:0] t_lb, t_rb;



//MODE1
wire signed [6:0] circle_a, circle_b; // coef
wire signed [12:0] circle_c;
wire signed [24:0] distance;
wire signed [24:0] radius;
wire signed [6:0] new_ax, new_ay;
wire signed [6:0] new_bx, new_by;
wire signed [6:0] new_cx, new_cy;
wire signed [6:0] new_dx, new_dy;



//MODE2
wire signed [15:0] area0, area1, area2, area3;
wire signed [16:0] tmp_add1, tmp_add2;
wire signed [16:0] total;
wire signed [15:0] total_area;
//==============================================//
//            FSM State Declaration             //
//==============================================//

// current_state

always @(posedge clk, negedge rst_n) begin
    if(!rst_n)
        current_state <= IDLE;
    else
        current_state <= next_state;
end

//next_state comb

always @(*)begin
    case(current_state)
    IDLE: if(in_valid) next_state = INPUT;
            else next_state = IDLE;
    INPUT: if(cnt == 3'd3 && mode == 2'd0) next_state = MODE0;
            else if(cnt == 3'd3 && mode == 2'd1) next_state = MODE1;
            else if(cnt == 3'd3 && mode == 2'd2) next_state = MODE2;
            else next_state = current_state;
    MODE0: next_state = CAL0;
    CAL0: if (o_flag) next_state = IDLE;
            else next_state = current_state;
    MODE1: next_state = OUTPUT;
    OUTPUT: if (!out_valid) next_state = IDLE;
        else next_state = current_state;
    MODE2: next_state = CAL2;
    CAL2: if (!out_valid) next_state = IDLE;
        else next_state = current_state;
    default next_state = current_state;
    endcase

end

//cnt
always @(posedge clk, negedge rst_n) begin
    if(!rst_n) cnt <= 0;
    else begin
        case(current_state)
        IDLE: if(in_valid) cnt <= cnt + 1;
            else cnt <= cnt;
        INPUT: if(in_valid) cnt <= cnt + 1;
            else cnt <= cnt;
        default: cnt <= 0;
        endcase
    end

end



//==============================================//
//                  Input Block                 //
//==============================================//

//x_in
always @(posedge clk, negedge rst_n) begin
    if(!rst_n)
        for (i=0;i<4;i=i+1)begin
            in_x[i] <= 8'd0;
        end
    else
        case(current_state)
        IDLE: if(in_valid) in_x[cnt] <= xi;
            else in_x[cnt] <= in_x[cnt];
        INPUT: if(in_valid) in_x[cnt] <= xi;
            else in_x[cnt] <= in_x[cnt];
        default: in_x[cnt] <= in_x[cnt];
        endcase
end
//y_in
always @(posedge clk, negedge rst_n) begin
    if(!rst_n)
        for (i=0;i<4;i=i+1)begin
            in_y[i] <= 8'd0;
        end
    else
        case(current_state)
        IDLE: if(in_valid) in_y[cnt] <= yi;
            else in_y[cnt] <= in_y[cnt];
        INPUT: if(in_valid) in_y[cnt] <= yi;
            else in_y[cnt] <= in_y[cnt];
        default: in_y[cnt] <= in_y[cnt];
        endcase
end

//==============================================//
//              Calculation Block1              //
//==============================================//




always @(posedge clk, negedge rst_n)begin
    if (!rst_n) begin
        cur_lb<=0;
        cur_rb<=0;
    end
    else begin
        case(current_state)
        IDLE:begin
            cur_lb<=0;
            cur_rb<=0;
        end
        INPUT:
            if(cnt == 3) begin
                cur_lb <= in_x[2];
                cur_rb <= xi;
            end
            else begin
                cur_lb <= cur_lb;
                cur_rb <= cur_rb;
            end
            
        MODE0:begin 
            cur_lb <= in_x[2];
            cur_rb <= in_x[3];
        end
        CAL0: begin
           if(xcnt == cur_rb)begin
                cur_lb <= next_lb;
                cur_rb <= next_rb;
           end
           else begin
                cur_lb <= cur_lb;
                cur_rb <= cur_rb;
           end
        end
        default: begin
            cur_lb <= cur_lb;
            cur_rb <= cur_rb;
            end
        endcase

    end

end

wire signed [8:0] delta_y;
assign delta_y = in_y[0]-in_y[2];


assign c1 = delta_y*in_x[2]-(in_x[0]-in_x[2])*in_y[2];
assign c2 = delta_y*in_x[3]+(in_x[3]-in_x[1])*in_y[2];
assign t_lb = ((delta_y*in_x[2]-(in_x[0]-in_x[2])*in_y[2]) + (in_x[0]-in_x[2])*(ycnt+1))/delta_y;
assign t_rb = ((delta_y*in_x[3]+(in_x[3]-in_x[1])*in_y[2]) - (in_x[3]-in_x[1])*(ycnt+1))/delta_y;

always @(posedge clk, negedge rst_n)begin
    if (!rst_n) begin
        next_lb<=0;
        next_rb<=0;
    end
    else begin
        case(current_state)
        IDLE:begin
            next_lb<=0;
            next_rb<=0;
        end
        MODE0: begin
            if ( (in_y[0]-in_y[2])*(t_lb-1)-(in_x[0]-in_x[2])*(ycnt+1) < c1 && (in_y[0]-in_y[2])*(t_lb)-(in_x[0]-in_x[2])*(ycnt+1) > c1  ) begin
                next_lb <= (t_lb-1);
            end
            else begin
                next_lb <= t_lb;
            end
            
            if ( (in_y[0]-in_y[2])*(t_rb-1)+(in_x[3]-in_x[1])*(ycnt+1) < c2 && (in_y[0]-in_y[2])*(t_rb)+(in_x[3]-in_x[1])*(ycnt+1) > c2 )begin
                next_rb <= t_rb-1;
            end
            else begin
                next_rb <= t_rb;
            end
        end
        CAL0: begin
            if ( (in_y[0]-in_y[2])*(t_lb-1)-(in_x[0]-in_x[2])*(ycnt+1) < c1 && (in_y[0]-in_y[2])*(t_lb)-(in_x[0]-in_x[2])*(ycnt+1) > c1  ) begin
                next_lb <= (t_lb-1);
            end
            else begin
                next_lb <= t_lb;
            end
            
            if ( (in_y[0]-in_y[2])*(t_rb-1)+(in_x[3]-in_x[1])*(ycnt+1) < c2 && (in_y[0]-in_y[2])*(t_rb)+(in_x[3]-in_x[1])*(ycnt+1) > c2 )begin
                next_rb <= t_rb-1;
            end
            else begin
                next_rb <= t_rb;
            end
        
      

        end
        default: begin
            next_lb <= next_lb;
            next_rb <= next_rb;
            end
        endcase

    end

end


always @(posedge clk, negedge rst_n)begin
    if(!rst_n) xcnt <= 0;
    else begin
        case(current_state)
        IDLE: xcnt<=0;
        INPUT: xcnt <= in_x[2];
        MODE0: xcnt <= xcnt+1;
        CAL0: begin
            if(xcnt < cur_rb) xcnt <= xcnt +1;
            else if (xcnt == cur_rb) xcnt <= next_lb;
            else xcnt <= xcnt;
        end
        endcase


    end
end

always @(posedge clk, negedge rst_n)begin
    if(!rst_n) ycnt <= 0;
    else begin
        case(current_state)
        IDLE:ycnt<=0;
        INPUT: ycnt <= in_y[2];
        MODE0: ycnt <= in_y[2];
        CAL0: begin
            if (xcnt == cur_rb) ycnt <= ycnt + 1;
            else ycnt <= ycnt;
        end
        endcase

    end
end


always @(posedge clk , negedge rst_n)begin
    if(!rst_n) o_flag<=1'b0;
    else begin
        case(current_state)
        IDLE: o_flag <=0;
        CAL0: if(xcnt == in_x[1] && ycnt == in_y[1]) o_flag <= 1'b1;
            else o_flag <= o_flag;
        default: o_flag <= o_flag;
        endcase
    end
end

//==============================================//
//              Calculation Block2              //
//==============================================//




assign new_ax = in_x[0]-in_x[2];
assign new_ay = in_y[0]-in_y[2];

assign new_bx = in_x[1]-in_x[2];
assign new_by = in_y[1]-in_y[2];

assign new_cx = 0;
assign new_cy = 0;

assign new_dx = in_x[3]-in_x[2];
assign new_dy = in_y[3]-in_y[2];

assign circle_a = new_ay-new_by;
assign circle_b = new_bx-new_ax;
assign circle_c = (new_ax-new_bx)*new_by+(new_by-new_ay)*new_bx;

assign distance = circle_c*circle_c;
assign radius = ((new_dx*new_dx)+(new_dy*new_dy))*(circle_a*circle_a+circle_b*circle_b);



//==============================================//
//              Calculation Block3              //
//==============================================//
assign area0 = in_x[0]*in_y[1] - in_x[1]*in_y[0];
assign area1 = in_x[1]*in_y[2] - in_x[2]*in_y[1];
assign area2 = in_x[2]*in_y[3] - in_x[3]*in_y[2];
assign area3 = in_x[3]*in_y[0] - in_x[0]*in_y[3];

assign tmp_add1 = area0 + area1;
assign tmp_add2 = area2 + area3;
assign total = ((tmp_add1+tmp_add2) > 0)?(tmp_add1+tmp_add2):(tmp_add1+tmp_add2)*(-1);
assign total_area = total/2;


//==============================================//
//                Output Block                  //
//==============================================//








always @(posedge clk , negedge rst_n)begin
    if(!rst_n)begin
        xo<=8'd0;
    end
    else begin
        case(current_state)
        IDLE: xo<=0;
        MODE0: xo <= in_x[2];
        CAL0: begin
            xo <= xcnt;
        end
        MODE2: xo <= total_area[15:8];
        default: xo<=xo;
        endcase
    end

end

always @(posedge clk , negedge rst_n)begin
    if(!rst_n)begin
        yo<=8'd0;
    end
    else begin
       case(current_state)
       IDLE: yo<=0;
        MODE0: yo<=in_y[2];
        CAL0:  yo <= ycnt;
        MODE1: if(distance > radius) yo <= 0;
            else if (distance == radius) yo <= 2;
            else yo <= 1; 
        MODE2: yo <= total_area[7:0];    
        default: yo <= yo;
        endcase
    end

end

always @(posedge clk , negedge rst_n)begin
    if(!rst_n) out_valid<=1'b0;
    else begin
        case(current_state)
        MODE0: out_valid<=1;
        CAL0: begin
                if(xo == in_x[1] && yo == in_y[1]) out_valid <=1'b0;
                else if(o_flag ==1 && ycnt != in_y[1]) out_valid <= 1'b0;
                else out_valid <= 1;
            end
        MODE2: out_valid <= 1'b1;
        MODE1: out_valid<=1'b1;
        OUTPUT: out_valid<=1'b0;
        CAL2: out_valid <= 1'b0;
        default: out_valid <= 0;
        endcase
    end
end






endmodule 