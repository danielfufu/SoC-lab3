`timescale 1ns / 1ps

module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    output  wire                     awready,
    output  wire                     wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,
    output  wire                     arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  wire                     rvalid,
    output  reg [(pDATA_WIDTH-1):0] rdata,    
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  wire                     ss_tready, 
    input   wire                     sm_tready, 
    output  reg                     sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  reg                     sm_tlast, 
    
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  reg [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  reg [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);
	parameter IDLE = 2'd00;
	parameter IN = 2'd01;
	parameter COMPUTE = 2'd02;

	reg[1:0] curr_state, next_state;
	reg awvalid_HIGH,wvalid_HIGH,w_done;
	reg arvalid_HIGH,rvalid_HIGH,r_done,rvalid_HIGH_delay,rvalid_HIGH_delay2,rvalid_HIGH_delay3,rvalid_HIGH_delay4;
	reg read_ap, read_len;
	reg [(pADDR_WIDTH-1):0] write_addr,read_addr;
	reg [(pDATA_WIDTH-1):0] write_data,read_data;
	reg [(pDATA_WIDTH-1):0] data_length,data_L,data_L_reg;
	reg [(pDATA_WIDTH-1):0] ap_signal; 
	wire signed[(pDATA_WIDTH-1):0] mult2, mult1;
	wire signed [(pDATA_WIDTH+1):0] mult_result;
	reg signed [(pDATA_WIDTH+1):0] acc;
	reg [(pADDR_WIDTH-1):0] data_w_addr;
	reg [5:0] tap_counter,data_counter,counter,times,last_counter;
	reg compute_counter,output_high,last,ap_can_write,sm_tlast_delay;
	reg [(pDATA_WIDTH-1):0] X_in;
	reg ss_tready_HIGH,ss_tready_HIGH2;
	//FSM
	always@(posedge axis_clk or negedge axis_rst_n)begin
		if(!axis_rst_n)begin
			curr_state <= IDLE;
		end
		else begin
			curr_state <= next_state;
		end
	end
	always@(*)begin
		case(curr_state) 
			IDLE : begin
				if(ap_signal[0]) next_state = IN;
				else next_state = IDLE;
			end
			IN  :begin
				if(sm_tlast && sm_tready && sm_tvalid) next_state = IDLE;
				else if(ss_tready_HIGH2) next_state = COMPUTE;
				else next_state = IN;
			end
			COMPUTE :begin
				if(output_high) next_state = IN;
				else next_state = COMPUTE;
			end
		endcase
	end

	///AXI LITE write
	assign awready = ! awvalid_HIGH;
	assign wready = !wvalid_HIGH&&curr_state==IDLE;
	
	always@(posedge axis_clk or negedge axis_rst_n)begin
		if(!axis_rst_n)begin
			awvalid_HIGH <= 0;
		end
		else if(awvalid_HIGH) awvalid_HIGH <= 0;
		else if(awvalid&&awready) awvalid_HIGH <= 1;
		else if(w_done) awvalid_HIGH <= 0;
		else awvalid_HIGH <= awvalid_HIGH;
	end
	always@(posedge axis_clk or negedge axis_rst_n)begin
		if(!axis_rst_n)begin
			wvalid_HIGH <= 0;
		end
		else if(wvalid_HIGH) wvalid_HIGH <= 0;
		else if(wvalid&&wready) wvalid_HIGH <= 1;
		else if(w_done) wvalid_HIGH <= 0;
		else wvalid_HIGH <= wvalid_HIGH;
	end
	
	always@(posedge axis_clk or negedge axis_rst_n)begin
		if(!axis_rst_n)begin
			w_done <= 0;
		end
		
		else if(wvalid_HIGH&&awvalid_HIGH) w_done <= 1;
		else w_done <= 0;
	end
	always@(posedge axis_clk or negedge axis_rst_n)begin
		if(!axis_rst_n)begin
			write_addr <= 0;
		end
		else if(awvalid&&awready) write_addr <= awaddr;
		else write_addr <= write_addr;
	end
	always@(posedge axis_clk or negedge axis_rst_n)begin
		if(!axis_rst_n)begin
			write_data <= 0;
		end
		else if(wvalid&&wready) write_data <= wdata;
		else write_data <= write_data;
	end
	always@(posedge axis_clk or negedge axis_rst_n)begin
		if(!axis_rst_n)begin
			ap_can_write <= 0;
		end
		else if(write_addr == 'h48) ap_can_write <= 1;
		else if(sm_tlast && sm_tready && sm_tvalid) ap_can_write <= 0;
		else ap_can_write <= ap_can_write;
	end
	//data length
	always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            data_length <= 0;
        end
        else begin
            if(w_done && write_addr == 12'h10) data_length <= write_data;
            else data_length <= data_length;
        end
    end
	///AXI LITE read
	assign tap_EN = 1;
	assign arready = !arvalid_HIGH  && !w_done; 
	assign rvalid = rvalid_HIGH_delay && !rvalid_HIGH_delay2 && !sm_tlast_delay;
	
	always@(posedge axis_clk or negedge axis_rst_n)begin
		if(!axis_rst_n)begin
			arvalid_HIGH <= 0;
		end
		else if(arvalid&&arready) arvalid_HIGH <= 1;
		else if(rvalid_HIGH_delay&&rvalid_HIGH_delay2) arvalid_HIGH <= 0;
		else arvalid_HIGH <= arvalid_HIGH;
	end
	
	always@(posedge axis_clk or negedge axis_rst_n)begin
		if(!axis_rst_n)begin
			rvalid_HIGH_delay <= 0;
		end
		else if(arvalid_HIGH) rvalid_HIGH_delay <= 1;
		else rvalid_HIGH_delay <= 0;
	end
	always@(posedge axis_clk or negedge axis_rst_n)begin
		if(!axis_rst_n)begin
			rvalid_HIGH_delay2 <= 0;
		end
		else if(rvalid_HIGH_delay) rvalid_HIGH_delay2 <= 1;
		else rvalid_HIGH_delay2 <= 0;
	end
	always@(posedge axis_clk or negedge axis_rst_n)begin
		if(!axis_rst_n)begin
			rvalid_HIGH_delay3 <= 0;
		end
		else if(rvalid_HIGH_delay2) rvalid_HIGH_delay3 <= 1;
		else rvalid_HIGH_delay3 <= 0;
	end
	always@(posedge axis_clk or negedge axis_rst_n)begin
		if(!axis_rst_n)begin
			rvalid_HIGH_delay4 <= 0;
		end
		else if(rvalid_HIGH_delay3) rvalid_HIGH_delay4 <= 1;
		else rvalid_HIGH_delay4 <= 0;
	end
	always@(posedge axis_clk or negedge axis_rst_n)begin
		if(!axis_rst_n)begin
			r_done <= 0;
		end
		else if(arvalid_HIGH) r_done <= 1;
		else r_done <= 0;
	end
	always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            read_addr <= 0;
        end
        else begin
            if(arvalid && arready) read_addr <= araddr;
            else read_addr <= read_addr;
        end
    end
	always@(posedge axis_clk or negedge axis_rst_n)begin
		if(!axis_rst_n)begin
			read_ap <= 0;
		end
		else if(arready&&arvalid&&araddr == 12'h00) read_ap <= 1;
		else if(rvalid&&rready) read_ap <= 0;
		else read_ap <= read_ap;
	end
	always@(posedge axis_clk or negedge axis_rst_n)begin
		if(!axis_rst_n)begin
			read_len <= 0;
		end
		else if(arready&&arvalid&&araddr == 12'h10) read_len <= 1;
		else if(rvalid&&rready) read_len <= 0;
		else read_len <= read_len;
	end
	always@(*)begin
		if(read_ap) rdata = ap_signal;
		else if(read_len) rdata = data_length; 
		else rdata = tap_Do;
	end
	//tap in
	assign tap_WE = {4{(w_done)&& write_addr != 12'h10 && write_addr != 12'h00 }};
	assign tap_Di = write_data;
	//tap out
	always@(*)begin
		if(arvalid_HIGH&&read_addr != 12'h10&& read_addr != 12'h00) tap_A =  read_addr - 12'h20;
		else if(w_done)  tap_A = write_addr - 12'h20 ;
		else tap_A = tap_counter;
	end
	//AP signal
	always@(posedge axis_clk or negedge axis_rst_n)begin
		 if(!axis_rst_n)begin
            ap_signal <= 32'h0000_0004; //IDLE
        end
		else if(ap_signal[1]==1 &&rready&&rvalid&&read_ap) ap_signal <= 32'h0000_0004; 
		else if(wvalid_HIGH && write_addr == 12'h00 && ap_can_write) ap_signal <= write_data;
		else if(sm_tready && sm_tvalid && sm_tlast) ap_signal <= 32'h0000_0006;
		else if(curr_state==COMPUTE) ap_signal <= 'h0;
		
	end
	//SSREADY
	assign ss_tready = (curr_state == IN) && !ss_tready_HIGH && !ss_tready_HIGH2 &&!sm_tvalid;
	
	always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            X_in <= 0;
        end
        else if(ss_tready && ss_tvalid) X_in <= ss_tdata;
        else X_in <= X_in;  
    end
	always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            ss_tready_HIGH <= 0;
        end
        else if(ss_tready && ss_tvalid) ss_tready_HIGH <= 1;
        else ss_tready_HIGH <= 0;  
    end
	always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            ss_tready_HIGH2 <= 0;
        end
        else if(ss_tready_HIGH ) ss_tready_HIGH2 <= 1;
        else ss_tready_HIGH2 <= 0;  
    end
	//COMPUTE
	assign mult2 = tap_Do;
    assign mult1 = data_Do;
	assign mult_result = mult1 * mult2;
	

	always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            data_L_reg <= 0;
        end
        else begin
			if(curr_state == IDLE) data_L_reg <= 0;
			else if(output_high && data_L_reg  == 'h 28) data_L_reg <= 0;
            else if(output_high) data_L_reg <= data_L_reg + 4;
            else data_L_reg <= data_L_reg;
        end
    end
	
	always@(posedge axis_clk or negedge axis_rst_n)begin
		if(!axis_rst_n)begin
           acc <= 0;
        end
		else if(ss_tready && ss_tvalid ) acc <= 0;
		else if(curr_state==COMPUTE&&compute_counter==1&&!output_high)  acc <= acc + mult_result ;
		else acc <= acc;
	end
	assign data_EN = 1;
	assign data_WE = {4{ss_tready_HIGH2 || curr_state==IDLE && awvalid_HIGH}};
	assign data_Di = (curr_state==IDLE )?0:X_in;
	always@(*)begin
		if(ss_tready_HIGH2) data_A = data_w_addr ;
		else if(curr_state==IDLE) data_A = counter;
		else if(curr_state==COMPUTE)  data_A = data_counter ;
		else data_A = 0;
	end
	always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
           counter <= 0;
        end
		else if(counter == 12'h28 && awvalid_HIGH) counter <= 0;
        else if(awvalid_HIGH) counter <= counter + 4;
		else counter <= counter;
    end
	always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
           data_w_addr <= 0;
        end
        else if(curr_state == IDLE) data_w_addr <= 0;
		else if(data_w_addr == 12'h28 && ss_tready_HIGH2) data_w_addr <= 0;
        else if(ss_tready_HIGH2) data_w_addr <= data_w_addr + 4;
		else data_w_addr <= data_w_addr;
    end
	always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
           tap_counter <= 0;
        end
        else if(curr_state == IDLE) tap_counter <= 0;
		else if(curr_state == COMPUTE&& compute_counter==1 &&!output_high && tap_counter == 'h28) tap_counter <= 0;
        else if(curr_state == COMPUTE&& compute_counter==1 &&!output_high) tap_counter <= tap_counter + 4;
		else tap_counter <= tap_counter;
    end
	always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
           data_counter <= 4;
        end
        else if(curr_state == IDLE) data_counter <= 4;
		else if(ss_tready && ss_tvalid&&data_counter==0) data_counter <= 'h28;
        else if(curr_state == COMPUTE&& compute_counter==1&&data_counter==0) data_counter <= 'h28;
		else if(ss_tready_HIGH||last_counter==3) data_counter <= data_L_reg;
        else if(curr_state == COMPUTE&& compute_counter==1 &&!output_high) data_counter <= data_counter - 4;
		else data_counter <= data_counter;
    end
	always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
           compute_counter <= 0;
        end
		else if(ss_tready && ss_tvalid ) compute_counter <= 0;
        else if(curr_state == COMPUTE) compute_counter <= compute_counter + 1; 
    end
	always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
           times <= 0;
        end
        else if(curr_state == IDLE) times <= 0;
		else if(ss_tready && ss_tvalid ) times <= 0;
        else if(curr_state == COMPUTE&& compute_counter==1) times <= times + 1;
		else times <= times;
    end
	always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
           output_high <= 0;
        end
        else if(curr_state == IDLE) output_high <= 0;
		else if(ss_tready && ss_tvalid || output_high ) output_high <= 0;
        else if(times==11) output_high <= 1;
		else output_high <= output_high;
    end
	//sm signal
	assign sm_tdata = acc;
	always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
           sm_tvalid <= 0;
        end
        else if(sm_tvalid && sm_tready) sm_tvalid <= 0;
		else if(output_high && sm_tready && !ss_tready)sm_tvalid <= 1;
        else sm_tvalid <= sm_tvalid;
    end
	
	always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            sm_tlast <= 0;
        end
        else begin
            if(sm_tvalid && sm_tready) sm_tlast <= 0;
            else if(output_high && sm_tready) sm_tlast <= (data_L == data_length - 1);
            else sm_tlast <= sm_tlast;
        end
    end
	always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            data_L <= 0;
        end
        else begin
			if(curr_state == IDLE) data_L <= 0;
            else if(output_high) data_L <= data_L + 1;
            else data_L <= data_L;
        end
    end
	always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            sm_tlast_delay <= 0;
        end
        else begin
			if(sm_tlast_delay) sm_tlast_delay <= 0;
            else if(sm_tlast) sm_tlast_delay <= 1;
            else sm_tlast_delay <= 0;
        end
    end
	/*always@(posedge axis_clk or negedge axis_rst_n)begin
		if(!axis_rst_n)begin
            last_counter <= 0;
        end
        else begin
			if(curr_state == IN) last_counter <= last_counter +1;
			else if(curr_state == COMPUTE || curr_state == IDLE) last_counter <=0;
			else last_counter <= last_counter;
        end
	end
	always@(posedge axis_clk or negedge axis_rst_n)begin
		if(!axis_rst_n)begin
            last <= 0;
        end
		else if(last_counter==3 && curr_state == IN) last<=1;
		else if(curr_state== IDLE) last <=0;
		else last <= last;
	end*/
	
endmodule