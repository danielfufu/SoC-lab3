`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/20/2023 10:38:55 AM
// Design Name: 
// Module Name: fir_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module fir_tb
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11,
    parameter Data_Num    = 600
)();
    wire                        awready;
    wire                        wready;
    reg                         awvalid;
    reg   [(pADDR_WIDTH-1): 0]  awaddr;
    reg                         wvalid;
    reg signed [(pDATA_WIDTH-1) : 0] wdata;
    wire                        arready;
    reg                         rready;
    reg                         arvalid;
    reg         [(pADDR_WIDTH-1): 0] araddr;
    wire                        rvalid;
    wire signed [(pDATA_WIDTH-1): 0] rdata;
    reg                         ss_tvalid;
    reg signed [(pDATA_WIDTH-1) : 0] ss_tdata;
    reg                         ss_tlast;
    wire                        ss_tready;
    reg                         sm_tready;
    wire                        sm_tvalid;
    wire signed [(pDATA_WIDTH-1) : 0] sm_tdata;
    wire                        sm_tlast;
    reg                         axis_clk;
    reg                         axis_rst_n;

// ram for tap
    wire [3:0]               tap_WE;
    wire                     tap_EN;
    wire [(pDATA_WIDTH-1):0] tap_Di;
    wire [(pADDR_WIDTH-1):0] tap_A;
    wire [(pDATA_WIDTH-1):0] tap_Do;

// ram for data RAM
    wire [3:0]               data_WE;
    wire                     data_EN;
    wire [(pDATA_WIDTH-1):0] data_Di;
    wire [(pADDR_WIDTH-1):0] data_A;
    wire [(pDATA_WIDTH-1):0] data_Do;



    fir fir_DUT(
        .awready(awready),
        .wready(wready),
        .awvalid(awvalid),
        .awaddr(awaddr),
        .wvalid(wvalid),
        .wdata(wdata),
        .arready(arready),
        .rready(rready),
        .arvalid(arvalid),
        .araddr(araddr),
        .rvalid(rvalid),
        .rdata(rdata),
        .ss_tvalid(ss_tvalid),
        .ss_tdata(ss_tdata),
        .ss_tlast(ss_tlast),
        .ss_tready(ss_tready),
        .sm_tready(sm_tready),
        .sm_tvalid(sm_tvalid),
        .sm_tdata(sm_tdata),
        .sm_tlast(sm_tlast),

        // ram for tap
        .tap_WE(tap_WE),
        .tap_EN(tap_EN),
        .tap_Di(tap_Di),
        .tap_A(tap_A),
        .tap_Do(tap_Do),

        // ram for data
        .data_WE(data_WE),
        .data_EN(data_EN),
        .data_Di(data_Di),
        .data_A(data_A),
        .data_Do(data_Do),

        .axis_clk(axis_clk),
        .axis_rst_n(axis_rst_n)

        );
    
    // RAM for tap
    bram11 tap_RAM (
        .CLK(axis_clk),
        .WE(tap_WE),
        .EN(tap_EN),
        .Di(tap_Di),
        .A(tap_A),
        .Do(tap_Do)
    );

    // RAM for data: choose bram11 or bram12
    bram11 data_RAM(
        .CLK(axis_clk),
        .WE(data_WE),
        .EN(data_EN),
        .Di(data_Di),
        .A(data_A),
        .Do(data_Do)
    );

    reg signed [(pDATA_WIDTH-1):0] Din_list[0:(Data_Num-1)];
    reg signed [(pDATA_WIDTH-1):0] golden_list[0:(Data_Num-1)];

    reg [31:0] status;
    reg [31:0]  data_length;
    integer Din, golden, input_data, golden_data, m;

    integer i,j;

    integer k;
    integer num_sent_out, num_result_received;
    reg signed [31:0] coef[0:10]; // fill in coef 
    reg done_received;
    reg last_received;
    integer latency = 0;

    always #5 axis_clk = (~axis_clk);

    initial begin
        $dumpfile("fir.vcd");
        $dumpvars();

        data_length = 0;
        

        coef[0]  =  32'd0;
        coef[1]  = -32'd10;
        coef[2]  = -32'd9;
        coef[3]  =  32'd23;
        coef[4]  =  32'd56;
        coef[5]  =  32'd63;
        coef[6]  =  32'd56;
        coef[7]  =  32'd23;
        coef[8]  = -32'd9;
        coef[9]  = -32'd10;
        coef[10] =  32'd0;

        axis_clk = 0;
		
		//reset
        axis_rst_n = 0;
        arvalid = 0;
        araddr = 0;
        rready = 0;
        awaddr = 0;
        awvalid = 0;
        wdata = 0;
        wvalid = 0;
        ss_tvalid = 0;
        ss_tlast = 0;
        ss_tdata = 0;
        sm_tready = 0;
        done_received = 0;
        num_result_received = 0;
        last_received = 0;
        num_sent_out = 0;


        axis_rst_n = 0;
        #20; axis_rst_n = 1;
        repeat(10) @(posedge axis_clk);
        
        test_case_gen1;
        wait_fir_idle;
        setup_fir;

        fork
            latency_cal;
            stream_in;
            stream_out;
            observe_ap_done;
        join

        $display("latency = %d", latency);

        test_case_gen_auto;
        wait_fir_idle;
        setup_fir;

        fork
            latency_cal;
            stream_in;
            stream_out;
            observe_ap_done;
        join

        $display("latency = %d", latency);

        test_case_gen_auto;
        wait_fir_idle;
        setup_fir;

        fork
            latency_cal;
            stream_in;
            stream_out;
            observe_ap_done;
        join

        $display("latency = %d", latency);
        
        repeat(3) @(posedge axis_clk);
        $finish;
    end


    task wait_fir_idle;begin
        status = 0;
        while(status[2] != 1) begin
            araddr <= 12'h00;
            arvalid <= 1;
            @(posedge axis_clk);
            while(!arready) @(posedge axis_clk);
            arvalid <= 0;
            rready <= 1;
            @(posedge axis_clk);
            while(!rvalid)  @(posedge axis_clk);
            status <= rdata;
            rready <= 0;
            @(posedge axis_clk);
        end 
        done_received = 0;
        last_received = 0;
    end
    endtask

    task setup_fir; begin
        $display("----Start the coefficient input(AXI-lite)----");
        config_write(12'h10, data_length);
        for(k=0; k< Tape_Num; k=k+1) begin
            config_write(12'h20+4*k, coef[k]);
        end
        // read-back and check
        $display(" Check Coefficient ...");
        for(k=0; k < Tape_Num; k=k+1) begin
            config_read_check(12'h20+4*k, coef[k], 32'hffffffff);
        end
        $display(" Tape programming done ...");
        $display(" Start FIR");
        @(posedge axis_clk) config_write(12'h00, 32'h0000_0001);    // ap_start = 1
        $display("----End the coefficient input(AXI-lite)----");
    end
    endtask

    task test_case_gen1; begin
        data_length = 0;
        Din = $fopen("./samples_triangular_wave.dat","r");
        golden = $fopen("./out_gold.dat","r");
        for(m=0;m<Data_Num;m=m+1) begin
            input_data = $fscanf(Din,"%d", Din_list[m]);
            golden_data = $fscanf(golden,"%d", golden_list[m]);
            data_length = data_length + 1;
        end
    end
    endtask
	
	task test_case_gen_auto; begin
        data_length = {$random}%500;
        for(i = 0;i<data_length;i=i+1)begin
            Din_list[i] = $random%1000;
        end

        for(i = 0;i<11;i=i+1)begin
            coef[i] = $random%1000;
        end

        for(i = 0;i<data_length+10;i=i+1)begin
            golden_list[i] = 0;
            for(j=0;j<11;j=j+1)begin
                if((i-j)>=0 && (i-j)<data_length) golden_list[i] = golden_list[i] + Din_list[i-j]*coef[j];
            end
        end
    end
    endtask

    task latency_cal; begin
        latency <= 0;
        while(!done_received)begin
            @(posedge axis_clk)
            latency <= latency + 1;
        end
    end
    endtask

    task config_write;
        input [11:0]    addr;
        input [31:0]    data;
        begin
            awvalid <= 1; awaddr <= addr;
            wvalid  <= 1; wdata <= data;
            @(posedge axis_clk);
            while (!wready) @(posedge axis_clk);
            awvalid <= 0; wvalid <= 0;
            awaddr <= 0; wdata <= 0;
            @(posedge axis_clk);
        end
    endtask

    task config_read_check;
        input [11:0]        addr;
        input signed [31:0] exp_data;
        input [31:0]        mask;
        begin
            araddr <= addr;
            arvalid <= 1;
            @(posedge axis_clk);
            while(!arready) @(posedge axis_clk);
            arvalid <= 0;
            araddr <= 0;
            rready <= 1;
            @(posedge axis_clk);
            while(!rvalid)  @(posedge axis_clk);
            if( (rdata & mask) != (exp_data & mask)) begin
                $display("ERROR: exp = %d, rdata = %d", exp_data, rdata);
                repeat(3) @(posedge axis_clk);
                $finish; 
            end else begin
                $display("OK: exp = %d, rdata = %d", exp_data, rdata);
            end
            arvalid <= 0;
            //@(posedge axis_clk);
        end
    endtask

    task stream_in;begin
        num_sent_out = 0;
        $display("----Start the data input(AXI-Stream)----");
        for(i=0;i<data_length;i=i+1) begin
            if(i == data_length-1) ss_tlast <= 1;
            else ss_tlast <= 0;
            ss(Din_list[i]);
            if(done_received) i=data_length;
            num_sent_out <= num_sent_out + 1;
        end
        ss_tvalid <= 0;
        ss_tlast <= 0;
        ss_tdata <= 0;
        @(posedge axis_clk);
        if(num_sent_out != data_length)begin
            $display("------done detected before all data sent out------");
            repeat(3) @(posedge axis_clk);
            $finish; 
        end
        $display("------End the data input(AXI-Stream)------");
    end
    endtask

    task stream_out; begin
        
        k = 0;
        while(last_received != 1 && !done_received) begin
            sm(golden_list[k],k,last_received);
            k = k+1;
        end
        sm_tready <= 0;
        if(k != data_length)begin
            $display("------early ending result transmission------");
            repeat(3) @(posedge axis_clk);
            $finish; 
        end
    end
    endtask

    task observe_ap_done; begin
        status <= 0;
        while(status[1] != 1) begin
            araddr <= 12'h00;
            arvalid <= 1;
            @(posedge axis_clk);
            while(!arready) @(posedge axis_clk);
            arvalid <= 0;
            rready <= 1;
            @(posedge axis_clk);
            while(!rvalid)  @(posedge axis_clk);
            status <= rdata;
            rready <= 0;
            @(posedge axis_clk);
        end 
        done_received <= 1;
    end
    endtask
    
    task ss;
        input  signed [31:0] in1;
        begin
            ss_tvalid <= 1;
            ss_tdata  <= in1;
            @(posedge axis_clk);
            while (!ss_tready && !done_received) begin
                @(posedge axis_clk);
            end
        end
    endtask

    task sm;
        input  signed [31:0] in2; // golden data
        input         [31:0] pcnt; // pattern count
        output last;
        begin
            sm_tready <= 1;
            @(posedge axis_clk);
            while(!sm_tvalid && !done_received) @(posedge axis_clk);
            if(!done_received)begin
                last <= sm_tlast;
                if (sm_tdata != in2) begin
                    $display("[ERROR] [Pattern %d] Golden answer: %d, Your answer: %d", pcnt, in2, sm_tdata);
                    repeat(3) @(posedge axis_clk);
                    $finish; 
                end
                else begin
                    $display("[PASS] [Pattern %d] Golden answer: %d, Your answer: %d", pcnt, in2, sm_tdata);
                end
            end
            @(posedge axis_clk);
        end
    endtask

    
    
    
endmodule

