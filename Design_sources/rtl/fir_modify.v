module fir
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    // axilite
    output  wire                     awready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,

    output  wire                     wready,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,

    output  wire                     arready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    input   wire                     rready,
    output  reg                      rvalid,
    output  reg  [(pDATA_WIDTH-1):0] rdata,

    // stream slave (Xn)
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  wire                     ss_tready, 

    // stream master (Yn)
    input   wire                     sm_tready, 
    output  wire                     sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  wire                     sm_tlast, 
    
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  reg  [(pDATA_WIDTH-1):0] tap_Di,
    output  reg  [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  reg  [(pDATA_WIDTH-1):0] data_Di,
    output  reg  [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    // clk & rst_n
    input   wire                     axis_clk,
    input   wire                     axis_rst_n

);

// ================ CLK & RST rename ================
wire clk, rst_n;
assign clk = axis_clk;
assign rst_n = axis_rst_n;
// ================ PARAMTER ================
wire            w_recv, aw_recv;
wire            AXIWR_tx_trans;
wire            Yn_trans;
wire            Xn_recv;

wire    [4:0]   tap_A_fsm_cnt_offset;
wire	[31:0]  adder_result;
wire    [31:0]  data_multi;
wire    [31:0]  adder_in;

reg             ap_idle;
reg             ap_done;
reg             ap_start;

reg     [11:0]  ar_data;
reg             ar_got;
reg             ar_trans;
reg             ar_ready;

reg	[31:0]	multi_result;

reg     [2:0]   curr_state, next_state;
reg     [3:0]   fsm_cnt;
reg     [3:0]   data_ram_wr_ptr, data_ram_rd_ptr;
reg     [31:0]  data_first;
reg     [31:0]  Yn_acc;
reg     [31:0]  Yn_lock;
reg             Yn_lock_valid;

reg     [31:0]  data_length;

reg     [2:0]   axiwr_curr_state, axiwr_next_state;
reg     [11:0]  aw_data;
reg     [31:0]  w_data;

// ================ FIR FSM CONTROL ================
// ======= cnt =======
always@(posedge clk or negedge rst_n) begin
    if(!rst_n)
        fsm_cnt <= 4'd0;
    else if(curr_state != next_state)
        fsm_cnt <= 4'd0;
    else
        fsm_cnt <= (fsm_cnt == 4'b1111)? 4'd0:
                                            fsm_cnt + 4'd1;
end 

// ================ FIR FSM ================
localparam IDLE_ST      = 3'd0;
localparam LOAD_ZERO_ST = 3'd5;     
localparam LOAD_PASS_ST = 3'd1;     
localparam CALCU_ST     = 3'd2;     
localparam FULL_ST      = 3'd3;    

always@(posedge clk or negedge rst_n) begin
    if(!rst_n)
        curr_state <= 3'd0;
    else 
        curr_state <= next_state;
end

always@(*) begin
    case(curr_state)
        IDLE_ST: begin
            next_state = (ap_start)? LOAD_ZERO_ST:
                                     IDLE_ST;
        end
        LOAD_ZERO_ST: begin         //fill up Data RAM with
            next_state = (fsm_cnt == 4'd10)? LOAD_PASS_ST:
                                             LOAD_ZERO_ST;
        end
        LOAD_PASS_ST: begin         //load Data from Xn & PASS result to Yn Lock reg
            next_state = (sm_tlast)? IDLE_ST:        //Is Xn reach the data_len && transferred
                         (Xn_recv)?  CALCU_ST:                    //If Xn recved
                                     LOAD_PASS_ST;
        end
        CALCU_ST: begin             //Calculate the Yn 
            next_state = (fsm_cnt == 4'd12)? (sm_tvalid && !sm_tready)? FULL_ST:LOAD_PASS_ST :
                                             CALCU_ST;
        end
        FULL_ST: begin              //Yn_lock didnt transferred and Yn_acc is finish        //Yn_acc needs to lock (adder must be 0)
            next_state = (sm_tready)? LOAD_PASS_ST:     //wait until the Yn transferred
                                      FULL_ST;
        end
        default: begin
            next_state = IDLE_ST;
        end
    endcase
end

// ================ CALCULATE ================
assign data_multi = data_Do;

always@(posedge clk) begin
	multi_result <= data_multi * tap_Do;
end

assign adder_in = (curr_state == CALCU_ST && fsm_cnt >= 4'd2)? multi_result:
                                                               32'd0;
assign adder_result = Yn_acc + adder_in;

always@(posedge clk or negedge rst_n) begin                         //Yn_acc
    if(!rst_n)
        Yn_acc <= 32'd0;
    else if(curr_state != CALCU_ST && curr_state != FULL_ST)        //reset  
        Yn_acc <= 32'd0;
    else 
        Yn_acc <= adder_result;
end

// ======= ap status =======
always@(posedge clk or negedge rst_n) begin
    if(!rst_n)
        ap_done <= 1'b0;
    else if(sm_tlast)       //set after finish the process
        ap_done <= 1'b1;
    else if(ap_done && (ar_trans && araddr == 12'h000) )   //reset after ap_done read
        ap_done <= 1'b0;
    else 
        ap_done <= ap_done;
end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) 
        ap_idle <= 1'b1;
    else if (ap_done && (ar_trans && araddr == 12'h000))
        ap_idle <= 1'b1;
    else if(ap_start)
        ap_idle <= 1'b0;
    else
        ap_idle <= ap_idle;
end
 
// ================ BRAM CONTROL ================

always@(posedge clk or negedge rst_n) begin     //data_ram_wr_ptr
    if(!rst_n) 
        data_ram_wr_ptr <= 4'd0;
    else if(curr_state == LOAD_PASS_ST && next_state == CALCU_ST)
        data_ram_wr_ptr <= (data_ram_wr_ptr == 4'd10)? 4'd0 : data_ram_wr_ptr + 4'd1;
    else
        data_ram_wr_ptr <= data_ram_wr_ptr;
end

always@(posedge clk or negedge rst_n) begin     //data_ram_rd_ptr
    if(!rst_n) 
        data_ram_rd_ptr <= 4'd0;
    else if(curr_state == LOAD_PASS_ST && next_state == CALCU_ST)
        data_ram_rd_ptr <= data_ram_wr_ptr;
    else if(curr_state == CALCU_ST)
        data_ram_rd_ptr <= (data_ram_rd_ptr == 4'd0)? 4'd10 : data_ram_rd_ptr - 4'd1;
    else
        data_ram_rd_ptr <= data_ram_rd_ptr;
end 

// ================ BRAM ================
// ======= Data =======
assign data_WE = ( (curr_state == LOAD_PASS_ST && Xn_recv) || curr_state == LOAD_ZERO_ST)? {4{1'b1}} : {4{1'b0}};
assign data_EN = (curr_state != IDLE_ST)? 1'b1 : 1'b0;


always@(*) begin     //data_Di
    case(curr_state)
        LOAD_PASS_ST: begin
            data_Di = (Xn_recv)? ss_tdata:
                                 32'd0;
        end
        default: begin
            data_Di = 32'd0;
        end
    endcase
end

always@(*) begin     //data_A
    case(curr_state)
        LOAD_ZERO_ST: begin
            data_A = {6'b0000, fsm_cnt, 2'b00};
        end
        LOAD_PASS_ST: begin
            data_A = (Xn_recv)? {6'b000_000, data_ram_wr_ptr, 2'b00}:
                                12'd0;
        end
        CALCU_ST: begin
            data_A = {4'b0000, data_ram_rd_ptr, 2'b00};
        end
        default: begin
            data_A = 12'd0;
        end
    endcase
end

// ================ Stream ================
// ================ ss (Xn) ================
// ****ss_tlast didnt use.****
assign ss_tready = (curr_state == LOAD_PASS_ST)? 1'b1 : 1'b0;
assign Xn_recv = ss_tvalid && ss_tready;

// ================ sm (Yn) ================
assign Yn_trans = sm_tready && sm_tvalid;
assign sm_tvalid = Yn_lock_valid;
assign sm_tdata = Yn_lock;
assign sm_tlast = ( Yn_lock_valid && (data_length == 32'd1) )? 1'b1 : 1'b0;

always@(posedge clk or negedge rst_n) begin     //Yn_lock
    if(!rst_n) 
        Yn_lock <= 32'd0;
    else if(curr_state == CALCU_ST && next_state == LOAD_PASS_ST)
        Yn_lock <= adder_result;
    else if(curr_state == FULL_ST && next_state == LOAD_PASS_ST)
        Yn_lock <= Yn_acc;
    else
        Yn_lock <= Yn_lock;
end 

always@(posedge clk or negedge rst_n) begin     //Yn_lock_valid
    if(!rst_n)
        Yn_lock_valid <= 1'b0;
    else if( (curr_state == CALCU_ST && next_state == LOAD_PASS_ST) || (curr_state == FULL_ST && next_state == LOAD_PASS_ST) )
        Yn_lock_valid <= 1'b1;
    else if(Yn_trans)
        Yn_lock_valid <= 1'b0;
    else
        Yn_lock_valid <= Yn_lock_valid;
end

// ================ axilite ================

// ================ AXI WR ================
// ================ AXI WR FSM ================

localparam AXIWR_RECV_ADDR_DATA_ST = 3'd0;
localparam AXIWR_RECV_ADDR_ST      = 3'd1;
localparam AXIWR_RECV_DATA_ST      = 3'd2;
localparam AXIWR_TX_DATA_ST        = 3'd3;
localparam AXIWR_IDLE_ST           = 3'd4;
localparam AXIWR_STALL_ST          = 3'd5;

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) 
        axiwr_curr_state <= AXIWR_IDLE_ST;
    else
        axiwr_curr_state <= axiwr_next_state;
end

always@(*) begin
    case(axiwr_curr_state)
        AXIWR_IDLE_ST: begin
            axiwr_next_state = (ap_idle)? AXIWR_RECV_ADDR_DATA_ST:
                                          AXIWR_IDLE_ST;
        end
        AXIWR_RECV_ADDR_DATA_ST: begin
            axiwr_next_state = (!ap_idle)?           AXIWR_IDLE_ST:
                               (aw_recv && w_recv) ? AXIWR_TX_DATA_ST:
                               (aw_recv)?            AXIWR_RECV_DATA_ST:
                               (w_recv)?             AXIWR_RECV_ADDR_ST:
                                                     AXIWR_RECV_ADDR_DATA_ST;
        end
        AXIWR_RECV_ADDR_ST: begin
            axiwr_next_state =  (!ap_idle)? AXIWR_IDLE_ST:
                                (aw_recv)?  AXIWR_TX_DATA_ST:
                                            AXIWR_RECV_ADDR_ST;
        end
        AXIWR_RECV_DATA_ST: begin
            axiwr_next_state =  (!ap_idle)? AXIWR_IDLE_ST:
                                (w_recv)?   AXIWR_TX_DATA_ST:
                                            AXIWR_RECV_DATA_ST;
        end
        AXIWR_TX_DATA_ST: begin
            axiwr_next_state = AXIWR_RECV_ADDR_DATA_ST;          
        end
        default: begin
            axiwr_next_state = AXIWR_IDLE_ST;
        end
    endcase
end 

// ================ AXI WR FSM CONTROL ================

assign awready  = (axiwr_curr_state == AXIWR_RECV_ADDR_DATA_ST || axiwr_curr_state == AXIWR_RECV_ADDR_ST)? 1'b1 : 1'b0;
assign wready   = (axiwr_curr_state == AXIWR_RECV_ADDR_DATA_ST || axiwr_curr_state == AXIWR_RECV_DATA_ST)? 1'b1 : 1'b0;
assign w_recv   = wvalid && wready;
assign aw_recv  = awready && awvalid;
assign AXIWR_tx_trans = (axiwr_curr_state == AXIWR_TX_DATA_ST)? 1'b1 : 1'b0;

// ================ AXI WR DATA ================

always@(posedge clk) begin                 //w_data
    if(w_recv)
        w_data <= wdata;
    else
        w_data <= w_data;
end

always@(posedge clk) begin                 //aw_data
    if(aw_recv)
        aw_data <= awaddr;
    else
        aw_data <= aw_data;
end

// ================ Tap ================
assign tap_EN = 1'b1;
assign tap_WE = (AXIWR_tx_trans)? {4{1'b1}} : {4{1'b0}}; 

// ==== tap_Di ====

always@(*) begin
    if(AXIWR_tx_trans && (aw_data >= 12'h020))
        tap_Di = w_data;
    else   
        tap_Di = 32'd0;                
end

/*
always@(posedge clk or negedge rst_n) begin        
    if(!rst_n)
        tap_Di <= 32'd0;
    else if(axiwr_curr_state == AXIWR_STALL_ST)             
        tap_Di <= (aw_data >= 12'h020)? w_data: 
                                        tap_Di;
    else
        tap_Di <= tap_Di;
end
*/

// ==== tap_A ====
assign tap_A_fsm_cnt_offset = (fsm_cnt == 4'd10)? 4'd0:
                                                 fsm_cnt;

always@(*) begin
    if(curr_state == CALCU_ST && fsm_cnt <= 4'd10)       // Common case
        tap_A = {6'd0, tap_A_fsm_cnt_offset, 2'd0};
    else if(AXIWR_tx_trans && (aw_data >= 12'h020) )          //tap ram will delay one cycle to read, so it must have stall cycle.
        tap_A = aw_data - 12'b0000_0010_0000;        
    else if(arready && arvalid && (araddr >= 12'h020) )
        tap_A = araddr - 12'b0000_0010_0000;
    else
        tap_A = 12'd0;
end

// ================ data_length ================
always@(posedge clk or negedge rst_n) begin     
    if(!rst_n)
        data_length <= 32'd0;
    else if(AXIWR_tx_trans)              
        data_length <= (aw_data == 12'h010)? w_data: 
                                             data_length;
    else if(Yn_trans)
        data_length <= data_length - 32'd1;
    else
        data_length <= data_length;
end

// ================ ap_start ================
always@(posedge clk or negedge rst_n) begin     
    if(!rst_n)
        ap_start <= 1'b0;
    else if(AXIWR_tx_trans && ap_idle)          //set by software
        ap_start <= (aw_data == 12'h000)? w_data: 
                                          ap_start;
    else if(curr_state != IDLE_ST)              //reset by engine
        ap_start <= 1'b0;
    else
        ap_start <= ap_start;
end

// ================ AXI RD ================

assign arready = ( axiwr_curr_state == AXIWR_TX_DATA_ST || (curr_state == CALCU_ST && fsm_cnt <= 4'd10) || (curr_state == LOAD_PASS_ST && Xn_recv) )? 1'b0 : (ar_trans)? 1'b0 : 1'b1;

/*
always@(posedge clk or negedge rst_n) begin         //arready
    if(!rst_n)
    	arready <= 1'b0;
    else if(axiwr_next_state == AXIWR_TX_DATA_ST || (curr_state == CALCU_ST && fsm_cnt <= 4'd8) || (curr_state == LOAD_PASS_ST) || (arready && arvalid) )
        arready <= 1'b0;
    else
        arready <= 1'b1;
end
*/

always@(posedge clk or negedge rst_n) begin         //ar_data
    if(!rst_n)
        ar_data <= 12'd0;
    else if(arready && arvalid)
        ar_data <= araddr;
    else
        ar_data <= ar_data;
end

always@(posedge clk or negedge rst_n) begin     //ar_trans
    if(!rst_n)
        ar_trans <= 1'b0;
    else
        ar_trans <= (arready && arvalid)? 1'b1 : 1'b0;
end

always@(posedge clk or negedge rst_n) begin     //rvalid
    if(!rst_n)
        rvalid <= 1'b0;
    else if(ar_trans)
        rvalid <= 1'b1;
    else if(rvalid && rready)
        rvalid <= 1'b0;
    else
        rvalid <= rvalid;
end

always@(posedge clk or negedge rst_n) begin     //rdata
    if(!rst_n)
        rdata <= 32'd0;
    else if(ar_trans) begin
        case(ar_data[7:0])
            8'h00: begin rdata <= {29'd0, ap_idle, ap_done, ap_start}; end
            8'h10: begin rdata <= data_length; end
            default: begin rdata <= tap_Do; end
        endcase
    end 
    else
        rdata <= rdata;
end

endmodule
