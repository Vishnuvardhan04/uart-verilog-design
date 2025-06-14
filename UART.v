//piso
module piso(data_in,data_out,load,shift,clock,piso_reset);
input [7:0] data_in;
input load,shift,clock,piso_reset;
output data_out;
reg[7:0]data;
always @(posedge clock,negedge piso_reset)
begin
if(!piso_reset)
data<=8'b00000000;
else if(load)
data<=data_in;
else if(shift)
data<={1'b0,data[7:1]};
else
data<=data;
end
assign data_out=data[0];
endmodule
//mux
module tx_mux(out,sel,data_bit_in,parity_bit_in);
output reg out;
input[1:0]sel;
input data_bit_in,parity_bit_in;
always @(sel,data_bit_in,parity_bit_in)
case(sel)
2'b00:out=1'b0;
2'b01:out=data_bit_in;
2'b10:out=parity_bit_in;
2'b11:out=1'b1;
endcase
endmodule
//parity generator
module parity_generator(data_in,parity_bit,load);
input [7:0]data_in;
input load;
output parity_bit;
reg[7:0]data_register;
always@(load,data_in)
if(load)
data_register=data_in;
else
data_register=data_register;
assign parity_bit=^data_register;
endmodule
//transmitter state machine
module transmitter_state_machine(reset,tx_start,select,clock,load,shift);
output reg[1:0]select;
output reg load,shift;
input reset,tx_start,clock;
parameter idle=3'b000,start_bit=3'b001,data_bit=3'b010,parity_bit=3'b011,stop_bit=3'b100;
reg[2:0]state;
reg[2:0]next_state;
integer count=1;
reg count_start=0;

always@(tx_start,state,count)
begin
case(state)
idle:next_state=tx_start?start_bit:idle;
start_bit:next_state=data_bit;
data_bit:begin
count_start=(count==8)?0:1;
next_state=(count==8)?parity_bit:data_bit;
end
parity_bit:
next_state=stop_bit;
stop_bit:
next_state=idle;
default:next_state=idle;
endcase
end
//state
always@(posedge clock,negedge reset)
if(!reset)
state<=idle;
else
state<=next_state;
//counter
always@(posedge clock)
if(count_start)
count<=count+1;
else
count<=1;
//output block
always@(state)
begin
if(state==idle)
begin
select=2'b11;
load=0;
shift=0;
end
else if(state==start_bit) 
begin
select=2'b00;
load=1;
shift=0;
end
else if(state==data_bit) 
begin
select=2'b01;
load=0;
shift=1;
end
else if(state==parity_bit) 
begin
select=2'b10;
load=1;
shift=0;
end
else if(state==stop_bit) 
begin
select=2'b11;
load=0;
shift=0;
end
else begin
select=2'b11;
load=0;
shift=0;
end
end
endmodule

//transmitter
module uart_transmitter(reset,tx_start,data_in,out,clock);
input reset,tx_start,clock;
input[7:0] data_in;
output out;
wire data_out,parity_bit;
wire [1:0] sel;
wire load,shift;
piso piso1(.piso_reset(reset),.clock(clock),.data_in(data_in),.load(load),.shift(shift),.data_out(data_out));
tx_mux tm1(.out(out),.sel(sel),.data_bit_in(data_out),.parity_bit_in(parity_bit));
parity_generator pg1(.data_in(data_in),.parity_bit(parity_bit),.load(load));
transmitter_state_machine tsm1(.reset(reset),.tx_start(tx_start),.select(sel),.clock(clock),.load(load),.shift(shift));
endmodule
//testbench
module uart_transmitter_tb;
reg reset,tx_start,clock=0;
reg[7:0]data_in;
wire out;
uart_transmitter urt1(reset,tx_start,data_in,out,clock);
initial
forever #2 clock=~clock;
initial begin
reset=0;tx_start=0;
#1 reset=1;
#2 tx_start=1;
#48 tx_start=0;
end
initial
begin
data_in=8'b0101_1011;
end
endmodule
/////////////////////////////////////////uart receiver//////////////////////////////////////////////
//master module
module uart_receiver(reset,clock,rx_data,parity_error,data_output,stop_bit_error);
input rx_data;
input reset,clock;
output parity_error,stop_bit_error;
output[7:0]data_output;
wire parity_load,rx_shift,check_stop;
wire[7:0]data_received;
wire start_detect;

assign data_output= (!stop_bit_error)?data_received:8'bx;
start_detector ssd1(.start_bit_receive(rx_data),.start_detect(start_detect));
parity_checker pc1(.data_received(data_received),.parity_bit_rx(rx_data),.parity_error(parity_error),.parity_load(parity_load));
receiver_fsm fs1(.reset(reset),.clock(clock),.parity_load(parity_load),.check_stop(check_stop),.parity_error(parity_error),.start_detect(start_detect),.rx_shift(rx_shift));
sipo sipo1(.rx_data(rx_data),.clock(clock),.sipo_out(data_received),.reset(reset),.rx_shift(rx_shift));
stop_checker sc22(.stop_bit_receive(rx_data),.stop_error(stop_bit_error),.check_stop(check_stop));
endmodule

//serial in parallel output register
module sipo(rx_data,clock,sipo_out,reset,rx_shift);
input clock,rx_shift,reset;
input rx_data;
output [7:0]sipo_out;
reg[7:0]register;

always@(posedge clock,negedge reset)
if(!reset)
register<=8'bxxxx_xxxx;
else if(rx_shift==1)
register<={rx_data,register[7:1]};
else
register<=register;

assign sipo_out=register;
endmodule

////////////////////////////////////////////////////////////////////////////////
//start_bit_detector

module start_detector(start_bit_receive,start_detect);
input start_bit_receive;
output reg start_detect;

always@(start_bit_receive)
if(start_bit_receive==0)begin
start_detect=1;
end
else                begin
start_detect=0;
end
endmodule

//////////////////////////////////////////////////////////////////////////////////////
//parity checker

module parity_checker(data_received,parity_bit_rx,parity_error,parity_load);
input[7:0]data_received;
input parity_bit_rx,parity_load;     //clock
output reg parity_error;

//always@(parity_bit_rx,parity_load)
always@(*)
if(parity_load)  begin
if(parity_bit_rx==^data_received)
parity_error=0;
else
parity_error=1;   end

endmodule


/////////////////////////////////////////////////////////////////////////////////////////////
//stop bit checker

module stop_checker(stop_bit_receive,stop_error,check_stop);
input stop_bit_receive;
input check_stop;
output reg stop_error;

always@(check_stop,stop_bit_receive)
if(check_stop==1)  begin
if(stop_bit_receive==1)
stop_error=0;
else
stop_error=1;      end

endmodule

///////////////////////////////////////////////////////////////////////////////////////////////////
//receiver fsm

module receiver_fsm(reset,clock,parity_load,check_stop,parity_error,start_detect,rx_shift);
input reset,clock;
output reg parity_load,check_stop,rx_shift;

parameter rx_idle=2'b00,rx_data_bit=2'b01,rx_parity_bit=2'b10,rx_stop_bit=2'b11;

input parity_error;
input start_detect;
reg[1:0]state;
reg[1:0]next_state;
integer rx_count=1;
reg start;

always@(state,start_detect,rx_count,parity_error)
begin                                                    //combinational ckt
case(state)
rx_idle:
next_state=(start_detect==1)?rx_data_bit:rx_idle;
rx_data_bit:                                       begin
start=(rx_count==8)?0:1;
next_state=(rx_count==8)?rx_parity_bit:rx_data_bit;    end
rx_parity_bit:
next_state=(parity_error==1)?rx_idle:rx_stop_bit;
rx_stop_bit:
next_state=rx_idle;
endcase
end

always@(posedge clock,negedge reset)         //sequential ckt next state store in state
if(!reset)
state<=rx_idle;
else
state<=next_state;

always@(posedge clock)
if(start)
rx_count<=rx_count+1;
else
rx_count<=1;

always@(state)
begin                                          //output part
if(state==rx_idle)  begin
rx_shift=0;
parity_load=0;
check_stop=0;
end
else if(state==rx_data_bit)  begin
rx_shift=1;
parity_load=0;
check_stop=0;    end
else if(state==rx_parity_bit)  begin
rx_shift=0;
parity_load=1;
check_stop=0;   end
else if(state==rx_stop_bit)   begin
rx_shift=0;
parity_load=0;
check_stop=1;  end
else    begin
rx_shift=0;
parity_load=0;
check_stop=0;  end
end


endmodule

module uart_receiver_tb;
reg reset,clock=0;
reg rx_data;
wire[7:0]data_output;
wire parity_error,stop_bit_error;

uart_receiver urt1(reset,clock,rx_data,parity_error,data_output,stop_bit_error);

initial
forever #2 clock=~clock;

initial
begin
reset=0;
#1 reset=1;
end

initial
begin
rx_data=1;  //start
#3 rx_data=0;
#4 rx_data=1;
#4 rx_data=0;
#4 rx_data=1;
#4 rx_data=0;
#4 rx_data=1;
#4 rx_data=1;
#4 rx_data=0;
#4 rx_data=1;
#4 rx_data=1;
#4 rx_data=1;

end

endmodule


module UART_main(clock, reset, tx_start, data_in, parity_error, stop_bit_error, data_output);
input reset, clock, tx_start;
input [7:0]data_in;
output parity_error, stop_bit_error;
output [7:0]data_output;
wire tx_rx_wire;

uart_receiver ur1(reset,clock,tx_rx_wire,parity_error,data_output,stop_bit_error);
uart_transmitter ut1(reset,tx_start,data_in,tx_rx_wire,clock);

endmodule

module UART_main_tb;
reg reset, tx_start;
reg clock=0;
reg [7:0]data_in;
wire parity_error, stop_bit_error;
wire [7:0]data_output;

 UART_main um1(clock, reset, tx_start, data_in, parity_error, stop_bit_error, data_output);

always #5 clock=~clock;

initial   begin
reset=0;
#3 reset=1;
end

initial  begin
tx_start=0; data_in=8'b1101_0110;
#4 tx_start=1;
#116 tx_start=0;
end
endmodule


module baud_generate_tx(clock,tx_sel,baud_tx_out);
input clock;
input[1:0]tx_sel;
output reg baud_tx_out;
wire baud_tick_115200bps,baud_tick_38400bps,baud_tick_19200bps,baud_tick_9600bps;
integer count1=0;
integer count2=0;
integer count3=0;
integer count4=0;

always@(posedge clock)
if(count1<86)
count1<=count1+1;
else
count1=1;

assign baud_tick_115200bps=(count1<44)?1:0;

always@(posedge clock)
if(count2<260)
count2<=count2+1;
else
count2=1;

assign baud_tick_38400bps=(count2<131)?1:0;

always@(posedge clock)
if(count3<520)
count3<=count3+1;
else
count3=1;

assign baud_tick_19200bps=(count3<261)?1:0;

always@(posedge clock)
if(count4<1042)
count4<=count4+1;
else
count4=1;

assign baud_tick_9600bps=(count4<522)?1:0;

always@(tx_sel,baud_tick_115200bps,baud_tick_38400bps,baud_tick_19200bps,baud_tick_9600bps)
begin
case(tx_sel)
2'b00:baud_tx_out=baud_tick_115200bps;
2'b01:baud_tx_out=baud_tick_38400bps;
2'b10:baud_tx_out=baud_tick_19200bps;
2'b11:baud_tx_out=baud_tick_9600bps;

endcase
end
endmodule




module baud_generate_rx(clock,rx_sel,baud_rx_out);
input clock;
input[1:0]rx_sel;
output reg baud_rx_out;
wire baud_tick_115200bps,baud_tick_38400bps,baud_tick_19200bps,baud_tick_9600bps;
integer count1=0;
integer count2=0;
integer count3=0;
integer count4=0;

always@(posedge clock)
if(count1<68)
count1<=count1+1;
else
count1=1;

assign baud_tick_115200bps=(count1<35)?1:0;

always@(posedge clock)
if(count2<208)
count2<=count2+1;
else
count2=1;

assign baud_tick_38400bps=(count2<105)?1:0;

always@(posedge clock)
if(count3<416)
count3<=count3+1;
else
count3=1;

assign baud_tick_19200bps=(count3<209)?1:0;

always@(posedge clock)
if(count4<832)
count4<=count4+1;
else
count4=1;

assign baud_tick_9600bps=(count4<417)?1:0;

always@(rx_sel,baud_tick_115200bps,baud_tick_38400bps,baud_tick_19200bps,baud_tick_9600bps)
begin
case(rx_sel)
2'b00:baud_rx_out=baud_tick_115200bps;
2'b01:baud_rx_out=baud_tick_38400bps;
2'b10:baud_rx_out=baud_tick_19200bps;
2'b11:baud_rx_out=baud_tick_9600bps;

endcase
end
endmodule

module uart_main1(reset,txclock,tx_start,data_in,parity_error,stop_bit_error,data_output,tx_sel,rx_sel,rxclock);
input reset,txclock,rxclock;
input[1:0]tx_sel,rx_sel;
input tx_start;
input[7:0]data_in;
output parity_error,stop_bit_error;
output[7:0]data_output;
wire tx_rx_wire;
wire baud_tx_out;
wire baud_rx_out;

uart_transmitter uart_tx1(reset,tx_start,data_in,tx_rx_wire,baud_tx_out);
uart_receiver uart_rx1(reset,baud_rx_out,tx_rx_wire,parity_error,data_output,stop_bit_error);
baud_generate_tx bg1(txclock,tx_sel,baud_tx_out);
baud_generate_rx br1(rxclock,rx_sel,baud_rx_out);
endmodule

module uart_main1_tb();
reg tx_start,reset;
reg txclock=0;
reg rxclock=0;
reg[1:0]tx_sel,rx_sel;
reg[7:0]data_in;
wire[7:0]data_output;
wire parity_error,stop_bit_error;

uart_main1 dut(reset,txclock,tx_start,data_in,parity_error,stop_bit_error,data_output,tx_sel,rx_sel,rxclock);

initial
forever #50 txclock=~txclock;

initial
forever #62.5 rxclock=~rxclock;

initial
begin
tx_sel=2'b11;
rx_sel=2'b11;
end

initial
begin
reset=0;tx_start=0;
#60000 reset=1;
#40 tx_start=1;
#48000 tx_start=0;
end

initial
begin
data_in=8'b0101_1011;
//data_in=8'b0101_1000;
end
endmodule