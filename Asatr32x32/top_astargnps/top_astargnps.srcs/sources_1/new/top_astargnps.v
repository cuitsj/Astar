`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/05/09 17:14:49
// Design Name: 
// Module Name: top_astargnps
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


module top_astargnps(
	input   sys_clk_p,              //系统时钟
	input   sys_clk_n,             //系统时钟
	input   sys_rst_n,             //系统复位
	input   uart_rx,               //串口接收端

	output  uart_tx,            //串口发送端
	output  led0,
	output  led1
);

//系統时钟
wire sys_clk;
//ip核
wire clk_100m;                              //200MHz时钟信号
wire clk_100map;                              //200MHz时钟信号
wire clk_10m;                               //100MHz时钟信号
wire rst_n;                                 //所有模块的复位信号
wire locked;                                //locked信号拉高,锁相环开始稳定输出时钟 

//UART接收
parameter CLK = 28'd100_000_000;
parameter BPS = 17'd115200;
wire [7:0] rx_data;
wire rx_done;

//Astar_gnps
parameter MAP_ROW = 8'd32;
parameter MAP_COLUMN = 8'd32;
parameter MAP_EXP = 8'd5;//2的6次方为64
parameter START_ROW = 8'd1;
parameter START_COLUMN = 8'd1;
parameter END_ROW = 8'd29;
parameter END_COLUMN = 8'd29;

//串口发送
wire [7:0] tx_data;
wire tx_flag;
wire tx_done;

//系统复位与锁相环locked相与,作为其它模块的复位信号 
assign  rst_n = ~sys_rst_n & locked; 

IBUFGDS #(
    .DIFF_TERM ("FALSE"),
    .IBUF_LOW_PWR ("FALSE")
) IBUFGDS_i
(
    .I (sys_clk_p), //差分时钟正端输入
    .IB (sys_clk_n), // 差分时钟负端输入
    .O (sys_clk) //时钟缓冲输出
 );
 
 
clk_wiz_0 clk_wiz_0_i
(
// Clock out ports
.clk_out1(clk_100m),     // output clk_out1
.clk_out2(clk_100map),     // output clk_out2
.clk_out3(clk_10m),     // output clk_out3
// Status and control signals
.reset(sys_rst_n), // input reset
.locked(locked),       // output locked
// Clock in ports
.clk_in1(sys_clk));      // input clk_in1

uart_rx #(
    .CLK(CLK),
    .BPS(BPS)) 
u_uart_rx(
    .clk(clk_100m),
    .rst(rst_n),
    .rx_pin(uart_rx),
    .rx_data(rx_data),
    .rx_done(rx_done)
);

astar_gnps #(
    .MAP_ROW(MAP_ROW),
    .MAP_COLUMN(MAP_COLUMN),
    .MAP_EXP(MAP_EXP),
    .START_ROW(START_ROW),
    .START_COLUMN(START_COLUMN),
    .END_ROW(END_ROW),
    .END_COLUMN(END_COLUMN))
astar_gnps_i(
	.clk(clk_100m),            //系统时钟
	.clk_ap(clk_100map),            //系统时钟
	.clk_ila(clk_100m),
	.rst_n(rst_n),          //复位信号，低电平有效
	.rxprocess_en(rx_done),     //
	.rx_data(rx_data),    //
	.tx_done(tx_done),
	.tx_data(tx_data),
	.tx_flag(tx_flag),
	.led0(led0),
	.led1(led1)
);

uart_tx #(
    .CLK(CLK),
    .BPS(BPS))
u_uart_tx(
    .clk(clk_100m),
    .rst(rst_n),
    .tx_en(tx_flag),
    .tx_data(tx_data),
    .tx_pin(uart_tx),
    .tx_done(tx_done)
);

////例化ILA模块
//ila_top_enps ila_top_enps_i(
//.clk(clk_200m),
//.probe0(enps_done),//1
//.probe1(tx_done),//1
//.probe2(leftspeed),//16
//.probe3(rightspeed),//16
//.probe4(tx_data),//8
//.probe5(tx_flag)//1
//);

endmodule
