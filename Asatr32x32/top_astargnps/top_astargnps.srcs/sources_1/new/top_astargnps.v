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
	input   sys_clk_p,              //ϵͳʱ��
	input   sys_clk_n,             //ϵͳʱ��
	input   sys_rst_n,             //ϵͳ��λ
	input   uart_rx,               //���ڽ��ն�

	output  uart_tx,            //���ڷ��Ͷ�
	output  led0,
	output  led1
);

//ϵ�yʱ��
wire sys_clk;
//ip��
wire clk_100m;                              //200MHzʱ���ź�
wire clk_100map;                              //200MHzʱ���ź�
wire clk_10m;                               //100MHzʱ���ź�
wire rst_n;                                 //����ģ��ĸ�λ�ź�
wire locked;                                //locked�ź�����,���໷��ʼ�ȶ����ʱ�� 

//UART����
parameter CLK = 28'd100_000_000;
parameter BPS = 17'd115200;
wire [7:0] rx_data;
wire rx_done;

//Astar_gnps
parameter MAP_ROW = 8'd32;
parameter MAP_COLUMN = 8'd32;
parameter MAP_EXP = 8'd5;//2��6�η�Ϊ64
parameter START_ROW = 8'd1;
parameter START_COLUMN = 8'd1;
parameter END_ROW = 8'd29;
parameter END_COLUMN = 8'd29;

//���ڷ���
wire [7:0] tx_data;
wire tx_flag;
wire tx_done;

//ϵͳ��λ�����໷locked����,��Ϊ����ģ��ĸ�λ�ź� 
assign  rst_n = ~sys_rst_n & locked; 

IBUFGDS #(
    .DIFF_TERM ("FALSE"),
    .IBUF_LOW_PWR ("FALSE")
) IBUFGDS_i
(
    .I (sys_clk_p), //���ʱ����������
    .IB (sys_clk_n), // ���ʱ�Ӹ�������
    .O (sys_clk) //ʱ�ӻ������
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
	.clk(clk_100m),            //ϵͳʱ��
	.clk_ap(clk_100map),            //ϵͳʱ��
	.clk_ila(clk_100m),
	.rst_n(rst_n),          //��λ�źţ��͵�ƽ��Ч
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

////����ILAģ��
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
