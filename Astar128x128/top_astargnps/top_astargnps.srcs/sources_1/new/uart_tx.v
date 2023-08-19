module uart_tx#(
    parameter CLK = 200_000_000,
    parameter BPS = 115200
)(
    input clk,
    input rst,
    input tx_en,
    input [7:0] tx_data,
    output reg tx_pin,
    output reg tx_done
);

localparam BPS_CNT = CLK/BPS;

//    reg tx_enr;
//    reg tx_enrr;
    reg [7:0] data_buff;
    reg tx_flag;
    reg [15:0] clk_cnt;
    reg [ 3:0] tx_cnt;

//    wire tx_start;
//
//    //捕获rx_done的上升沿
//    assign tx_start = (~tx_enrr)&tx_enr;
//    always @(posedge clk) begin
//        if (!rst) begin
//            tx_enr <= 0;
//            tx_enrr <= 0;
//        end
//        else begin
//            tx_enr <= tx_en;
//            tx_enrr <= tx_enr;
//        end
//    end

    //缓存发送的数据，发送标志置位
    always @(posedge clk) begin
        if (!rst) begin
            tx_flag <= 0;
            data_buff <= 0;
            tx_done <= 1;
        end
        else if (tx_en) begin
            tx_flag <= 1;
            data_buff <= tx_data;
            tx_done <= 0;
        end
        else if (tx_cnt == 4'd9 && clk_cnt == BPS_CNT-2) begin//发送完成
            tx_flag <= 0;
            data_buff <= 0;
            tx_done <= 1;
        end
        else begin //其他
            tx_flag <= tx_flag;
            data_buff <= data_buff;
            tx_done <= tx_done;
        end
    end

    //波特率计数，发送数据位数计数
    always @(posedge clk) begin
        if (!rst) begin
            clk_cnt <= 0;
            tx_cnt <= 0;
        end
        else if (tx_flag) begin
            if (clk_cnt == BPS_CNT - 1) begin
                clk_cnt <= 0;
                tx_cnt <= tx_cnt + 1'b1;
            end
            else clk_cnt <= clk_cnt + 1'b1;
        end
        else begin //发送过程结束
            clk_cnt <= 0;
            tx_cnt <= 0;
        end
    end

    //将数据输出到发送端口
    always @(posedge clk) begin
        if (!rst) tx_pin <= 1;
        else if (tx_flag) begin
            case (tx_cnt)
                4'd0:tx_pin <= 0;
                4'd1:tx_pin <= data_buff[0]; 
                4'd2:tx_pin <= data_buff[1]; 
                4'd3:tx_pin <= data_buff[2]; 
                4'd4:tx_pin <= data_buff[3]; 
                4'd5:tx_pin <= data_buff[4]; 
                4'd6:tx_pin <= data_buff[5]; 
                4'd7:tx_pin <= data_buff[6]; 
                4'd8:tx_pin <= data_buff[7]; 
                4'd9:tx_pin <= 1; 
                default:; 
            endcase
        end
        else tx_pin <= 1;
    end
endmodule
