module uart_rx#(
    parameter CLK = 200_000_000,
    parameter BPS = 115200
)(
    input   clk,
    input   rst,
    input   rx_pin,
    
    output reg [7:0] rx_data,
    output reg rx_done
);

//����һλ�����Ƶļ���ֵ
localparam  BPS_CNT = CLK/BPS;

reg rx_pinr;
reg rx_pinrr;
reg rx_flag;
reg [3:0] rx_cnt;
reg [15:0] clk_cnt;
reg [7:0] data_buff;

wire rx_start;

//�½����ǿ�ʼ�ź�
assign rx_start = rx_pinrr &(~rx_pinr);

//����ܽ��źŴ�����
always @(posedge clk) begin
    if (!rst) begin
        rx_pinr <= 0;
        rx_pinrr <= 0;
    end
    else begin
        rx_pinr <= rx_pin;
        rx_pinrr <= rx_pinr;
    end
end

//���չ��̱�־��λ
always @(posedge clk) begin
    if (!rst) begin 
        rx_flag <= 0;
    end
    else if (rx_start) begin//���տ�ʼ
        rx_flag <= 1;
    end
    else if (rx_cnt == 4'd9 && clk_cnt == BPS_CNT/2) begin//���ս���
        rx_flag <= 0;
    end
    else begin//���յĹ����л���û��ʼ
        rx_flag <= rx_flag;
    end
end

 //�����ʼ�������������λ��������
always @(posedge clk) begin
    if (!rst) begin
        clk_cnt <= 0;
        rx_cnt <= 0;
    end
    else if (rx_flag) begin//����״̬����ʼ����
        if (clk_cnt == BPS_CNT - 1) begin//������һλ���ݵ�ʱ��
            clk_cnt <= 0;
            rx_cnt <= rx_cnt + 1'b1;//���յ�����λ������
        end
        else clk_cnt <= clk_cnt + 1'b1;//�����ʼ���
    end
    else begin//���չ��̽���
        clk_cnt <= 0;
        rx_cnt <= 0;
    end
end

//���ź����ϵ����ݴ������ݻ���Ĵ���
always @(posedge clk) begin
    if (!rst) begin
        data_buff <= 0;
        rx_data <= 0;
        rx_done <= 0;
    end
    else if (rx_flag) begin
        if (clk_cnt == BPS_CNT/2) begin
            case (rx_cnt)
                4'd1:data_buff[0] <= rx_pinrr;
                4'd2:data_buff[1] <= rx_pinrr;
                4'd3:data_buff[2] <= rx_pinrr;
                4'd4:data_buff[3] <= rx_pinrr;
                4'd5:data_buff[4] <= rx_pinrr;
                4'd6:data_buff[5] <= rx_pinrr;
                4'd7:data_buff[6] <= rx_pinrr;
                4'd8:begin
                        rx_data <= {rx_pinrr,data_buff[6:0]};
                        rx_done <= 1;
                        data_buff <= 0;
                end
                default:begin
                    data_buff <= 0;
                    rx_data <= 0;
                    rx_done <= 0;
                end
            endcase
        end
        else begin
            data_buff <= data_buff;
            rx_data <= 0;
            rx_done <= 0;
        end
    end
    else begin
        data_buff <= 0;
        rx_data <= 0;
        rx_done <= 0;
    end
end
    
endmodule
