module astar_gnps#(
    parameter MAP_ROW = 8'd16,
    parameter MAP_COLUMN = 8'd16,
    parameter MAP_EXP = 8'd4,//2��6�η�Ϊ64
    parameter START_ROW = 8'd2,
    parameter START_COLUMN = 8'd2,
    parameter END_ROW = 8'd14,
    parameter END_COLUMN = 8'd14,
    parameter OPEN_ROW = 12'd256,
    parameter CLOSE_ROW = 12'd256
)(
	input                          clk,       
	input                          rst_n,         
	input                          rxprocess_en,//�����յ�����
	input [7:0]                   rx_data,    //��������
	input                          tx_done,  //���ڷ������
	
	output reg [7:0]          tx_data,
	output reg                 tx_flag,   // ʹ�ܴ��ڷ���
	output reg                 led0,
    output reg                 led1
);

//Astar-gnps״̬
localparam      S0      = 5'd0;
localparam      S1      = 5'd1;
localparam      S2      = 5'd2;
localparam      S3      = 5'd3;
localparam      S4      = 5'd4;
localparam      S5      = 5'd5;
localparam      S6      = 5'd6;
localparam      S7      = 5'd7;
localparam      S8      = 5'd8;
localparam      S9      = 5'd9;
localparam      S10      = 5'd10;
localparam      S11      = 5'd11;
localparam      S12      = 5'd12;
localparam      S13      = 5'd13;
localparam      S14      = 5'd14;
localparam      S15      = 5'd15;
localparam      S16      = 5'd16;
localparam      S17      = 5'd17;
localparam      S18      = 5'd18;
localparam      S19      = 5'd19;
localparam      S20      = 5'd20;

reg [4:0] current_state;

//Astar-gnpsø����
reg e1,e2,e3,e4,e5,e6,e7,e8,e9,e10,e11,e12,e13,e14,e15,e16,e17,e18,e19,e20,e21,e22,e23,e24,e25,e26,e27,e28,e29,e30,e32,e33,e35_1,e35_2,e35_3;
reg e2_1;
reg [1:0] e31,e34;

//״̬0�����ڽ������ݴ���
reg rxprocess_flag;//��ʼ�����־
reg [7:0] rx_buff;//�������ݻ���
reg [7:0] map_row,map_column;//��ͼ�����м���
(* ram_style="distributed" *) reg map[(MAP_ROW<<MAP_EXP)-1:0];

//״̬1,13,15
reg [11:0]  parent_node[2:0];
reg [11:0] close_row;//closelist������
reg [11:0] min_f;
reg [11:0] open_row3;//���ڴ�openlist�����µĸ��ڵ��������
//closelist
(*ram_style="block"*) reg [7:0] closelist_parenty[CLOSE_ROW-1:0];//���ڵ�y
(*ram_style="block"*) reg [7:0] closelist_parentx[CLOSE_ROW-1:0];//���ڵ�x
(*ram_style="block"*) reg [7:0] closelist_childy[CLOSE_ROW-1:0];//���ڵ�y
(*ram_style="block"*) reg [7:0] closelist_childx[CLOSE_ROW-1:0];//���ڵ�x

//״̬2
reg obs_flag1,obs_flag2,obs_flag3,obs_flag4,obs_flag5,obs_flag6,obs_flag7,obs_flag8;

//״̬3
reg child_flag1,child_flag2,child_flag3,child_flag4,child_flag5,child_flag6,child_flag7,child_flag8;
reg [7:0] child_node1[1:0],child_node2[1:0],child_node3[1:0],child_node4[1:0],child_node5[1:0],child_node6[1:0],child_node7[1:0],child_node8[1:0];
reg [11:0] close_row1,close_row2,close_row3,close_row4,close_row5,close_row6,close_row7,close_row8;

//״̬4,5,6,7,8,9,10,11,12,14
reg [11:0] open_row;//openlist������
reg [11:0] open_row1;
reg [11:0] open_row2;//���ڴ�openlist��ɾ�����ڵ��������
reg inopen_flag1,inopen_flag2,inopen_flag3,inopen_flag4,inopen_flag5,inopen_flag6,inopen_flag7,inopen_flag8;
//openlist
(*ram_style="block"*) reg [7:0] openlist_parenty[OPEN_ROW-1:0];//���ڵ�y
(*ram_style="block"*) reg [7:0] openlist_parentx[OPEN_ROW-1:0];//���ڵ�x
(*ram_style="block"*) reg [7:0] openlist_childy[OPEN_ROW-1:0];//���ڵ�y
(*ram_style="block"*) reg [7:0] openlist_childx[OPEN_ROW-1:0];//���ڵ�x
(*ram_style="block"*) reg [11:0] openlist_g[OPEN_ROW-1:0];//g
(*ram_style="block"*) reg [11:0] openlist_h[OPEN_ROW-1:0];//h
(*ram_style="block"*) reg [11:0] openlist_f[OPEN_ROW-1:0];//f

//״̬16
reg findpath_flag;

//״̬17
reg [11:0] open_row4;//���ڻ������·����������
reg [11:0] close_row9;//���ڻ������·������
reg [7:0] path_row,path_column;//���ڻ���·�����ڵ�����

//״̬18
reg [1:0] tx_cnt;

//�㷨״̬��ת
always @(posedge clk) begin
    if (!rst_n) begin
        current_state <= S20;//��ʼ״̬ΪS20
    end
    else begin
        case (current_state)
            S0:begin
                if (e1) current_state <= S1;
                else current_state <= S0;
            end
            S1:begin
                if (e2 && e2_1) current_state <= S2;
                else current_state <= S1;
            end
            S2:begin
                if (e3 && e4 && e5 && e6 && e7 && e8 && e9 && e10) current_state <= S3;
                else current_state <= S2;
            end
            S3:begin
                if (e11 && e12 && e13 && e14 && e15 && e16 && e17 && e18) current_state <= S4;
                else current_state <= S3;
            end
            S4:begin
                if (e19) current_state <= S5;
                else current_state <= S4;
            end
            S5:begin
                if (e20) current_state <= S6;
                else current_state <= S5;
            end
            S6:begin
                if (e21) current_state <= S7;
                else current_state <= S6;
            end
            S7:begin
                if (e22) current_state <= S8;
                else current_state <= S7;
            end
            S8:begin
                if (e23) current_state <= S9;
                else current_state <= S8;
            end
            S9:begin
                if (e24) current_state <= S10;
                else current_state <= S9;
            end
            S10:begin
                if (e25) current_state <= S11;
                else current_state <= S10;
            end
            S11:begin
                if (e26) current_state <= S12;
                else current_state <= S11;
            end
            S12:begin
                if (e27) current_state <= S13;
                else current_state <= S12;
            end
            S13:begin
                if (e28) current_state <= S14;
                else current_state <= S13;
            end
            S14:begin
                if (e29) current_state <= S15;
                else current_state <= S14;
            end
            S15:begin
                if (e30) current_state <= S16;
                else current_state <= S15;
            end
            S16:begin
                if (e31 == 1) current_state <= S2;//�滮δ����
                else if (e31 == 2) current_state <= S20;//�滮������û���ҵ����·��
                else if (e31 == 3) current_state <= S17;//�㷨�����������ҵ����·��
                else current_state <= S16;//��������ת��������ת
            end
            S17:begin
                if (e32) current_state <= S18;
                else current_state <= S17;
            end
            S18:begin
                if (e33) current_state <= S19;
                else current_state <= S18;
            end
            S19:begin
                if (e34== 1) current_state <= S17;
                else if (e34 == 2) current_state <= S20;
                else current_state <= S19;
            end
            S20:begin
                if (e35_1 && e35_2 && e35_3) current_state <= S0;
                else current_state <= S20;
            end
            default:current_state <= S20;
        endcase
    end
end

//״̬0���յ�ͼ
//��һ��,���洮�ڽ��յ�������
always @(posedge clk) begin
    if (!rst_n) begin
        rxprocess_flag <= 0;
        rx_buff <= 0;
    end
    else if (current_state == S0 && rxprocess_en) begin//״̬0���Ҵ����յ�����
        rxprocess_flag <= 1;
        rx_buff <= rx_data;
    end
    else begin//��û��ʼ�����ڴ���Ĺ�����
        rxprocess_flag <= 0;
        rx_buff <= 0;
    end
end

//���ݼ���
always @(posedge clk) begin
    if(!rst_n) begin
        map_row<= 0;
        map_column<= 0;
        e1 <= 0;
        led0<=1'b1;
    end
    else if (current_state == S0 && rxprocess_flag && e1==0) begin
        if (map_column == MAP_COLUMN-8) begin
            if (map_row == MAP_ROW-1) begin
                map_column<=0;
                map_row <= 0;
                e1 <= 1;//�������
                led0<=~led0;//������һ����ͼLED0��תһ��
            end
            else begin
                map_column<=0;
                map_row<=map_row+8'd1;
            end
        end
        else map_column<=map_column+8'd8;
    end
    else if (current_state == S0 && e1) begin
        e1 <= 0;
    end
end

//���ݴ洢
always @(posedge clk) begin
    if (current_state == S0 && rxprocess_flag) begin
        map[(map_row<<MAP_EXP)+map_column+0]<=rx_buff[0];
        map[(map_row<<MAP_EXP)+map_column+1]<=rx_buff[1];
        map[(map_row<<MAP_EXP)+map_column+2]<=rx_buff[2];
        map[(map_row<<MAP_EXP)+map_column+3]<=rx_buff[3];
        map[(map_row<<MAP_EXP)+map_column+4]<=rx_buff[4];
        map[(map_row<<MAP_EXP)+map_column+5]<=rx_buff[5];
        map[(map_row<<MAP_EXP)+map_column+6]<=rx_buff[6];
        map[(map_row<<MAP_EXP)+map_column+7]<=rx_buff[7];
    end
end

//LED1ָʾ�Ƿ�������·��
always @(posedge clk) begin
    if(!rst_n) begin
        led1<=0;
    end
    else if (current_state == S0 && findpath_flag) begin
        led1<=1;
    end
    else if (current_state == S0 && findpath_flag==0) begin
        led1<=0;
    end
end

//Astar-GNPS
always @(posedge clk) begin
    if (!rst_n) begin
        parent_node[0]<=0;
        parent_node[1]<=0;
        parent_node[2]<=0;
        min_f <= 12'b1111_1111_1111;
        open_row3<=0;
        close_row<=0;
        e2<=0;
        e28<=0;
        e30<=0;
        e35_1 <= 0;
    end
    //״̬20��Ĥ35
    else if (current_state == S20 && e35_1==0) begin
        close_row<=0;
        e35_1 <= 1;
    end
    else if (current_state == S20 && e35_1 && e35_2 && e35_3) begin
        e35_1 <= 0;
    end
    
    //״̬1,Ĥ2
    else if (current_state == S1 && e2 == 0) begin//���������Ϊ���ڵ�
        parent_node[0]<=START_ROW;
        parent_node[1]<=START_COLUMN;
        parent_node[2]<=8'd0;
        e2<=1;
    end
    else if (current_state == S1 && e2 && e2_1) begin
        e2<=0;
    end
    
    //״̬13��Ĥ28�������ڵ���ӵ�closelist��
    else if (current_state == S13 && e28==0) begin
        closelist_parenty[close_row] <= openlist_parenty[parent_node[2]];
        closelist_parentx[close_row] <= openlist_parentx[parent_node[2]];
        closelist_childy[close_row] <= openlist_childy[parent_node[2]];
        closelist_childx[close_row] <= openlist_childx[parent_node[2]];
        close_row <= close_row+1;//closelist������
        e28<=1;
    end
    else if (current_state == S13 && e28) begin
        e28<=0;
    end
    
    //״̬15��Ĥ30����openlist�����µĸ��ڵ�
    else if (current_state == S15 && open_row3 != open_row && e30==0) begin
        if (openlist_f[open_row3] < min_f) begin
            min_f <= openlist_f[open_row3];
            parent_node[0]<=openlist_childy[open_row3];
            parent_node[1]<=openlist_childx[open_row3];
            parent_node[2]<=open_row3;
            open_row3<=open_row3+1;
        end
        else begin
            open_row3<=open_row3+1;
        end
    end
    else if (current_state == S15 && open_row3 == open_row && e30==0) begin
        open_row3<=0;
        min_f <= 12'b1111_1111_1111;
        e30<=1;
    end
    else if (current_state == S15 && e30) begin
        e30<=0;
    end
end

//�ж��ӽڵ�1�Ƿ�Ϊ�ϰ���
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag1 <= 0;
        e3<=0;
    end
    //״̬2��Ĥ3
    else if (current_state == S2 && e3==0 && map[(parent_node[0]<<MAP_EXP)+MAP_ROW+parent_node[1]-1] == 0) begin
        obs_flag1 <= 0;
        e3<=1;
    end
    else if (current_state == S2 && e3==0 && map[(parent_node[0]<<MAP_EXP)+MAP_ROW+parent_node[1]-1] == 1) begin
        obs_flag1 <= 1;
        e3<=1;
    end
    else if (current_state == S2 && e3 && e4 && e5 && e6 && e7 && e8 && e9 && e10) begin
        e3<=0;
    end
end

//�ж��ӽڵ�2�Ƿ�Ϊ�ϰ���
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag2 <= 0;
        e4<=0;
    end
    //״̬2��Ĥ4
    else if (current_state == S2 && e4==0 && map[(parent_node[0]<<MAP_EXP)+MAP_ROW+parent_node[1]] == 0) begin
        obs_flag2 <= 0;
        e4<=1;
    end
    else if (current_state == S2 && e4==0 && map[(parent_node[0]<<MAP_EXP)+MAP_ROW+parent_node[1]] == 1) begin
        obs_flag2 <= 1;
        e4<=1;
    end
    else if (current_state == S2 && e3 && e4 && e5 && e6 && e7 && e8 && e9 && e10) begin
        e4<=0;
    end
end

//�ж��ӽڵ�3�Ƿ�Ϊ�ϰ���
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag3 <= 0;
        e5<=0;
    end
    //״̬2��Ĥ5
    else if (current_state == S2 && e5==0 && map[(parent_node[0]<<MAP_EXP)+MAP_ROW+parent_node[1]+1] == 0) begin
        obs_flag3 <= 0;
        e5<=1;
    end
    else if (current_state == S2 && e5==0 && map[(parent_node[0]<<MAP_EXP)+MAP_ROW+parent_node[1]+1] == 1) begin
        obs_flag3 <= 1;
        e5<=1;
    end
    else if (current_state == S2 && e3 && e4 && e5 && e6 && e7 && e8 && e9 && e10) begin
        e5<=0;
    end
end

//�ж��ӽڵ�4�Ƿ�Ϊ�ϰ���
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag4 <= 0;
        e6<=0;
    end
    //״̬2��Ĥ6
    else if (current_state == S2 && e6==0 && map[(parent_node[0]<<MAP_EXP)+parent_node[1]-1] == 0) begin
        obs_flag4 <= 0;
        e6<=1;
    end
    else if (current_state == S2 && e6==0 && map[(parent_node[0]<<MAP_EXP)+parent_node[1]-1] == 1) begin
        obs_flag4 <= 1;
        e6<=1;
    end
    else if (current_state == S2 && e3 && e4 && e5 && e6 && e7 && e8 && e9 && e10) begin
        e6<=0;
    end
end

//�ж��ӽڵ�5�Ƿ�Ϊ�ϰ���
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag5 <= 0;
        e7<=0;
    end
    //״̬2��Ĥ7
    else if (current_state == S2 && e7==0 && map[(parent_node[0]<<MAP_EXP)+parent_node[1]+1] == 0) begin
        obs_flag5 <= 0;
        e7<=1;
    end
    else if (current_state == S2 && e7==0 && map[(parent_node[0]<<MAP_EXP)+parent_node[1]+1] == 1) begin
        obs_flag5 <= 1;
        e7<=1;
    end
    else if (current_state == S2 && e3 && e4 && e5 && e6 && e7 && e8 && e9 && e10) begin
        e7<=0;
    end
end

//�ж��ӽڵ�6�Ƿ�Ϊ�ϰ���
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag6 <= 0;
        e8<=0;
    end
    //״̬2��Ĥ8
    else if (current_state == S2 && e8==0 && map[(parent_node[0]<<MAP_EXP)-MAP_ROW+parent_node[1]-1] == 0) begin
        obs_flag6 <= 0;
        e8<=1;
    end
    else if (current_state == S2 && e8==0 && map[(parent_node[0]<<MAP_EXP)-MAP_ROW+parent_node[1]-1] == 1) begin
        obs_flag6 <= 1;
        e8<=1;
    end
    else if (current_state == S2 && e3 && e4 && e5 && e6 && e7 && e8 && e9 && e10) begin
        e8<=0;
    end
end

//�ж��ӽڵ�7�Ƿ�Ϊ�ϰ���
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag7 <= 0;
        e9<=0;
    end
    //״̬2��Ĥ9
    else if (current_state == S2 && e9==0 && map[(parent_node[0]<<MAP_EXP)-MAP_ROW+parent_node[1]] == 0) begin
        obs_flag7 <= 0;
        e9<=1;
    end
    else if (current_state == S2 && e9==0 && map[(parent_node[0]<<MAP_EXP)-MAP_ROW+parent_node[1]] == 1) begin
        obs_flag7 <= 1;
        e9<=1;
    end
    else if (current_state == S2 && e3 && e4 && e5 && e6 && e7 && e8 && e9 && e10) begin
        e9<=0;
    end
end

//�ж��ӽڵ�8�Ƿ�Ϊ�ϰ���
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag8 <= 0;
        e10<=0;
    end
    //״̬2��Ĥ10
    else if (current_state == S2 && e10==0 && map[(parent_node[0]<<MAP_EXP)-MAP_ROW+parent_node[1]+1] == 0) begin
        obs_flag8 <= 0;
        e10<=1;
    end
    else if (current_state == S2 && e10==0 && map[(parent_node[0]<<MAP_EXP)-MAP_ROW+parent_node[1]+1] == 1) begin
        obs_flag8 <= 1;
        e10<=1;
    end
    else if (current_state == S2 && e3 && e4 && e5 && e6 && e7 && e8 && e9 && e10) begin
        e10<=0;
    end
end

//�ж��ӽڵ�1�Ƿ���closelist��
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag1 <= 0;
        child_node1[0]<=0;
        child_node1[1]<=0;
        close_row1 <= 0;
        e11<=0;
    end
    //״̬3��Ĥ11
    else if (current_state == S3 && e11==0 && obs_flag1==0 && close_row1!=close_row) begin
        if (parent_node[0]+1 == closelist_childy[close_row1] && parent_node[1]-1 == closelist_childx[close_row1]) begin//��closelist��
            close_row1<=0;
            child_flag1 <= 0;
            child_node1[0]<=0;
            child_node1[1]<=0;
            e11<=1;
        end
        else close_row1 <= close_row1 + 1;
    end
    else if (current_state == S3 && e11==0 && obs_flag1==0 && close_row1==close_row) begin//����closelist��
            close_row1<=0;
            child_flag1 <= 1;
            child_node1[0]<=parent_node[0]+1;
            child_node1[1]<=parent_node[1]-1;
            e11<=1;
    end
    else if (current_state == S3 && e11==0 && obs_flag1) begin//�ӽڵ����ϰ���
        child_flag1 <= 0;
        child_node1[0]<=0;
        child_node1[1]<=0;
        e11<=1;
    end
    else if (current_state == S3 && e11 && e12 && e13 && e14 && e15 && e16 && e17 && e18)begin
        e11<=0;
    end
end

//�ж��ӽڵ�2�Ƿ���closelist��
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag2 <= 0;
        child_node2[0]<=0;
        child_node2[1]<=0;
        close_row2 <= 0;
        e12<=0;
    end
    //״̬3��Ĥ12
    else if (current_state == S3 && e12==0 && obs_flag2==0 && close_row2!=close_row) begin
        if (parent_node[0]+1 == closelist_childy[close_row2] && parent_node[1] == closelist_childx[close_row2]) begin//��closelist��
            close_row2<=0;
            child_flag2 <= 0;
            child_node2[0]<=0;
            child_node2[1]<=0;
            e12<=1;
        end
        else close_row2 <= close_row2 + 1;
    end
    else if (current_state == S3 && e12==0 && obs_flag2==0 && close_row2==close_row) begin//����closelist��
            close_row2<=0;
            child_flag2 <= 1;
            child_node2[0]<=parent_node[0]+1;
            child_node2[1]<=parent_node[1];
            e12<=1;
    end
    else if (current_state == S3 && e12==0 && obs_flag2) begin//�ӽڵ����ϰ���
        child_flag2 <= 0;
        child_node2[0]<=0;
        child_node2[1]<=0;
        e12<=1;
    end
    else if (current_state == S3 && e11 && e12 && e13 && e14 && e15 && e16 && e17 && e18)begin
        e12<=0;
    end
end

//�ж��ӽڵ�3�Ƿ���closelist��
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag3 <= 0;
        child_node3[0]<=0;
        child_node3[1]<=0;
        close_row3 <= 0;
        e13<=0;
    end
    //״̬3��Ĥ13
    else if (current_state == S3 && e13==0 && obs_flag3==0 && close_row3!=close_row) begin
        if (parent_node[0]+1 == closelist_childy[close_row3] && parent_node[1]+1 == closelist_childx[close_row3]) begin//��closelist��
            close_row3<=0;
            child_flag3 <= 0;
            child_node3[0]<=0;
            child_node3[1]<=0;
            e13<=1;
        end
        else close_row3 <= close_row3 + 1;
    end
    else if (current_state == S3 && e13==0 && obs_flag3==0 && close_row3==close_row) begin//����closelist��
            close_row3<=0;
            child_flag3 <= 1;
            child_node3[0]<=parent_node[0]+1;
            child_node3[1]<=parent_node[1]+1;
            e13<=1;
    end
    else if (current_state == S3 && e13==0 && obs_flag3) begin//�ӽڵ����ϰ���
        child_flag3 <= 0;
        child_node3[0]<=0;
        child_node3[1]<=0;
        e13<=1;
    end
    else if (current_state == S3 && e11 && e12 && e13 && e14 && e15 && e16 && e17 && e18)begin
        e13<=0;
    end
end

//�ж��ӽڵ�4�Ƿ���closelist��
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag4 <= 0;
        child_node4[0]<=0;
        child_node4[1]<=0;
        close_row4 <= 0;
        e14<=0;
    end
    //״̬3��Ĥ14
    else if (current_state == S3 && e14==0 && obs_flag4==0 && close_row4!=close_row) begin
        if (parent_node[0] == closelist_childy[close_row4] && parent_node[1]-1 == closelist_childx[close_row4]) begin//��closelist��
            close_row4<=0;
            child_flag4 <= 0;
            child_node4[0]<=0;
            child_node4[1]<=0;
            e14<=1;
        end
        else close_row4 <= close_row4 + 1;
    end
    else if (current_state == S3 && e14==0 && obs_flag4==0 && close_row4==close_row) begin//����closelist��
            close_row4<=0;
            child_flag4 <= 1;
            child_node4[0]<=parent_node[0];
            child_node4[1]<=parent_node[1]-1;
            e14<=1;
    end
    else if (current_state == S3 && e14==0 && obs_flag4) begin//�ӽڵ����ϰ���
        child_flag4 <= 0;
        child_node4[0]<=0;
        child_node4[1]<=0;
        e14<=1;
    end
    else if (current_state == S3 && e11 && e12 && e13 && e14 && e15 && e16 && e17 && e18)begin
        e14<=0;
    end
end

//�ж��ӽڵ�5�Ƿ���closelist��
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag5 <= 0;
        child_node5[0]<=0;
        child_node5[1]<=0;
        close_row5 <= 0;
        e15<=0;
    end
    //״̬3��Ĥ15
    else if (current_state == S3 && e15==0 && obs_flag5==0 && close_row5!=close_row) begin
        if (parent_node[0] == closelist_childy[close_row5] && parent_node[1]+1 == closelist_childx[close_row5]) begin//��closelist��
            close_row5<=0;
            child_flag5 <= 0;
            child_node5[0]<=0;
            child_node5[1]<=0;
            e15<=1;
        end
        else close_row5 <= close_row5 + 1;
    end
    else if (current_state == S3 && e15==0 && obs_flag5==0 && close_row5==close_row) begin//����closelist��
            close_row5<=0;
            child_flag5 <= 1;
            child_node5[0]<=parent_node[0];
            child_node5[1]<=parent_node[1]+1;
            e15<=1;
    end
    else if (current_state == S3 && e15==0 && obs_flag5) begin//�ӽڵ����ϰ���
        child_flag5 <= 0;
        child_node5[0]<=0;
        child_node5[1]<=0;
        e15<=1;
    end
    else if (current_state == S3 && e11 && e12 && e13 && e14 && e15 && e16 && e17 && e18)begin
        e15<=0;
    end
end

//�ж��ӽڵ�6�Ƿ���closelist��
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag6 <= 0;
        child_node6[0]<=0;
        child_node6[1]<=0;
        close_row6 <= 0;
        e16<=0;
    end
    //״̬3��Ĥ16
    else if (current_state == S3 && e16==0 && obs_flag6==0 && close_row6!=close_row) begin
        if (parent_node[0]-1 == closelist_childy[close_row6] && parent_node[1]-1 == closelist_childx[close_row6]) begin//��closelist��
            close_row6<=0;
            child_flag6 <= 0;
            child_node6[0]<=0;
            child_node6[1]<=0;
            e16<=1;
        end
        else close_row6 <= close_row6 + 1;
    end
    else if (current_state == S3 && e16==0 && obs_flag6==0 && close_row6==close_row) begin//����closelist��
            close_row6<=0;
            child_flag6 <= 1;
            child_node6[0]<=parent_node[0]-1;
            child_node6[1]<=parent_node[1]-1;
            e16<=1;
    end
    else if (current_state == S3 && e16==0 && obs_flag6) begin//�ӽڵ����ϰ���
        child_flag6 <= 0;
        child_node6[0]<=0;
        child_node6[1]<=0;
        e16<=1;
    end
    else if (current_state == S3 && e11 && e12 && e13 && e14 && e15 && e16 && e17 && e18)begin
        e16<=0;
    end
end

//�ж��ӽڵ�7�Ƿ���closelist��
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag7 <= 0;
        child_node7[0]<=0;
        child_node7[1]<=0;
        close_row7 <= 0;
        e17<=0;
    end
    //״̬3��Ĥ17
    else if (current_state == S3 && e17==0 && obs_flag7==0 && close_row7!=close_row) begin
        if (parent_node[0]-1 == closelist_childy[close_row7] && parent_node[1] == closelist_childx[close_row7]) begin//��closelist��
            close_row7<=0;
            child_flag7 <= 0;
            child_node7[0]<=0;
            child_node7[1]<=0;
            e17<=1;
        end
        else close_row7 <= close_row7 + 1;
    end
    else if (current_state == S3 && e17==0 && obs_flag7==0 && close_row7==close_row) begin//����closelist��
            close_row7<=0;
            child_flag7 <= 1;
            child_node7[0]<=parent_node[0]-1;
            child_node7[1]<=parent_node[1];
            e17<=1;
    end
    else if (current_state == S3 && e17==0 && obs_flag7) begin//�ӽڵ����ϰ���
        child_flag7 <= 0;
        child_node7[0]<=0;
        child_node7[1]<=0;
        e17<=1;
    end
    else if (current_state == S3 && e11 && e12 && e13 && e14 && e15 && e16 && e17 && e18)begin
        e17<=0;
    end
end

//�ж��ӽڵ�8�Ƿ���closelist��
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag8 <= 0;
        child_node8[0]<=0;
        child_node8[1]<=0;
        close_row8 <= 0;
        e18<=0;
    end
    //״̬3��Ĥ18
    else if (current_state == S3 && e18==0 && obs_flag8==0 && close_row8!=close_row) begin
        if (parent_node[0]-1 == closelist_childy[close_row8] && parent_node[1]+1 == closelist_childx[close_row8]) begin//��closelist��
            close_row8<=0;
            child_flag8 <= 0;
            child_node8[0]<=0;
            child_node8[1]<=0;
            e18<=1;
        end
        else close_row8 <= close_row8 + 1;
    end
    else if (current_state == S3 && e18==0 && obs_flag8==0 && close_row8==close_row) begin//����closelist��
            close_row8<=0;
            child_flag8 <= 1;
            child_node8[0]<=parent_node[0]-1;
            child_node8[1]<=parent_node[1]+1;
            e18<=1;
    end
    else if (current_state == S3 && e18==0 && obs_flag8) begin//�ӽڵ����ϰ���
        child_flag8 <= 0;
        child_node8[0]<=0;
        child_node8[1]<=0;
        e18<=1;
    end
    else if (current_state == S3 && e11 && e12 && e13 && e14 && e15 && e16 && e17 && e18)begin
        e18<=0;
    end
end


//openlist
always @(posedge clk) begin
    if (!rst_n) begin
        open_row<=0;
        open_row1<=0;
        open_row2<=0;
        inopen_flag1 <= 0;
        inopen_flag2 <= 0;
        inopen_flag3 <= 0;
        inopen_flag4 <= 0;
        inopen_flag5 <= 0;
        inopen_flag6 <= 0;
        inopen_flag7 <= 0;
        inopen_flag8 <= 0;
        e19<=0;
        e20<=0;
        e21<=0;
        e22<=0;
        e23<=0;
        e24<=0;
        e25<=0;
        e26<=0;
        e27<=0;
        e29<=0;
        e2_1<=0;
        e35_2 <= 0;
    end
    //״̬20��Ĥ35
    else if (current_state == S20 && e35_2==0) begin
        open_row<=0;
        e35_2 <= 1;
    end
    else if (current_state == S20 && e35_1 && e35_2 && e35_3) begin
        e35_2 <= 0;
    end
    //״̬1,Ĥ2
    else if (current_state == S1 && e2_1 == 0) begin//��������openlist
        openlist_parenty[0] <= START_ROW;
        openlist_parentx[0] <= START_COLUMN;
        openlist_childy[0] <= START_ROW;
        openlist_childx[0] <= START_COLUMN;
        openlist_g[0] <= 0;
        openlist_h[0] <= ((END_ROW-START_ROW)+(END_COLUMN-START_COLUMN))<<3;
        openlist_f[0] <= (((END_ROW-START_ROW)+(END_COLUMN-START_COLUMN))<<3);
        open_row<=open_row+1;
        e2_1<=1;
    end
    else if (current_state == S1 && e2 && e2_1) begin
        e2_1<=0;
    end
    //״̬4��Ĥ19,�ж��ӽڵ��Ƿ���openlist��
    else if (current_state == S4 && open_row1!=open_row && e19==0) begin
            if (child_flag1 && child_node1[0] == openlist_childy[open_row1] && child_node1[1] == openlist_childx[open_row1])begin//�ӽڵ���openlist��
                if (openlist_g[parent_node[2]]+11<openlist_g[open_row1]) begin//�ж��Ƿ�Ҫ����ԭ���ĵ�ĸ��ڵ㡢g��f
                    openlist_parenty[open_row1] <= parent_node[0];
                    openlist_parentx[open_row1] <= parent_node[1];
                    openlist_g[open_row1] <= openlist_g[parent_node[2]]+11;
                    openlist_f[open_row1] <= openlist_g[parent_node[2]]+11+openlist_h[open_row1];
                    open_row1<=open_row1+1;
                    inopen_flag1 <= 1;
                end
                else begin 
                    inopen_flag1 <= 1;
                    open_row1<=open_row1+1;
                end
            end
            else if (child_flag2 && child_node2[0] == openlist_childy[open_row1] && child_node2[1] == openlist_childx[open_row1])begin//�ӽڵ���openlist��
                if (openlist_g[parent_node[2]]+8<openlist_g[open_row1]) begin//�µ�g��ԭ����gС�������ԭ���ĵ�ĸ��ڵ��g,f
                    openlist_parenty[open_row1] <= parent_node[0];
                    openlist_parentx[open_row1] <= parent_node[1];
                    openlist_g[open_row1] <= openlist_g[parent_node[2]]+8;
                    openlist_f[open_row1] <= openlist_g[parent_node[2]]+8+openlist_h[open_row1];
                    open_row1<=open_row1+1;
                    inopen_flag2 <= 1;
                end
                else begin
                    open_row1<=open_row1+1;
                    inopen_flag2 <= 1;
                end
            end
            else if (child_flag3 && child_node3[0] == openlist_childy[open_row1] && child_node3[1] == openlist_childx[open_row1])begin//�ӽڵ���openlist��
                if (openlist_g[parent_node[2]]+11<openlist_g[open_row1]) begin//�µ�g��ԭ����gС�������ԭ���ĵ�ĸ��ڵ��g,f
                    openlist_parenty[open_row1] <= parent_node[0];
                    openlist_parentx[open_row1] <= parent_node[1];
                    openlist_g[open_row1] <= openlist_g[parent_node[2]]+11;
                    openlist_f[open_row1] <= openlist_g[parent_node[2]]+11+openlist_h[open_row1];
                    open_row1<=open_row1+1;
                    inopen_flag3 <= 1;
                end
                else begin
                    open_row1<=open_row1+1;
                    inopen_flag3 <= 1;
                end
            end
            else if (child_flag4 && child_node4[0] == openlist_childy[open_row1] && child_node4[1] == openlist_childx[open_row1])begin//�ӽڵ���openlist��
                if (openlist_g[parent_node[2]]+8<openlist_g[open_row1]) begin//�µ�g��ԭ����gС�������ԭ���ĵ�ĸ��ڵ��g,f
                    openlist_parenty[open_row1] <= parent_node[0];
                    openlist_parentx[open_row1] <= parent_node[1];
                    openlist_g[open_row1] <= openlist_g[parent_node[2]]+8;
                    openlist_f[open_row1] <= openlist_g[parent_node[2]]+8+openlist_h[open_row1];
                    open_row1<=open_row1+1;
                    inopen_flag4 <= 1;
                end
                else begin
                    open_row1<=open_row1+1;
                    inopen_flag4 <= 1;
                end
            end
            else if (child_flag5 && child_node5[0] == openlist_childy[open_row1] && child_node5[1] == openlist_childx[open_row1])begin//�ӽڵ���openlist��
                if (openlist_g[parent_node[2]]+8<openlist_g[open_row1]) begin//�µ�g��ԭ����gС�������ԭ���ĵ�ĸ��ڵ��g,f
                    openlist_parenty[open_row1] <= parent_node[0];
                    openlist_parentx[open_row1] <= parent_node[1];
                    openlist_g[open_row1] <= openlist_g[parent_node[2]]+8;
                    openlist_f[open_row1] <= openlist_g[parent_node[2]]+8+openlist_h[open_row1];
                    open_row1<=open_row1+1;
                    inopen_flag5 <= 1;
                end
                else begin
                    open_row1<=open_row1+1;
                    inopen_flag5 <= 1;
                end
            end
            else if (child_flag6 && child_node6[0] == openlist_childy[open_row1] && child_node6[1] == openlist_childx[open_row1])begin//�ӽڵ���openlist��
                if (openlist_g[parent_node[2]]+11<openlist_g[open_row1]) begin//�µ�g��ԭ����gС�������ԭ���ĵ�ĸ��ڵ��g,f
                    openlist_parenty[open_row1] <= parent_node[0];
                    openlist_parentx[open_row1] <= parent_node[1];
                    openlist_g[open_row1] <= openlist_g[parent_node[2]]+11;
                    openlist_f[open_row1] <= openlist_g[parent_node[2]]+11+openlist_h[open_row1];
                    open_row1<=open_row1+1;
                    inopen_flag6 <= 1;
                end
                else begin
                    open_row1<=open_row1+1;
                    inopen_flag6 <= 1;
                end
            end
            else if (child_flag7 && child_node7[0] == openlist_childy[open_row1] && child_node7[1] == openlist_childx[open_row1])begin//�ӽڵ���openlist��
                if (openlist_g[parent_node[2]]+8<openlist_g[open_row1]) begin//�µ�g��ԭ����gС�������ԭ���ĵ�ĸ��ڵ��g,f
                    openlist_parenty[open_row1] <= parent_node[0];
                    openlist_parentx[open_row1] <= parent_node[1];
                    openlist_g[open_row1] <= openlist_g[parent_node[2]]+8;
                    openlist_f[open_row1] <= openlist_g[parent_node[2]]+8+openlist_h[open_row1];
                    open_row1<=open_row1+1;
                    inopen_flag7 <= 1;
                end
                else begin
                    open_row1<=open_row1+1;
                    inopen_flag7 <= 1;
                end
            end
            else if (child_flag8 && child_node8[0] == openlist_childy[open_row1] && child_node8[1] == openlist_childx[open_row1])begin//�ӽڵ���openlist��
                if (openlist_g[parent_node[2]]+11<openlist_g[open_row1]) begin//�µ�g��ԭ����gС�������ԭ���ĵ�ĸ��ڵ��g,f
                    openlist_parenty[open_row1] <= parent_node[0];
                    openlist_parentx[open_row1] <= parent_node[1];
                    openlist_g[open_row1] <= openlist_g[parent_node[2]]+11;
                    openlist_f[open_row1] <= openlist_g[parent_node[2]]+11+openlist_h[open_row1];
                    open_row1<=open_row1+1;
                    inopen_flag8 <= 1;
                end
                else begin
                    open_row1<=open_row1+1;
                    inopen_flag8 <= 1;
                end
            end
            else open_row1<=open_row1+1;
    end
    else if (current_state == S4 && open_row1==open_row && e19==0) begin
        open_row1<=0;
        e19<=1;
    end
    else if (current_state == S4 && e19) begin
        e19<=0;
    end
    //״̬5��Ĥ20������ӽڵ�1��openlist
    else if (current_state == S5 && e20==0) begin
        if (child_flag1 && inopen_flag1 == 0) begin//�ӽڵ㲻��openlist��
            openlist_parenty[open_row] <= parent_node[0];
            openlist_parentx[open_row] <= parent_node[1];
            openlist_childy[open_row] <= child_node1[0];
            openlist_childx[open_row] <= child_node1[1];
            openlist_g[open_row] <= openlist_g[parent_node[2]]+11;
            openlist_h[open_row] <= ((END_ROW+END_COLUMN)-(child_node1[0]+child_node1[1]))<<3;
            openlist_f[open_row] <= openlist_g[parent_node[2]]+11+(((END_ROW+END_COLUMN)-(child_node1[0]+child_node1[1]))<<3);
            open_row<=open_row+1;
            e20<=1;
        end
        else begin
            inopen_flag1 <= 0;
            e20<=1;
        end
    end
    else if (current_state == S5 && e20) begin
        e20<=0;
    end
    //״̬6��Ĥ21������ӽڵ�2��openlist
    else if (current_state == S6 && e21==0) begin
        if (child_flag2 && inopen_flag2 == 0) begin//�ӽڵ㲻��openlist��
            openlist_parenty[open_row] <= parent_node[0];
            openlist_parentx[open_row] <= parent_node[1];
            openlist_childy[open_row] <= child_node2[0];
            openlist_childx[open_row] <= child_node2[1];
            openlist_g[open_row] <= openlist_g[parent_node[2]]+8;
            openlist_h[open_row] <= ((END_ROW+END_COLUMN)-(child_node2[0]+child_node2[1]))<<3;
            openlist_f[open_row] <= openlist_g[parent_node[2]]+8+(((END_ROW+END_COLUMN)-(child_node2[0]+child_node2[1]))<<3);
            open_row<=open_row+1;
            e21<=1;
        end
        else begin
            inopen_flag2 <= 0;
            e21<=1;
        end
    end
    else if (current_state == S6 && e21) begin
        e21<=0;
    end
    //״̬7��Ĥ22������ӽڵ�3��openlist
    else if (current_state == S7 && e22==0) begin
         if (child_flag3 && inopen_flag3 == 0) begin//�ӽڵ㲻��openlist��
            openlist_parenty[open_row] <= parent_node[0];
            openlist_parentx[open_row] <= parent_node[1];
            openlist_childy[open_row] <= child_node3[0];
            openlist_childx[open_row] <= child_node3[1];
            openlist_g[open_row] <= openlist_g[parent_node[2]]+11;
            openlist_h[open_row] <= ((END_ROW+END_COLUMN)-(child_node3[0]+child_node3[1]))<<3;
            openlist_f[open_row] <= openlist_g[parent_node[2]]+11+(((END_ROW+END_COLUMN)-(child_node3[0]+child_node3[1]))<<3);
            open_row<=open_row+1;
            e22<=1;
        end
        else begin
            inopen_flag3 <= 0;
            e22<=1;
        end
    end
    else if (current_state == S7 && e22) begin
        e22<=0;
    end
    //״̬8��Ĥ23������ӽڵ�4��openlist
    else if (current_state == S8 && e23==0) begin
          if (child_flag4 && inopen_flag4 == 0) begin//�ӽڵ㲻��openlist��
            openlist_parenty[open_row] <= parent_node[0];
            openlist_parentx[open_row] <= parent_node[1];
            openlist_childy[open_row] <= child_node4[0];
            openlist_childx[open_row] <= child_node4[1];
            openlist_g[open_row] <= openlist_g[parent_node[2]]+8;
            openlist_h[open_row] <= ((END_ROW+END_COLUMN)-(child_node4[0]+child_node4[1]))<<3;
            openlist_f[open_row] <= openlist_g[parent_node[2]]+8+(((END_ROW+END_COLUMN)-(child_node4[0]+child_node4[1]))<<3);
            open_row<=open_row+1;
            e23<=1;
        end
        else begin
            inopen_flag4 <= 0;
            e23<=1;
        end
    end
    else if (current_state == S8 && e23) begin
        e23<=0;
    end
    //״̬9��Ĥ24������ӽڵ�5��openlist
    else if (current_state == S9 && e24==0) begin
        if (child_flag5 && inopen_flag5 == 0) begin//�ӽڵ㲻��openlist��
            openlist_parenty[open_row] <= parent_node[0];
            openlist_parentx[open_row] <= parent_node[1];
            openlist_childy[open_row] <= child_node5[0];
            openlist_childx[open_row] <= child_node5[1];
            openlist_g[open_row] <= openlist_g[parent_node[2]]+8;
            openlist_h[open_row] <= ((END_ROW+END_COLUMN)-(child_node5[0]+child_node5[1]))<<3;
            openlist_f[open_row] <= openlist_g[parent_node[2]]+8+(((END_ROW+END_COLUMN)-(child_node5[0]+child_node5[1]))<<3);
            open_row<=open_row+1;
            e24<=1;
        end
        else begin
            inopen_flag5 <= 0;
            e24<=1;
        end
    end
    else if (current_state == S9 && e24) begin
        e24<=0;
    end
    //״̬10��Ĥ25������ӽڵ�6��openlist
    else if (current_state == S10 && e25==0) begin
        if (child_flag6 && inopen_flag6 == 0) begin//�ӽڵ㲻��openlist��
            openlist_parenty[open_row] <= parent_node[0];
            openlist_parentx[open_row] <= parent_node[1];
            openlist_childy[open_row] <= child_node6[0];
            openlist_childx[open_row] <= child_node6[1];
            openlist_g[open_row] <= openlist_g[parent_node[2]]+11;
            openlist_h[open_row] <= ((END_ROW+END_COLUMN)-(child_node6[0]+child_node6[1]))<<3;
            openlist_f[open_row] <= openlist_g[parent_node[2]]+11+(((END_ROW+END_COLUMN)-(child_node6[0]+child_node6[1]))<<3);
            open_row<=open_row+1;
            e25<=1;
        end
        else begin
            inopen_flag6 <= 0;
            e25<=1;
        end
    end
    else if (current_state == S10 && e25) begin
        e25<=0;
    end
    //״̬11��Ĥ26������ӽڵ�7��openlist
    else if (current_state == S11 && e26==0) begin
        if (child_flag7 && inopen_flag7 == 0) begin//�ӽڵ㲻��openlist��
            openlist_parenty[open_row] <= parent_node[0];
            openlist_parentx[open_row] <= parent_node[1];
            openlist_childy[open_row] <= child_node7[0];
            openlist_childx[open_row] <= child_node7[1];
            openlist_g[open_row] <= openlist_g[parent_node[2]]+8;
            openlist_h[open_row] <= ((END_ROW+END_COLUMN)-(child_node7[0]+child_node7[1]))<<3;
            openlist_f[open_row] <= openlist_g[parent_node[2]]+8+(((END_ROW+END_COLUMN)-(child_node7[0]+child_node7[1]))<<3);
            open_row<=open_row+1;
            e26<=1;
        end
        else begin
            inopen_flag7 <= 0;
            e26<=1;
        end
    end
    else if (current_state == S11 && e26) begin
        e26<=0;
    end
    //״̬12��Ĥ27������ӽڵ�8��openlist
    else if (current_state == S12 && e27==0) begin
        if (child_flag8 && inopen_flag8 == 0) begin//�ӽڵ㲻��openlist��
            openlist_parenty[open_row] <= parent_node[0];
            openlist_parentx[open_row] <= parent_node[1];
            openlist_childy[open_row] <= child_node8[0];
            openlist_childx[open_row] <= child_node8[1];
            openlist_g[open_row] <= openlist_g[parent_node[2]]+11;
            openlist_h[open_row] <= ((END_ROW+END_COLUMN)-(child_node8[0]+child_node8[1]))<<3;
            openlist_f[open_row] <= openlist_g[parent_node[2]]+11+(((END_ROW+END_COLUMN)-(child_node8[0]+child_node8[1]))<<3);
            open_row<=open_row+1;
            e27<=1;
        end
        else begin
            inopen_flag8 <= 0;
            e27<=1;
        end
    end
    else if (current_state == S12 && e27) begin
        e27<=0;
    end
    //״̬14��Ĥ29�������ڵ��openlist��ɾ��
    else if (current_state == S14 && open_row2 == 0 && e29==0 && parent_node[2] != 0) begin
        open_row2 <= parent_node[2];
    end
    else if (current_state == S14 && open_row2 != open_row-1 && e29==0) begin
        open_row2<=open_row2+1;
        openlist_parenty[open_row2]<=openlist_parenty[open_row2+1];
        openlist_parentx[open_row2]<=openlist_parentx[open_row2+1];
        openlist_childy[open_row2]<=openlist_childy[open_row2+1];
        openlist_childx[open_row2]<=openlist_childx[open_row2+1];
        openlist_g[open_row2]<=openlist_g[open_row2+1];
        openlist_h[open_row2]<=openlist_h[open_row2+1];
        openlist_f[open_row2]<=openlist_f[open_row2+1];
    end
    else if (current_state == S14 && open_row2 == open_row-1 && e29==0) begin
        open_row2<=0;
        open_row<=open_row-1;
        e29<=1;
    end
    else if (current_state == S14 && e29) begin
        e29<=0;
    end
    
end

//�жϽ���
always @(posedge clk) begin
    if (!rst_n) begin
        e31<=0;
        findpath_flag<=0;
    end
    //״̬16��Ĥ31���ж��㷨�Ƿ����
    else if (current_state == S16 && e31==0) begin
        if (parent_node[0] == END_ROW && parent_node[1] == END_COLUMN && open_row != 0) begin//���ڵ�Ϊ�յ����h����0
            e31<=2'd3;//�㷨���������ҵ����·������һ��״̬ΪS17
            findpath_flag<=1;
        end
        else if (open_row == 0) begin//openlistΪ�գ��㷨������û���ҵ�·��,��һ��״̬ΪS20
            e31<=2'd2;
            findpath_flag<=0;
        end
        else begin//�㷨û�н�������һ��״̬ΪS2
            e31<=2'd1;
        end
    end
    else if (current_state == S16 && e31!=0) begin
        e31<=0;
    end
end

//�����ҵ������·��
always @(posedge clk) begin
    if (!rst_n) begin
        close_row9<=0;
        open_row4<=0;
        path_row<=END_ROW;
        path_column<=END_COLUMN;
        e32<=0;
        e35_3 <= 0;
    end
    //״̬20��Ĥ35
    else if (current_state == S20 && e35_3==0) begin
        path_row<=END_ROW;
        path_column<=END_COLUMN;
        e35_3 <= 1;
    end
    else if (current_state == S20 && e35_1 && e35_2 && e35_3) begin
        e35_3 <= 0;
    end
    //״̬17��Ĥ32,��������·��
    else if (current_state == S17 && open_row4!=open_row && e32==0) begin
        if (openlist_childy[open_row4] == path_row && openlist_childx[open_row4] == path_column)begin//��openlist���ҵ�
            path_row <= openlist_parenty[open_row4];
            path_column <= openlist_parentx[open_row4];
            e32<=1;
            open_row4<=0;
        end
        else open_row4<=open_row4+1;
    end
    else if (current_state == S17 && open_row4==open_row && close_row9!=close_row && e32==0) begin
        if (closelist_childy[close_row9] == path_row && closelist_childx[close_row9] == path_column)begin
            path_row <= closelist_parenty[close_row9];
            path_column <= closelist_parentx[close_row9];
            e32<=1;
            open_row4<=0;
            close_row9<=0;
        end
        else close_row9<=close_row9+1;
    end
    else if (current_state == S17 && e32) begin
        e32<=0;
    end
end

//���ڷ���
always @(posedge clk) begin
    if(!rst_n) begin
        tx_data <= 0;
        tx_flag <= 0;
        tx_cnt <= 0;
        e33<=0;
    end
    //״̬18��Ĥ33
    else if (current_state == S18 && tx_done && e33==0) begin
        case (tx_cnt)
            0:begin
                tx_data <= path_row;
                tx_flag<=1;
                tx_cnt <= 2'd1;
                e33<=0;
            end
            1:begin
                tx_data <= 0;
                tx_flag <= 0;
                tx_cnt <= 2'd2;
                e33<=0;
            end
            2:begin
                tx_data <= path_column;
                tx_flag <= 1'b1;
                tx_cnt <= 2'd3;
                e33<=0;
            end
            3:begin
                tx_data <= 0;
                tx_flag <= 0;
                tx_cnt <= 0;
                e33<=1;
            end
        endcase
    end
    else if (current_state == S18 && e33) begin
        e33<=0;
    end
end

//�ж�·�������Ƿ�����
always @(posedge clk) begin
    if (!rst_n) begin
        e34<=0;
    end
    //״̬19��Ĥ34
    else if (current_state == S19 && path_row==START_ROW && path_column==START_COLUMN && e34==0) begin//���ݵ���㣬·�������꣬��ת��S20
        e34<=2;
    end
    else if (current_state == S19 && (path_row!=START_ROW || path_column!=START_COLUMN) && e34==0) begin//û�л��ݵ���㣬·������δ��ɣ���ת��S17
        e34<=1;
    end
    else if (current_state == S19 && e34!=0) begin
        e34<=0;
    end
end


//����ILAģ��
ila_0 ila_0_i(
    .clk(clk),
    .probe0(current_state),//5
    .probe1(led0),//1
    .probe2(findpath_flag),//1
    .probe3(led1),//1
    .probe4(e1),//1
    .probe5(e31)//2
);


endmodule
