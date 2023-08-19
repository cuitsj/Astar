module astar_gnps#(
    parameter MAP_ROW = 8'd16,
    parameter MAP_COLUMN = 8'd16,
    parameter MAP_EXP = 8'd4,//2的6次方为64
    parameter START_ROW = 8'd2,
    parameter START_COLUMN = 8'd2,
    parameter END_ROW = 8'd14,
    parameter END_COLUMN = 8'd14,
    parameter OPEN_ROW = 12'd300,
    parameter CLOSE_ROW = 12'd300
)(
	input                          clk,            
	input                          rst_n,         
	input                          rxprocess_en,//串口收到数据
	input [7:0]                   rx_data,    //串口数据
	input                          tx_done,  //串口发送完成
	
	output reg [7:0]          tx_data,
	output reg                 tx_flag,   // 使能串口发送
	output reg                 led0,
    output reg                 led1
);

//Astar-gnps状态
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

//Astar-gnps酶变量
reg e1,e2,e3,e4,e5,e6,e7,e8,e9,e10,e11,e12,e13,e14,e15,e16,e17,e18,e19,e20,e21,e22,e23,e24,e25,e26,e27,e28,e29,e30,e32,e33,e35;
reg e2_1;
reg [1:0] e31,e34;

//状态0，串口接收数据处理
reg rxprocess_flag;//开始处理标志
reg [7:0] rx_buff;//串口数据缓存
reg [7:0] map_row,map_column;//处理的数据个数计数
//reg map[MAP_ROW-1:0][MAP_COLUMN-1:0];
reg map[(MAP_ROW<<MAP_EXP)-1:0];

//状态1,13,15
reg [11:0]  parent_node[2:0];
reg [11:0] close_row;//closelist的行数
reg [11:0] min_f;
reg [11:0] open_row3;//用于从openlist中找新的父节点计数行数
//closelist
reg [7:0] closelist_parenty[CLOSE_ROW-1:0];//父节点y
reg [7:0] closelist_parentx[CLOSE_ROW-1:0];//父节点x
reg [7:0] closelist_childy[CLOSE_ROW-1:0];//父节点y
reg [7:0] closelist_childx[CLOSE_ROW-1:0];//父节点x

//状态2
reg obs_flag1,obs_flag2,obs_flag3,obs_flag4,obs_flag5,obs_flag6,obs_flag7,obs_flag8;

//状态3
reg child_flag1,child_flag2,child_flag3,child_flag4,child_flag5,child_flag6,child_flag7,child_flag8;
reg [7:0] child_node1[1:0],child_node2[1:0],child_node3[1:0],child_node4[1:0],child_node5[1:0],child_node6[1:0],child_node7[1:0],child_node8[1:0];
reg [11:0] close_row1,close_row2,close_row3,close_row4,close_row5,close_row6,close_row7,close_row8;

//状态4,5,6,7,8,9,10,11,12,14
reg [11:0] open_row;//openlist的行数
reg [11:0] open_row1;
reg [11:0] open_row2;//用于从openlist中删除父节点计数行数
reg inopen_flag1,inopen_flag2,inopen_flag3,inopen_flag4,inopen_flag5,inopen_flag6,inopen_flag7,inopen_flag8;
//openlist
reg [7:0] openlist_parenty[OPEN_ROW-1:0];//父节点y
reg [7:0] openlist_parentx[OPEN_ROW-1:0];//父节点x
reg [7:0] openlist_childy[OPEN_ROW-1:0];//父节点y
reg [7:0] openlist_childx[OPEN_ROW-1:0];//父节点x
reg [7:0] openlist_g[OPEN_ROW-1:0];//g
reg [7:0] openlist_h[OPEN_ROW-1:0];//h
reg [7:0] openlist_f[OPEN_ROW-1:0];//f

//状态16
reg findpath_flag;

//状态17
reg [11:0] open_row4;//用于回溯最短路径计数行数
reg [11:0] close_row9;//用于回溯最短路径计数
reg [11:0] path_row,path_column;

//状态18
reg [1:0] tx_cnt;





//算法状态跳转
always @(posedge clk) begin
    if (!rst_n) begin
        current_state <= S20;//初始状态为S20
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
                if (e31 == 1) current_state <= S2;//规划未结束
                else if (e31 == 2) current_state <= S17;//规划结束，并且找到最短路径
                else if (e31 == 3) current_state <= S20;//算法结束，没有找到最短路径
                else current_state <= S16;//不满足跳转条件则不跳转
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
                if (e35) current_state <= S0;
                else current_state <= S20;
            end
            default:current_state <= S20;
        endcase
    end
end

//状态0接收地图
//打一拍,缓存串口接收到的数据
always @(posedge clk) begin
    if (!rst_n) begin
        rxprocess_flag <= 0;
        rx_buff <= 0;
    end
    else if (current_state == S0 && rxprocess_en) begin//状态0，且串口收到数据
        rxprocess_flag <= 1;
        rx_buff <= rx_data;
    end
    else begin//还没开始或者在处理的过程中
        rxprocess_flag <= 0;
        rx_buff <= 0;
    end
end

//数据计数
always @(posedge clk) begin
    if(!rst_n) begin
        map_row<= 0;
        map_column<= 0;
    end
    else if (current_state == S0 && rxprocess_flag) begin
        if (map_column == MAP_COLUMN-8) begin
            if (map_row == MAP_ROW-1) begin
                map_column<=0;
                map_row <= 0;
            end
            else begin
                map_column<=0;
                map_row<=map_row+8'd1;
            end
        end
        else map_column<=map_column+8'd8;
    end
    else begin
        map_row<=map_row;
        map_column<=map_column;
    end
end

//数据存储
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

//接收完毕判断
always @(posedge clk) begin
    if(!rst_n) begin
        e1 <= 0;
        led0<=1;
    end
    else if (current_state == S0 && map_row == MAP_ROW-1 && map_column == MAP_COLUMN-8) begin
        e1 <= 1;
        led0<=0;
    end
    else begin
        e1 <= 0;
        led0<=led0;
    end
end

//LED1指示是否搜索到路径
always @(posedge clk) begin
    if(!rst_n) begin
        led1<=0;
    end
    else if (current_state == S0 && findpath_flag) begin
        led1<=1;
    end
    else begin
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
    end
    
    //状态1,膜2
    else if (current_state == S1 && e2 == 0) begin//将起点设置为父节点
        parent_node[0]<=START_ROW;
        parent_node[1]<=START_COLUMN;
        parent_node[2]<=8'd0;
        e2<=1;
    end
    else if (current_state == S1 && e2 && e2_1) begin
        e2<=0;
    end
    
    //状态13，膜28，将父节点添加到closelist中
    else if (current_state == S13 && e28==0) begin
        closelist_parenty[close_row] <= openlist_parenty[parent_node[2]];
        closelist_parentx[close_row] <= openlist_parentx[parent_node[2]];
        closelist_childy[close_row] <= openlist_childy[parent_node[2]];
        closelist_childx[close_row] <= openlist_childx[parent_node[2]];
        close_row <= close_row+1;//closelist的行数
        e28<=1;
    end
    else if (current_state == S13 && e28) begin
        e28<=0;
    end
    
    //状态15，膜30，在openlist中找新的父节点
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

//判断子节点1是否为障碍物
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag1 <= 0;
        e3<=0;
    end
    //状态2，膜3
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

//判断子节点2是否为障碍物
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag2 <= 0;
        e4<=0;
    end
    //状态2，膜4
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

//判断子节点3是否为障碍物
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag3 <= 0;
        e5<=0;
    end
    //状态2，膜5
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

//判断子节点4是否为障碍物
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag4 <= 0;
        e6<=0;
    end
    //状态2，膜6
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

//判断子节点5是否为障碍物
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag5 <= 0;
        e7<=0;
    end
    //状态2，膜7
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

//判断子节点6是否为障碍物
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag6 <= 0;
        e8<=0;
    end
    //状态2，膜8
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

//判断子节点7是否为障碍物
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag7 <= 0;
        e9<=0;
    end
    //状态2，膜9
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

//判断子节点8是否为障碍物
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag8 <= 0;
        e10<=0;
    end
    //状态2，膜10
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

//判断子节点1是否在closelist中
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag1 <= 0;
        child_node1[0]<=0;
        child_node1[1]<=0;
        close_row1 <= 0;
        e11<=0;
    end
    //状态3，膜11
    else if (current_state == S3 && e11==0 && obs_flag1==0 && close_row1!=close_row) begin
        if (parent_node[0]+1 == closelist_childy[close_row1] && parent_node[1]-1 == closelist_childx[close_row1]) begin//在closelist中
            close_row1<=0;
            child_flag1 <= 0;
            child_node1[0]<=0;
            child_node1[1]<=0;
            e11<=1;
        end
        else close_row1 <= close_row1 + 1;
    end
    else if (current_state == S3 && e11==0 && obs_flag1==0 && close_row1==close_row) begin//不在closelist中
            close_row1<=0;
            child_flag1 <= 1;
            child_node1[0]<=parent_node[0]+1;
            child_node1[1]<=parent_node[1]-1;
            e11<=1;
    end
    else if (current_state == S3 && e11==0 && obs_flag1) begin//子节点是障碍物
        child_flag1 <= 0;
        child_node1[0]<=0;
        child_node1[1]<=0;
        e11<=1;
    end
    else if (current_state == S3 && e11 && e12 && e13 && e14 && e15 && e16 && e17 && e18)begin
        e11<=0;
    end
//    else begin
//        child_flag1 <= 0;
//        child_node1[0]<=0;
//        child_node1[1]<=0;
//    end
end

//判断子节点2是否在closelist中
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag2 <= 0;
        child_node2[0]<=0;
        child_node2[1]<=0;
        close_row2 <= 0;
        e12<=0;
    end
    //状态3，膜12
    else if (current_state == S3 && e12==0 && obs_flag2==0 && close_row2!=close_row) begin
        if (parent_node[0]+1 == closelist_childy[close_row2] && parent_node[1] == closelist_childx[close_row2]) begin//在closelist中
            close_row2<=0;
            child_flag2 <= 0;
            child_node2[0]<=0;
            child_node2[1]<=0;
            e12<=1;
        end
        else close_row2 <= close_row2 + 1;
    end
    else if (current_state == S3 && e12==0 && obs_flag2==0 && close_row2==close_row) begin//不在closelist中
            close_row2<=0;
            child_flag2 <= 1;
            child_node2[0]<=parent_node[0]+1;
            child_node2[1]<=parent_node[1];
            e12<=1;
    end
    else if (current_state == S3 && e12==0 && obs_flag2) begin//子节点是障碍物
        child_flag2 <= 0;
        child_node2[0]<=0;
        child_node2[1]<=0;
        e12<=1;
    end
    else if (current_state == S3 && e11 && e12 && e13 && e14 && e15 && e16 && e17 && e18)begin
        e12<=0;
    end
//    else begin
//        child_flag2 <= 0;
//        child_node2[0]<=0;
//        child_node2[1]<=0;
//    end
end

//判断子节点3是否在closelist中
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag3 <= 0;
        child_node3[0]<=0;
        child_node3[1]<=0;
        close_row3 <= 0;
        e13<=0;
    end
    //状态3，膜13
    else if (current_state == S3 && e13==0 && obs_flag3==0 && close_row3!=close_row) begin
        if (parent_node[0]+1 == closelist_childy[close_row3] && parent_node[1]+1 == closelist_childx[close_row3]) begin//在closelist中
            close_row3<=0;
            child_flag3 <= 0;
            child_node3[0]<=0;
            child_node3[1]<=0;
            e13<=1;
        end
        else close_row3 <= close_row3 + 1;
    end
    else if (current_state == S3 && e13==0 && obs_flag3==0 && close_row3==close_row) begin//不在closelist中
            close_row3<=0;
            child_flag3 <= 1;
            child_node3[0]<=parent_node[0]+1;
            child_node3[1]<=parent_node[1]+1;
            e13<=1;
    end
    else if (current_state == S3 && e13==0 && obs_flag3) begin//子节点是障碍物
        child_flag3 <= 0;
        child_node3[0]<=0;
        child_node3[1]<=0;
        e13<=1;
    end
    else if (current_state == S3 && e11 && e12 && e13 && e14 && e15 && e16 && e17 && e18)begin
        e13<=0;
    end
//    else begin
//        child_flag3 <= 0;
//        child_node3[0]<=0;
//        child_node3[1]<=0;
//    end
end

//判断子节点4是否在closelist中
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag4 <= 0;
        child_node4[0]<=0;
        child_node4[1]<=0;
        close_row4 <= 0;
        e14<=0;
    end
    //状态3，膜14
    else if (current_state == S3 && e14==0 && obs_flag4==0 && close_row4!=close_row) begin
        if (parent_node[0] == closelist_childy[close_row4] && parent_node[1]-1 == closelist_childx[close_row4]) begin//在closelist中
            close_row4<=0;
            child_flag4 <= 0;
            child_node4[0]<=0;
            child_node4[1]<=0;
            e14<=1;
        end
        else close_row4 <= close_row4 + 1;
    end
    else if (current_state == S3 && e14==0 && obs_flag4==0 && close_row4==close_row) begin//不在closelist中
            close_row4<=0;
            child_flag4 <= 1;
            child_node4[0]<=parent_node[0];
            child_node4[1]<=parent_node[1]-1;
            e14<=1;
    end
    else if (current_state == S3 && e14==0 && obs_flag4) begin//子节点是障碍物
        child_flag4 <= 0;
        child_node4[0]<=0;
        child_node4[1]<=0;
        e14<=1;
    end
    else if (current_state == S3 && e11 && e12 && e13 && e14 && e15 && e16 && e17 && e18)begin
        e14<=0;
    end
//    else begin
//        child_flag4 <= 0;
//        child_node4[0]<=0;
//        child_node4[1]<=0;
//    end
end

//判断子节点5是否在closelist中
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag5 <= 0;
        child_node5[0]<=0;
        child_node5[1]<=0;
        close_row5 <= 0;
        e15<=0;
    end
    //状态3，膜15
    else if (current_state == S3 && e15==0 && obs_flag5==0 && close_row5!=close_row) begin
        if (parent_node[0] == closelist_childy[close_row5] && parent_node[1]+1 == closelist_childx[close_row5]) begin//在closelist中
            close_row5<=0;
            child_flag5 <= 0;
            child_node5[0]<=0;
            child_node5[1]<=0;
            e15<=1;
        end
        else close_row5 <= close_row5 + 1;
    end
    else if (current_state == S3 && e15==0 && obs_flag5==0 && close_row5==close_row) begin//不在closelist中
            close_row5<=0;
            child_flag5 <= 1;
            child_node5[0]<=parent_node[0];
            child_node5[1]<=parent_node[1]+1;
            e15<=1;
    end
    else if (current_state == S3 && e15==0 && obs_flag5) begin//子节点是障碍物
        child_flag5 <= 0;
        child_node5[0]<=0;
        child_node5[1]<=0;
        e15<=1;
    end
    else if (current_state == S3 && e11 && e12 && e13 && e14 && e15 && e16 && e17 && e18)begin
        e15<=0;
    end
//    else begin
//        child_flag5 <= 0;
//        child_node5[0]<=0;
//        child_node5[1]<=0;
//    end
end

//判断子节点6是否在closelist中
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag6 <= 0;
        child_node6[0]<=0;
        child_node6[1]<=0;
        close_row6 <= 0;
        e16<=0;
    end
    //状态3，膜16
    else if (current_state == S3 && e16==0 && obs_flag6==0 && close_row6!=close_row) begin
        if (parent_node[0]-1 == closelist_childy[close_row6] && parent_node[1]-1 == closelist_childx[close_row6]) begin//在closelist中
            close_row6<=0;
            child_flag6 <= 0;
            child_node6[0]<=0;
            child_node6[1]<=0;
            e16<=1;
        end
        else close_row6 <= close_row6 + 1;
    end
    else if (current_state == S3 && e16==0 && obs_flag6==0 && close_row6==close_row) begin//不在closelist中
            close_row6<=0;
            child_flag6 <= 1;
            child_node6[0]<=parent_node[0]-1;
            child_node6[1]<=parent_node[1]-1;
            e16<=1;
    end
    else if (current_state == S3 && e16==0 && obs_flag6) begin//子节点是障碍物
        child_flag6 <= 0;
        child_node6[0]<=0;
        child_node6[1]<=0;
        e16<=1;
    end
    else if (current_state == S3 && e11 && e12 && e13 && e14 && e15 && e16 && e17 && e18)begin
        e16<=0;
    end
//    else begin
//        child_flag6 <= 0;
//        child_node6[0]<=0;
//        child_node6[1]<=0;
//    end
end

//判断子节点7是否在closelist中
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag7 <= 0;
        child_node7[0]<=0;
        child_node7[1]<=0;
        close_row7 <= 0;
        e17<=0;
    end
    //状态3，膜17
    else if (current_state == S3 && e17==0 && obs_flag7==0 && close_row7!=close_row) begin
        if (parent_node[0]-1 == closelist_childy[close_row7] && parent_node[1] == closelist_childx[close_row7]) begin//在closelist中
            close_row7<=0;
            child_flag7 <= 0;
            child_node7[0]<=0;
            child_node7[1]<=0;
            e17<=1;
        end
        else close_row7 <= close_row7 + 1;
    end
    else if (current_state == S3 && e17==0 && obs_flag7==0 && close_row7==close_row) begin//不在closelist中
            close_row7<=0;
            child_flag7 <= 1;
            child_node7[0]<=parent_node[0]-1;
            child_node7[1]<=parent_node[1];
            e17<=1;
    end
    else if (current_state == S3 && e17==0 && obs_flag7) begin//子节点是障碍物
        child_flag7 <= 0;
        child_node7[0]<=0;
        child_node7[1]<=0;
        e17<=1;
    end
    else if (current_state == S3 && e11 && e12 && e13 && e14 && e15 && e16 && e17 && e18)begin
        e17<=0;
    end
//    else begin
//        child_flag7 <= 0;
//        child_node7[0]<=0;
//        child_node7[1]<=0;
//    end
end

//判断子节点8是否在closelist中
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag8 <= 0;
        child_node8[0]<=0;
        child_node8[1]<=0;
        close_row8 <= 0;
        e18<=0;
    end
    //状态3，膜18
    else if (current_state == S3 && e18==0 && obs_flag8==0 && close_row8!=close_row) begin
        if (parent_node[0]-1 == closelist_childy[close_row8] && parent_node[1]+1 == closelist_childx[close_row8]) begin//在closelist中
            close_row8<=0;
            child_flag8 <= 0;
            child_node8[0]<=0;
            child_node8[1]<=0;
            e18<=1;
        end
        else close_row8 <= close_row8 + 1;
    end
    else if (current_state == S3 && e18==0 && obs_flag8==0 && close_row8==close_row) begin//不在closelist中
            close_row8<=0;
            child_flag8 <= 1;
            child_node8[0]<=parent_node[0]-1;
            child_node8[1]<=parent_node[1]+1;
            e18<=1;
    end
    else if (current_state == S3 && e18==0 && obs_flag8) begin//子节点是障碍物
        child_flag8 <= 0;
        child_node8[0]<=0;
        child_node8[1]<=0;
        e18<=1;
    end
    else if (current_state == S3 && e11 && e12 && e13 && e14 && e15 && e16 && e17 && e18)begin
        e18<=0;
    end
//    else begin
//        child_flag8 <= 0;
//        child_node8[0]<=0;
//        child_node8[1]<=0;
//    end
end


//openlist
always @(posedge clk) begin:openlist
    integer i;
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
        for (i=0;i<OPEN_ROW;i=i+1)begin
            openlist_parenty[i] <= 0;
            openlist_parentx[i] <= 0;
            openlist_childy[i] <= 0;
            openlist_childx[i] <= 0;
            openlist_g[i] <= 0;
            openlist_h[i] <= 0;
            openlist_f[i] <= 0;
        end
    end
    //状态1,膜2
    else if (current_state == S1 && e2_1 == 0) begin//将起点放入openlist
        
        openlist_parenty[0] <= START_ROW;
        openlist_parentx[0] <= START_COLUMN;
        openlist_childy[0] <= START_ROW;
        openlist_childx[0] <= START_COLUMN;
        openlist_g[0] <= 0;
        openlist_h[0] <= ((END_ROW-child_node1[0])+(END_COLUMN-child_node1[1]))*8;
        openlist_f[0] <= (((END_ROW-child_node1[0])+(END_COLUMN-child_node1[1]))*8);
        open_row<=open_row+1;
        e2_1<=1;
    end
    else if (current_state == S1 && e2 && e2_1) begin
        e2_1<=0;
    end
    
    //状态4，膜19,判断子节点是否在openlist中
    else if (current_state == S4 && open_row1!=open_row && e19==0) begin
            if (child_flag1 && child_node1[0] == openlist_childy[open_row1] && child_node1[1] == openlist_childx[open_row1])begin//子节点在openlist中
                if (openlist_g[parent_node[2]]+11<openlist_g[open_row1]) begin//判断是否要更新原来的点的父节点、g和f
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
            else if (child_flag2 && child_node2[0] == openlist_childy[open_row1] && child_node2[1] == openlist_childx[open_row1])begin//子节点在openlist中
                if (openlist_g[parent_node[2]]+8<openlist_g[open_row1]) begin//新的g比原来的g小，则更新原来的点的父节点和g,f
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
            else if (child_flag3 && child_node3[0] == openlist_childy[open_row1] && child_node3[1] == openlist_childx[open_row1])begin//子节点在openlist中
                if (openlist_g[parent_node[2]]+11<openlist_g[open_row1]) begin//新的g比原来的g小，则更新原来的点的父节点和g,f
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
            else if (child_flag4 && child_node4[0] == openlist_childy[open_row1] && child_node4[1] == openlist_childx[open_row1])begin//子节点在openlist中
                if (openlist_g[parent_node[2]]+8<openlist_g[open_row1]) begin//新的g比原来的g小，则更新原来的点的父节点和g,f
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
            else if (child_flag5 && child_node5[0] == openlist_childy[open_row1] && child_node5[1] == openlist_childx[open_row1])begin//子节点在openlist中
                if (openlist_g[parent_node[2]]+8<openlist_g[open_row1]) begin//新的g比原来的g小，则更新原来的点的父节点和g,f
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
            else if (child_flag6 && child_node6[0] == openlist_childy[open_row1] && child_node6[1] == openlist_childx[open_row1])begin//子节点在openlist中
                if (openlist_g[parent_node[2]]+11<openlist_g[open_row1]) begin//新的g比原来的g小，则更新原来的点的父节点和g,f
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
            else if (child_flag7 && child_node7[0] == openlist_childy[open_row1] && child_node7[1] == openlist_childx[open_row1])begin//子节点在openlist中
                if (openlist_g[parent_node[2]]+8<openlist_g[open_row1]) begin//新的g比原来的g小，则更新原来的点的父节点和g,f
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
            else if (child_flag8 && child_node8[0] == openlist_childy[open_row1] && child_node8[1] == openlist_childx[open_row1])begin//子节点在openlist中
                if (openlist_g[parent_node[2]]+11<openlist_g[open_row1]) begin//新的g比原来的g小，则更新原来的点的父节点和g,f
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
    
    //状态5，膜20，添加子节点1到openlist
    else if (current_state == S5 && e20==0) begin
        if (inopen_flag1 == 0) begin//子节点不在openlist中
            openlist_parenty[open_row] <= parent_node[0];
            openlist_parentx[open_row] <= parent_node[1];
            openlist_childy[open_row] <= child_node1[0];
            openlist_childx[open_row] <= child_node1[1];
            openlist_g[open_row] <= openlist_g[parent_node[2]]+11;
            openlist_h[open_row] <= ((END_ROW-child_node1[0])+(END_COLUMN-child_node1[1]))*8;
            openlist_f[open_row] <= openlist_g[parent_node[2]]+11+(((END_ROW-child_node1[0])+(END_COLUMN-child_node1[1]))*8);
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
    //状态6，膜21，添加子节点2到openlist
    else if (current_state == S6 && e21==0) begin
        if (inopen_flag2 == 0) begin//子节点不在openlist中
            openlist_parenty[open_row] <= parent_node[0];
            openlist_parentx[open_row] <= parent_node[1];
            openlist_childy[open_row] <= child_node2[0];
            openlist_childx[open_row] <= child_node2[1];
            openlist_g[open_row] <= openlist_g[parent_node[2]]+8;
            openlist_h[open_row] <= ((END_ROW-child_node2[0])+(END_COLUMN-child_node2[1]))*8;
            openlist_f[open_row] <= openlist_g[parent_node[2]]+8+(((END_ROW-child_node2[0])+(END_COLUMN-child_node2[1]))*8);
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
    //状态7，膜22，添加子节点3到openlist
    else if (current_state == S7 && e22==0) begin
         if (inopen_flag3 == 0) begin//子节点不在openlist中
            openlist_parenty[open_row] <= parent_node[0];
            openlist_parentx[open_row] <= parent_node[1];
            openlist_childy[open_row] <= child_node3[0];
            openlist_childx[open_row] <= child_node3[1];
            openlist_g[open_row] <= openlist_g[parent_node[2]]+11;
            openlist_h[open_row] <= ((END_ROW-child_node3[0])+(END_COLUMN-child_node3[1]))*8;
            openlist_f[open_row] <= openlist_g[parent_node[2]]+11+(((END_ROW-child_node3[0])+(END_COLUMN-child_node3[1]))*8);
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
    //状态8，膜23，添加子节点4到openlist
    else if (current_state == S8 && e23==0) begin
          if (inopen_flag4 == 0) begin//子节点不在openlist中
            openlist_parenty[open_row] <= parent_node[0];
            openlist_parentx[open_row] <= parent_node[1];
            openlist_childy[open_row] <= child_node4[0];
            openlist_childx[open_row] <= child_node4[1];
            openlist_g[open_row] <= openlist_g[parent_node[2]]+8;
            openlist_h[open_row] <= ((END_ROW-child_node4[0])+(END_COLUMN-child_node4[1]))*8;
            openlist_f[open_row] <= openlist_g[parent_node[2]]+8+(((END_ROW-child_node4[0])+(END_COLUMN-child_node4[1]))*8);
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
    //状态9，膜24，添加子节点5到openlist
    else if (current_state == S9 && e24==0) begin
        if (inopen_flag5 == 0) begin//子节点不在openlist中
            openlist_parenty[open_row] <= parent_node[0];
            openlist_parentx[open_row] <= parent_node[1];
            openlist_childy[open_row] <= child_node5[0];
            openlist_childx[open_row] <= child_node5[1];
            openlist_g[open_row] <= openlist_g[parent_node[2]]+8;
            openlist_h[open_row] <= ((END_ROW-child_node5[0])+(END_COLUMN-child_node5[1]))*8;
            openlist_f[open_row] <= openlist_g[parent_node[2]]+8+(((END_ROW-child_node5[0])+(END_COLUMN-child_node5[1]))*8);
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
    //状态10，膜25，添加子节点6到openlist
    else if (current_state == S10 && e25==0) begin
        if (inopen_flag6 == 0) begin//子节点不在openlist中
            openlist_parenty[open_row] <= parent_node[0];
            openlist_parentx[open_row] <= parent_node[1];
            openlist_childy[open_row] <= child_node6[0];
            openlist_childx[open_row] <= child_node6[1];
            openlist_g[open_row] <= openlist_g[parent_node[2]]+11;
            openlist_h[open_row] <= ((END_ROW-child_node6[0])+(END_COLUMN-child_node6[1]))*8;
            openlist_f[open_row] <= openlist_g[parent_node[2]]+11+(((END_ROW-child_node6[0])+(END_COLUMN-child_node6[1]))*8);
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
    //状态11，膜26，添加子节点7到openlist
    else if (current_state == S11 && e26==0) begin
        if (inopen_flag7 == 0) begin//子节点不在openlist中
            openlist_parenty[open_row] <= parent_node[0];
            openlist_parentx[open_row] <= parent_node[1];
            openlist_childy[open_row] <= child_node7[0];
            openlist_childx[open_row] <= child_node7[1];
            openlist_g[open_row] <= openlist_g[parent_node[2]]+8;
            openlist_h[open_row] <= ((END_ROW-child_node7[0])+(END_COLUMN-child_node7[1]))*8;
            openlist_f[open_row] <= openlist_g[parent_node[2]]+8+(((END_ROW-child_node7[0])+(END_COLUMN-child_node7[1]))*8);
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
    //状态12，膜27，添加子节点8到openlist
    else if (current_state == S12 && e27==0) begin
        if (inopen_flag8 == 0) begin//子节点不在openlist中
            openlist_parenty[open_row] <= parent_node[0];
            openlist_parentx[open_row] <= parent_node[1];
            openlist_childy[open_row] <= child_node8[0];
            openlist_childx[open_row] <= child_node8[1];
            openlist_g[open_row] <= openlist_g[parent_node[2]]+11;
            openlist_h[open_row] <= ((END_ROW-child_node8[0])+(END_COLUMN-child_node8[1]))*8;
            openlist_f[open_row] <= openlist_g[parent_node[2]]+11+(((END_ROW-child_node8[0])+(END_COLUMN-child_node8[1]))*8);
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
    
    //状态14，膜29，将父节点从openlist中删除
    else if (current_state == S14 && open_row2 == 0 && e29==0 && parent_node[2] != 0) begin
        open_row2 <= parent_node[2];
    end
    else if (current_state == S14 && open_row2 != open_row-1 && e29==0) begin
        open_row2<=open_row2+1;
        openlist_parenty[open_row2]<=openlist_parenty[open_row2+1];
        openlist_parentx[open_row2]<=openlist_parentx[open_row2+1];
        openlist_childy[open_row2]<=openlist_childy[open_row2+1];
        openlist_childy[open_row2]<=openlist_childy[open_row2+1];
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

//判断结束
always @(posedge clk) begin
    if (!rst_n) begin
        e31<=0;
        findpath_flag<=0;
    end
    //状态16，膜31，判断算法是否结束
    else if (current_state == S16 && e31==0) begin
        if (parent_node[0] == END_ROW && parent_node[1] == END_COLUMN && open_row != 0) begin//父节点为终点或者h等于0
            e31<=2'd3;//算法结束并且找到最短路径，下一个状态为S17
            findpath_flag<=1;
        end
        else if (open_row == 0) begin//openlist为空，算法结束，没有找到路径,下一个状态为S20
            e31<=2'd2;
            findpath_flag<=0;
        end
        else begin//算法没有结束，下一个状态为S2
            e31<=2'd1;
        end
    end
    else if (current_state == S16 && e31!=0) begin
        e31<=0;
    end
end

//回溯找到的最短路径
always @(posedge clk) begin
    if (!rst_n) begin
        close_row9<=0;
        open_row4<=0;
        path_row<=END_ROW;
        path_column<=END_COLUMN;
        e32<=0;
    end
    //状态17，膜32,回溯整条路径
    else if (current_state == S17 && open_row4!=open_row && e32==0) begin
        if (openlist_childy[open_row4] == path_row && openlist_childx[open_row4] == path_column)begin//在openlist中找到
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

//串口发送
always @(posedge clk) begin
    if(!rst_n) begin
        tx_data <= 0;
        tx_flag <= 0;
        tx_cnt <= 0;
        e33<=0;
    end
    //状态18，膜33
    else if (current_state == S18 && tx_done && e33==0) begin
        case (tx_cnt)
            0:begin
                tx_data <= path_row[7:0];
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
                tx_data <= path_column[7:0];
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
            default:begin
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

//判断路径坐标是否发送完
always @(posedge clk) begin
    if (!rst_n) begin
        e34<=0;
    end
    //状态19，膜34
    else if (current_state == S19 && path_row==START_ROW && path_column==START_COLUMN && e34==0) begin//回溯到起点，路径发送完，跳转到S20
        e34<=2;
    end
    else if (current_state == S19 && (path_row!=START_ROW || path_column!=START_COLUMN) && e34==0) begin//没有回溯到起点，路径发送未完成，跳转到S17
        e34<=1;
    end
    else if (current_state == S19 && e34!=0) begin
        e34<=0;
    end
end

//接收完毕判断
always @(posedge clk) begin
    if(!rst_n) begin
        e35 <= 0;
    end
    else if (current_state == S20 && e35 == 0) begin
        e35 <= 1;
    end
    else if (current_state == S20 && e35) begin
        e35 <= 0;
    end
end


endmodule
