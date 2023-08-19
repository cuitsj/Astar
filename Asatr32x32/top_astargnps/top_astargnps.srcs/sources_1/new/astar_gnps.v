module astar_gnps#(
    parameter MAP_ROW = 8'd16,
    parameter MAP_COLUMN = 8'd16,
    parameter MAP_EXP = 8'd4,//2的6次方为64
    parameter START_ROW = 8'd2,
    parameter START_COLUMN = 8'd2,
    parameter END_ROW = 8'd14,
    parameter END_COLUMN = 8'd14
)(
	input                          clk,         
    input                          clk_ap,      
    input                          clk_ila,   
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
localparam      S21      = 5'd21;
localparam      S22      = 5'd22;

reg [4:0] current_state;

//Astar-gnps酶变量
reg e1,e2,e3,e4,e5,e6,e7,e8,e9,e10,e11,e12,e13,e14,e15,e16,e17,e18,e19,e20,e21,e22,e23,e24,e25,e26,e27,e28,e29,e30,e31,e32,e34,e35,e37_1,e37_2;
reg e2_1;
reg [1:0] e33,e36;

//状态0，串口接收数据处理
reg rxprocess_flag;//开始处理标志
reg [7:0] rx_buff;//串口数据缓存
reg [7:0] map_row,map_column;//地图的行列计数
reg [2:0] map_cntbit;//地图数据接收位计数
reg [11:0] map_addra;//地图的RAM地址
//map-BRAM
reg map_en,map_wea,map_dina;
wire map_douta;

//状态1,15,17
reg [11:0]  parent_node[2:0];
reg [11:0] close_row;//closelist的行数
reg [11:0] min_f;
//closelist-BRAM
reg close_ena,close_wea;
reg [11:0] close_addra;
reg [11:0] close_addr1;//地址计数
reg [7:0] close_cyin,close_cxin;
wire [7:0] close_cyout,close_cxout;

//openlist-BRAM
reg openpy_ena,openpy_wea,openpy_enb,openpy_web;
reg openpx_ena,openpx_wea,openpx_enb,openpx_web;
reg opency_ena,opency_wea,opency_enb,opency_web;
reg opencx_ena,opencx_wea,opencx_enb,opencx_web;
reg openg1_ena,openg1_wea,openg1_enb,openg1_web;
reg openg2_ena,openg2_wea,openg2_enb,openg2_web;
reg openh_ena,openh_wea,openh_enb,openh_web;
reg openf_ena,openf_wea,openf_enb,openf_web;

reg [11:0] openpy_addra,openpy_addrb;
reg [11:0] openpx_addra,openpx_addrb;
reg [11:0] opency_addra,opency_addrb;
reg [11:0] opencx_addra,opencx_addrb;
reg [11:0] openg1_addra,openg1_addrb;
reg [11:0] openg2_addra,openg2_addrb;
reg [11:0] openh_addra,openh_addrb;
reg [11:0] openf_addra,openf_addrb;

reg [7:0] openpy_dina,openpy_dinb;
reg [7:0] openpx_dina,openpx_dinb;
reg [7:0] opency_dina,opency_dinb;
reg [7:0] opencx_dina,opencx_dinb;
reg [11:0] openg1_dina,openg1_dinb;
reg [11:0] openg2_dina,openg2_dinb;
reg [11:0] openh_dina,openh_dinb;
reg [11:0] openf_dina,openf_dinb;

wire [7:0] openpy_douta,openpy_doutb;
wire [7:0] openpx_douta,openpx_doutb;
wire [7:0] opency_douta,opency_doutb;
wire [7:0] opencx_douta,opencx_doutb;
wire [11:0] openg1_douta,openg1_doutb;
wire [11:0] openg2_douta,openg2_doutb;
wire [11:0] openh_douta,openh_doutb;
wire [11:0] openf_douta,openf_doutb;

//状态2,3
reg [3:0] child_cnt;
reg new_child1,new_child2,new_child3,new_child4,new_child5,new_child6,new_child7,new_child8;
reg obs_flag1,obs_flag2,obs_flag3,obs_flag4,obs_flag5,obs_flag6,obs_flag7,obs_flag8;

//状态4
reg child_flag1,child_flag2,child_flag3,child_flag4,child_flag5,child_flag6,child_flag7,child_flag8;
reg [7:0] child_node1[1:0],child_node2[1:0],child_node3[1:0],child_node4[1:0],child_node5[1:0],child_node6[1:0],child_node7[1:0],child_node8[1:0];

//状态5,6,7,8,9,10,11,12,14,16
reg [11:0] open_row;//openlist的总行数
reg [11:0] openreal_row;//openlist的真实行数
reg [11:0] open_addr1;//用于判断子节点是否在openlist中计数
reg [11:0] open_addr2;//用于从openlist中找新的父节点计数行数
reg [11:0] open_addr3;//用于从openlist中回溯路径计数
reg inopen_flag1,inopen_flag2,inopen_flag3,inopen_flag4,inopen_flag5,inopen_flag6,inopen_flag7,inopen_flag8;

//状态18
reg findpath_flag;

//状态19
reg [7:0] path_row,path_column;//用于缓存路径各节点坐标

//状态20
reg [1:0] tx_cnt;

//算法状态跳转
always @(posedge clk) begin
    if (!rst_n) begin
        current_state <= S22;//初始状态为S22
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
                if (e3) current_state <= S3;
                else current_state <= S2;
            end
            S3:begin
                if (e4 && e5 && e6 && e7 && e8 && e9 && e10 && e11) current_state <= S4;
                else current_state <= S3;
            end
            S4:begin
                if (e12 && e13 && e14 && e15 && e16 && e17 && e18 && e19) current_state <= S5;
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
                if (e31) current_state <= S17;
                else current_state <= S16;
            end
            S17:begin
                if (e32) current_state <= S18;
                else current_state <= S17;
            end
            S18:begin
                if (e33 == 1) current_state <= S2;//规划未结束
                else if (e33 == 2) current_state <= S22;//规划结束，没有找到最短路径
                else if (e33 == 3) current_state <= S19;//算法结束，并且找到最短路径
                else current_state <= S18;//不满足跳转条件则不跳转
            end
            S19:begin
                if (e34) current_state <= S20;
                else current_state <= S19;
            end
            S20:begin
                if (e35) current_state <= S21;
                else current_state <= S20;
            end
            S21:begin
                if (e36== 1) current_state <= S19;
                else if (e36 == 2) current_state <= S22;
                else current_state <= S21;
            end
            S22:begin
                if (e37_1 && e37_2) current_state <= S0;
                else current_state <= S22;
            end
            default:current_state <= S22;
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
    else if (current_state == S0 && map_cntbit == 3'd7) begin//接收完一个字节
        rxprocess_flag <= 0;
        rx_buff <= 0;
    end
end

//数据计数
always @(posedge clk) begin
    if(!rst_n) begin
        map_row<= 0;
        map_column<= 0;
        e1 <= 0;
        led0<=1'b1;
        map_cntbit<=0;
    end
    else if (current_state == S0 && rxprocess_flag && e1==0) begin
        if (map_cntbit == 3'd7) begin
            if (map_column == MAP_COLUMN-8) begin
                if (map_row == MAP_ROW-1) begin
                    map_cntbit<=0;
                    map_column<=0;
                    map_row <= 0;
                    e1 <= 1;//接收完毕
                    led0<=~led0;//接受完一幅地图LED0翻转一次
                end
                else begin
                    map_cntbit<=0;
                    map_column<=0;
                    map_row<=map_row+8'd1;
                end
            end
            else begin
                map_cntbit<=0;
                map_column<=map_column+8'd8;
            end
        end
        else map_cntbit <= map_cntbit + 3'd1;
    end
    else if (current_state == S0 && e1) begin
        e1 <= 0;
    end
end

//例化BRAM
mapram1 map_i (
  .clka(clk_ap),    // input wire clka
  .ena(map_en),      // input wire ena
  .wea(map_wea),      // input wire [0 : 0] wea
  .addra(map_addra),  // input wire [11 : 0] addra
  .dina(map_dina),    // input wire [0 : 0] dina
  .douta(map_douta)  // output wire [0 : 0] douta
);

//地图的存储和读取
always @(posedge clk) begin
    if(!rst_n) begin
        map_en <= 0;
        map_wea <= 0;
        map_addra <= 0;
        map_dina <= 0;
        e3<=0;
        new_child1<=0;
        new_child2<=0;
        new_child3<=0;
        new_child4<=0;
        new_child5<=0;
        new_child6<=0;
        new_child7<=0;
        new_child8<=0;
    end
    else if (current_state == S0 && rxprocess_flag) begin
        map_en <= 1;
        map_wea <= 1;
        map_addra <= (map_row<<MAP_EXP)+map_column+map_cntbit;
        map_dina <= rx_buff[map_cntbit];
    end
    else if (current_state == S0 && rxprocess_flag == 0) begin
        map_en <= 0;
        map_wea <= 0;
        map_addra <= 0;
        map_dina <= 0;
    end
    //状态2，膜3，读取子节点对应地图点是否为障碍物
    else if (current_state == S2 && e3 == 0) begin
        case (child_cnt)
            0:begin
                map_en <= 1;
                map_wea <= 0;
                map_addra <= (parent_node[0]<<MAP_EXP)+MAP_ROW+parent_node[1]-1;
            end
            1:begin
                new_child1 <= map_douta;
                map_en <= 1;
                map_wea <= 0;
                map_addra <= (parent_node[0]<<MAP_EXP)+MAP_ROW+parent_node[1];
            end
            2:begin
                new_child2 <= map_douta;
                map_en <= 1;
                map_wea <= 0;
                map_addra <= (parent_node[0]<<MAP_EXP)+MAP_ROW+parent_node[1]+1;
            end
            3:begin
                new_child3 <= map_douta;
                map_en <= 1;
                map_wea <= 0;
                map_addra <= (parent_node[0]<<MAP_EXP)+parent_node[1]-1;
            end
            4:begin
                new_child4 <= map_douta;
                map_en <= 1;
                map_wea <= 0;
                map_addra <= (parent_node[0]<<MAP_EXP)+parent_node[1]+1;
            end
            5:begin
                new_child5 <= map_douta;
                map_en <= 1;
                map_wea <= 0;
                map_addra <= (parent_node[0]<<MAP_EXP)-MAP_ROW+parent_node[1]-1;
            end
            6:begin
                new_child6 <= map_douta;
                map_en <= 1;
                map_wea <= 0;
                map_addra <= (parent_node[0]<<MAP_EXP)-MAP_ROW+parent_node[1];
            end
            7:begin
                new_child7 <= map_douta;
                map_en <= 1;
                map_wea <= 0;
                map_addra <= (parent_node[0]<<MAP_EXP)-MAP_ROW+parent_node[1]+1;
            end
            8:begin
                new_child8 <= map_douta;
                map_en <= 0;
                map_wea <= 0;
                map_addra <= 0;
                e3<=1;
            end
            default:begin
                map_en <= 0;
                map_wea <= 0;
                map_addra <= 0;
                map_dina <= 0;
                e3<=1;
            end
        endcase
    end
    else if (current_state == S2 && e3) begin
        e3<=0;
    end
end

//地图数据读取计数
always @(posedge clk) begin
    if(!rst_n) begin
        child_cnt <= 0;
    end
    else if (current_state == S2 && e3 == 0) begin
        if (child_cnt == 8) child_cnt <=0;
        else child_cnt <= child_cnt + 1;
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
    else if (current_state == S0 && findpath_flag==0) begin
        led1<=0;
    end
end

//判断子节点1是否为障碍物
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag1 <= 0;
        e4<=0;
    end
    //状态3，膜4
    else if (current_state == S3 && e4==0 && new_child1 == 0) begin
        obs_flag1 <= 0;
        e4<=1;
    end
    else if (current_state == S3 && e4==0 && new_child1 == 1) begin
        obs_flag1 <= 1;
        e4<=1;
    end
    else if (current_state == S3 && e4 && e5 && e6 && e7 && e8 && e9 && e10 && e11) begin
        e4<=0;
    end
end

//判断子节点2是否为障碍物
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag2 <= 0;
        e5<=0;
    end
    //状态3，膜5
    else if (current_state == S3 && e5==0 && new_child2 == 0) begin
        obs_flag2 <= 0;
        e5<=1;
    end
    else if (current_state == S3 && e5==0 && new_child2 == 1) begin
        obs_flag2 <= 1;
        e5<=1;
    end
    else if (current_state == S3 && e4 && e5 && e6 && e7 && e8 && e9 && e10 && e11) begin
        e5<=0;
    end
end

//判断子节点3是否为障碍物
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag3 <= 0;
        e6<=0;
    end
    //状态3，膜6
    else if (current_state == S3 && e6==0 && new_child3 == 0) begin
        obs_flag3 <= 0;
        e6<=1;
    end
    else if (current_state == S3 && e6==0 && new_child3 == 1) begin
        obs_flag3 <= 1;
        e6<=1;
    end
    else if (current_state == S3 && e4 && e5 && e6 && e7 && e8 && e9 && e10 && e11) begin
        e6<=0;
    end
end

//判断子节点4是否为障碍物
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag4 <= 0;
        e7<=0;
    end
    //状态3，膜7
    else if (current_state == S3 && e7==0 && new_child4 == 0) begin
        obs_flag4 <= 0;
        e7<=1;
    end
    else if (current_state == S3 && e7==0 && new_child4 == 1) begin
        obs_flag4 <= 1;
        e7<=1;
    end
    else if (current_state == S3 && e4 && e5 && e6 && e7 && e8 && e9 && e10 && e11) begin
        e7<=0;
    end
end

//判断子节点5是否为障碍物
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag5 <= 0;
        e8<=0;
    end
    //状态3，膜8
    else if (current_state == S3 && e8==0 && new_child5 == 0) begin
        obs_flag5 <= 0;
        e8<=1;
    end
    else if (current_state == S3 && e8==0 && new_child5 == 1) begin
        obs_flag5 <= 1;
        e8<=1;
    end
    else if (current_state == S3 && e4 && e5 && e6 && e7 && e8 && e9 && e10 && e11) begin
        e8<=0;
    end
end

//判断子节点6是否为障碍物
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag6 <= 0;
        e9<=0;
    end
    //状态3，膜9
    else if (current_state == S3 && e9==0 && new_child6 == 0) begin
        obs_flag6 <= 0;
        e9<=1;
    end
    else if (current_state == S3 && e9==0 && new_child6 == 1) begin
        obs_flag6 <= 1;
        e9<=1;
    end
    else if (current_state == S3 && e4 && e5 && e6 && e7 && e8 && e9 && e10 && e11) begin
        e9<=0;
    end
end

//判断子节点7是否为障碍物
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag7 <= 0;
        e10<=0;
    end
    //状态3，膜10
    else if (current_state == S3 && e10==0 && new_child7 == 0) begin
        obs_flag7 <= 0;
        e10<=1;
    end
    else if (current_state == S3 && e10==0 && new_child7 == 1) begin
        obs_flag7 <= 1;
        e10<=1;
    end
    else if (current_state == S3 && e4 && e5 && e6 && e7 && e8 && e9 && e10 && e11) begin
        e10<=0;
    end
end

//判断子节点8是否为障碍物
always @(posedge clk) begin
    if (!rst_n) begin
        obs_flag8 <= 0;
        e11<=0;
    end
    //状态3，膜11
    else if (current_state == S3 && e11==0 && new_child8 == 0) begin
        obs_flag8 <= 0;
        e11<=1;
    end
    else if (current_state == S3 && e11==0 && new_child8 == 1) begin
        obs_flag8 <= 1;
        e11<=1;
    end
    else if (current_state == S3 && e4 && e5 && e6 && e7 && e8 && e9 && e10 && e11) begin
        e11<=0;
    end
end

closeram8 closelist_childy_i (
  .clka(clk_ap),    // input wire clka
  .ena(close_ena),      // input wire ena
  .wea(close_wea),      // input wire [0 : 0] wea
  .addra(close_addra),  // input wire [11 : 0] addra
  .dina(close_cyin),    // input wire [7 : 0] dina
  .douta(close_cyout)  // output wire [7 : 0] douta
);

closeram8 closelist_childx_i (
  .clka(clk_ap),    // input wire clka
  .ena(close_ena),      // input wire ena
  .wea(close_wea),      // input wire [0 : 0] wea
  .addra(close_addra),  // input wire [11 : 0] addra
  .dina(close_cxin),    // input wire [7 : 0] dina
  .douta(close_cxout)  // output wire [7 : 0] douta
);

//Astar-GNPS
always @(posedge clk) begin
    if (!rst_n) begin
        parent_node[0]<=0;
        parent_node[1]<=0;
        parent_node[2]<=0;
        min_f <= 12'b1111_1111_1111;
        close_row<=0; 
        e37_1 <= 0;
        e2<=0;
        e30<=0;
        close_ena<=0;
        close_wea<=0;
        close_addra<=0;
        close_cyin<=0;
        close_cxin<=0;
        close_addr1<=0;
    end
    //状态22，膜37
    else if (current_state == S22 && e37_1==0) begin
        close_row<=0;
        e37_1 <= 1;
    end
    else if (current_state == S22 && e37_1 && e37_2) begin
        e37_1 <= 0;
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
    
    //状态15，膜30，将父节点添加到closelist中
    else if (current_state == S15 && e30==0) begin
        close_ena<=1;
        close_wea<=1;
        close_addra<=close_row;
        close_cyin<=parent_node[0];
        close_cxin<=parent_node[1];
        close_row <= close_row+1;//closelist的行数
        e30<=1;
    end
    else if (current_state == S15 && e30) begin
        close_ena<=0;
        close_wea<=0;
        close_addra<=0;
        close_cyin<=0;
        close_cxin<=0;
        e30<=0;
    end

    //状态4，判断子节点是否在closelist中
    else if (current_state == S4 && (e12==0 || e13==0 || e14==0 || e15==0 || e16==0 || e17==0 || e18==0 || e19==0) && close_row!=0) begin
        if (close_addr1==close_row) begin
            close_addr1<=0;
            close_ena<=0;
            close_wea<=0;
            close_addra<=0;
        end
        else begin
            close_addr1<=close_addr1+1;
            close_ena<=1;
            close_wea<=0;
            close_addra<=close_addr1;
        end
    end
    else if (current_state == S4 && e12 && e13 && e14 && e15 && e16 && e17 && e18 && e19 && close_addr1!=0) begin//提前结束
        close_addr1<=0;
        close_ena<=0;
        close_wea<=0;
        close_addra<=0;
    end
    
    //状态17，膜32，在openlist中找新的父节点
    else if (current_state == S17 && open_addr2 != open_row && open_addr2!=0 && e32==0) begin
        if (openf_douta < min_f) begin
            min_f <= openf_douta;
            parent_node[0]<=opency_douta;
            parent_node[1]<=opencx_douta;
            parent_node[2]<=open_addr2-1;
        end
    end
    else if (current_state == S17 && open_addr2 == open_row && e32==0) begin
        if (openf_douta < min_f) begin
            parent_node[0]<=opency_douta;
            parent_node[1]<=opencx_douta;
            parent_node[2]<=open_addr2-1;

            min_f <= 12'b1111_1111_1111;
        end
        else begin
            min_f <= 12'b1111_1111_1111;
        end
    end
end

//判断子节点1是否在closelist中
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag1 <= 0;
        child_node1[0]<=0;
        child_node1[1]<=0;
        e12<=0;
    end
    //状态4，膜12
    else if (current_state == S4 && e12==0 && obs_flag1==0 && close_addr1!=close_row && close_addr1!=0 && close_row!=0) begin
        if (parent_node[0]+1 == close_cyout && parent_node[1]-1 == close_cxout) begin//在closelist中
            child_flag1 <= 0;
            child_node1[0]<=0;
            child_node1[1]<=0;
            e12<=1;
        end
    end
    else if (current_state == S4 && e12==0 && obs_flag1==0 && close_addr1==close_row && close_row!=0) begin
        if (parent_node[0]+1 == close_cyout && parent_node[1]-1 == close_cxout) begin//在closelist中
            child_flag1 <= 0;
            child_node1[0]<=0;
            child_node1[1]<=0;
            e12<=1;
        end
        else begin//不在closelist中
            child_flag1 <= 1;
            child_node1[0]<=parent_node[0]+1;
            child_node1[1]<=parent_node[1]-1;
            e12<=1;
        end
    end
    else if (current_state == S4 && e12==0 && obs_flag1) begin//子节点是障碍物
        child_flag1 <= 0;
        child_node1[0]<=0;
        child_node1[1]<=0;
        e12<=1;
    end
    else if (current_state == S4 && e12==0 && close_row==0) begin//closelist为空
        child_flag1 <= 1;
        child_node1[0]<=parent_node[0]+1;
        child_node1[1]<=parent_node[1]-1;
        e12<=1;
    end
    else if (current_state == S4 && e12 && e13 && e14 && e15 && e16 && e17 && e18 && e19)begin
        e12<=0;
    end
end

//判断子节点2是否在closelist中
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag2 <= 0;
        child_node2[0]<=0;
        child_node2[1]<=0;
        e13<=0;
    end
    //状态4，膜13
    else if (current_state == S4 && e13==0 && obs_flag2==0 && close_addr1!=close_row && close_addr1!=0 && close_row!=0) begin
        if (parent_node[0]+1 == close_cyout && parent_node[1] == close_cxout) begin//在closelist中
            child_flag2 <= 0;
            child_node2[0]<=0;
            child_node2[1]<=0;
            e13<=1;
        end
    end
    else if (current_state == S4 && e13==0 && obs_flag2==0 && close_addr1==close_row && close_row!=0) begin//不在closelist中
        if (parent_node[0]+1 == close_cyout && parent_node[1] == close_cxout) begin//在closelist中
            child_flag2 <= 0;
            child_node2[0]<=0;
            child_node2[1]<=0;
            e13<=1;
        end
        else begin
            child_flag2 <= 1;
            child_node2[0]<=parent_node[0]+1;
            child_node2[1]<=parent_node[1];
            e13<=1;
        end
    end
    else if (current_state == S4 && e13==0 && obs_flag2) begin//子节点是障碍物
        child_flag2 <= 0;
        child_node2[0]<=0;
        child_node2[1]<=0;
        e13<=1;
    end
    else if (current_state == S4 && e13==0 && close_row==0) begin//closelist为空
        child_flag2 <= 1;
        child_node2[0]<=parent_node[0]+1;
        child_node2[1]<=parent_node[1];
        e13<=1;
    end
    else if (current_state == S4 && e12 && e13 && e14 && e15 && e16 && e17 && e18 && e19)begin
        e13<=0;
    end
end

//判断子节点3是否在closelist中
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag3 <= 0;
        child_node3[0]<=0;
        child_node3[1]<=0;
        e14<=0;
    end
    //状态4，膜14
    else if (current_state == S4 && e14==0 && obs_flag3==0 && close_addr1!=close_row && close_addr1!=0 && close_row!=0) begin
        if (parent_node[0]+1 == close_cyout && parent_node[1]+1 == close_cxout) begin//在closelist中
            child_flag3 <= 0;
            child_node3[0]<=0;
            child_node3[1]<=0;
            e14<=1;
        end
    end
    else if (current_state == S4 && e14==0 && obs_flag3==0 && close_addr1==close_row && close_row!=0) begin//不在closelist中
        if (parent_node[0]+1 == close_cyout && parent_node[1]+1 == close_cxout) begin//在closelist中
            child_flag3 <= 0;
            child_node3[0]<=0;
            child_node3[1]<=0;
            e14<=1;
        end
        else begin
            child_flag3 <= 1;
            child_node3[0]<=parent_node[0]+1;
            child_node3[1]<=parent_node[1]+1;
            e14<=1;
        end
    end
    else if (current_state == S4 && e14==0 && obs_flag3) begin//子节点是障碍物
        child_flag3 <= 0;
        child_node3[0]<=0;
        child_node3[1]<=0;
        e14<=1;
    end
    else if (current_state == S4 && e14==0 && close_row==0) begin//closelist为空
        child_flag3 <= 1;
        child_node3[0]<=parent_node[0]+1;
        child_node3[1]<=parent_node[1]+1;
        e14<=1;
    end
    else if (current_state == S4 && e12 && e13 && e14 && e15 && e16 && e17 && e18 && e19)begin
        e14<=0;
    end
end

//判断子节点4是否在closelist中
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag4 <= 0;
        child_node4[0]<=0;
        child_node4[1]<=0;
        e15<=0;
    end
    //状态4，膜15
    else if (current_state == S4 && e15==0 && obs_flag4==0 && close_addr1!=close_row && close_addr1!=0 && close_row!=0) begin
        if (parent_node[0] == close_cyout && parent_node[1]-1 == close_cxout) begin//在closelist中
            child_flag4 <= 0;
            child_node4[0]<=0;
            child_node4[1]<=0;
            e15<=1;
        end
    end
    else if (current_state == S4 && e15==0 && obs_flag4==0 && close_addr1==close_row && close_row!=0) begin//不在closelist中
        if (parent_node[0] == close_cyout && parent_node[1]-1 == close_cxout) begin//在closelist中
            child_flag4 <= 0;
            child_node4[0]<=0;
            child_node4[1]<=0;
            e15<=1;
        end
        else begin
            child_flag4 <= 1;
            child_node4[0]<=parent_node[0];
            child_node4[1]<=parent_node[1]-1;
            e15<=1;
        end
    end
    else if (current_state == S4 && e15==0 && obs_flag4) begin//子节点是障碍物
        child_flag4 <= 0;
        child_node4[0]<=0;
        child_node4[1]<=0;
        e15<=1;
    end
    else if (current_state == S4 && e15==0 && close_row==0) begin//closelist为空
        child_flag4 <= 1;
        child_node4[0]<=parent_node[0];
        child_node4[1]<=parent_node[1]-1;
        e15<=1;
    end
    else if (current_state == S4 && e12 && e13 && e14 && e15 && e16 && e17 && e18 && e19)begin
        e15<=0;
    end
end

//判断子节点5是否在closelist中
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag5 <= 0;
        child_node5[0]<=0;
        child_node5[1]<=0;
        e16<=0;
    end
    //状态4，膜16
    else if (current_state == S4 && e16==0 && obs_flag5==0 && close_addr1!=close_row && close_addr1!=0 && close_row!=0) begin
        if (parent_node[0] == close_cyout && parent_node[1]+1 == close_cxout) begin//在closelist中
            child_flag5 <= 0;
            child_node5[0]<=0;
            child_node5[1]<=0;
            e16<=1;
        end
    end
    else if (current_state == S4 && e16==0 && obs_flag5==0 && close_addr1==close_row && close_row!=0) begin//不在closelist中
        if (parent_node[0] == close_cyout && parent_node[1]+1 == close_cxout) begin//在closelist中
            child_flag5 <= 0;
            child_node5[0]<=0;
            child_node5[1]<=0;
            e16<=1;
        end
        else begin
            child_flag5 <= 1;
            child_node5[0]<=parent_node[0];
            child_node5[1]<=parent_node[1]+1;
            e16<=1;
        end
    end
    else if (current_state == S4 && e16==0 && obs_flag5) begin//子节点是障碍物
        child_flag5 <= 0;
        child_node5[0]<=0;
        child_node5[1]<=0;
        e16<=1;
    end
    else if (current_state == S4 && e16==0 && close_row==0) begin//closelist为空
        child_flag5 <= 1;
        child_node5[0]<=parent_node[0];
        child_node5[1]<=parent_node[1]+1;
        e16<=1;
    end
    else if (current_state == S4 && e12 && e13 && e14 && e15 && e16 && e17 && e18 && e19)begin
        e16<=0;
    end
end

//判断子节点6是否在closelist中
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag6 <= 0;
        child_node6[0]<=0;
        child_node6[1]<=0;
        e17<=0;
    end
    //状态4，膜17
    else if (current_state == S4 && e17==0 && obs_flag6==0 && close_addr1!=close_row && close_addr1!=0 && close_row!=0) begin
        if (parent_node[0]-1 == close_cyout && parent_node[1]-1 == close_cxout) begin//在closelist中
            child_flag6 <= 0;
            child_node6[0]<=0;
            child_node6[1]<=0;
            e17<=1;
        end
    end
    else if (current_state == S4 && e17==0 && obs_flag6==0 && close_addr1==close_row && close_row!=0) begin//不在closelist中
        if (parent_node[0]-1 == close_cyout && parent_node[1]-1 == close_cxout) begin//在closelist中
            child_flag6 <= 0;
            child_node6[0]<=0;
            child_node6[1]<=0;
            e17<=1;
        end
        else begin
            child_flag6 <= 1;
            child_node6[0]<=parent_node[0]-1;
            child_node6[1]<=parent_node[1]-1;
            e17<=1;
        end
    end
    else if (current_state == S4 && e17==0 && obs_flag6) begin//子节点是障碍物
        child_flag6 <= 0;
        child_node6[0]<=0;
        child_node6[1]<=0;
        e17<=1;
    end
    else if (current_state == S4 && e17==0 && close_row==0) begin//closelist为空
        child_flag6 <= 1;
        child_node6[0]<=parent_node[0]-1;
        child_node6[1]<=parent_node[1]-1;
        e17<=1;
    end
    else if (current_state == S4 && e12 && e13 && e14 && e15 && e16 && e17 && e18 && e19)begin
        e17<=0;
    end
end

//判断子节点7是否在closelist中
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag7 <= 0;
        child_node7[0]<=0;
        child_node7[1]<=0;
        e18<=0;
    end
    //状态4，膜18
    else if (current_state == S4 && e18==0 && obs_flag7==0 && close_addr1!=close_row && close_addr1!=0 && close_row!=0) begin
        if (parent_node[0]-1 == close_cyout && parent_node[1] == close_cxout) begin//在closelist中
            child_flag7 <= 0;
            child_node7[0]<=0;
            child_node7[1]<=0;
            e18<=1;
        end
    end
    else if (current_state == S4 && e18==0 && obs_flag7==0 && close_addr1==close_row && close_row!=0) begin//不在closelist中
        if (parent_node[0]-1 == close_cyout && parent_node[1] == close_cxout) begin//在closelist中
            child_flag7 <= 0;
            child_node7[0]<=0;
            child_node7[1]<=0;
            e18<=1;
        end
        else begin
            child_flag7 <= 1;
            child_node7[0]<=parent_node[0]-1;
            child_node7[1]<=parent_node[1];
            e18<=1;
        end
    end
    else if (current_state == S4 && e18==0 && obs_flag7) begin//子节点是障碍物
        child_flag7 <= 0;
        child_node7[0]<=0;
        child_node7[1]<=0;
        e18<=1;
    end
    else if (current_state == S4 && e18==0 && close_row==0) begin//closelist为空
        child_flag7 <= 1;
        child_node7[0]<=parent_node[0]-1;
        child_node7[1]<=parent_node[1];
        e18<=1;
end
    else if (current_state == S4 && e12 && e13 && e14 && e15 && e16 && e17 && e18 && e19)begin
        e18<=0;
    end
end

//判断子节点8是否在closelist中
always @(posedge clk) begin
    if (!rst_n) begin
        child_flag8 <= 0;
        child_node8[0]<=0;
        child_node8[1]<=0;
        e19<=0;
    end
    //状态4，膜19
    else if (current_state == S4 && e19==0 && obs_flag8==0 && close_addr1!=close_row && close_addr1!=0 && close_row!=0) begin
        if (parent_node[0]-1 == close_cyout && parent_node[1]+1 == close_cxout) begin//在closelist中
            child_flag8 <= 0;
            child_node8[0]<=0;
            child_node8[1]<=0;
            e19<=1;
        end
    end
    else if (current_state == S4 && e19==0 && obs_flag8==0 && close_addr1==close_row && close_row!=0) begin//不在closelist中
        if (parent_node[0]-1 == close_cyout && parent_node[1]+1 == close_cxout) begin//在closelist中
            child_flag8 <= 0;
            child_node8[0]<=0;
            child_node8[1]<=0;
            e19<=1;
        end
        else begin
            child_flag8 <= 1;
            child_node8[0]<=parent_node[0]-1;
            child_node8[1]<=parent_node[1]+1;
            e19<=1;
        end

    end
    else if (current_state == S4 && e19==0 && obs_flag8) begin//子节点是障碍物
        child_flag8 <= 0;
        child_node8[0]<=0;
        child_node8[1]<=0;
        e19<=1;
    end
    else if (current_state == S4 && e19==0 && close_row==0) begin//closelist为空
        child_flag8 <= 1;
        child_node8[0]<=parent_node[0]-1;
        child_node8[1]<=parent_node[1]+1;
        e19<=1;
    end
    else if (current_state == S4 && e12 && e13 && e14 && e15 && e16 && e17 && e18 && e19)begin
        e19<=0;
    end
end

openram8 openlist_parenty_i (
  .clka(clk_ap),    // input wire clka
  .ena(openpy_ena),      // input wire ena
  .wea(openpy_wea),      // input wire [0 : 0] wea
  .addra(openpy_addra),  // input wire [11 : 0] addra
  .dina(openpy_dina),    // input wire [7 : 0] dina
  .douta(openpy_douta),  // output wire [7 : 0] douta
  .clkb(clk_ap),    // input wire clkb
  .enb(openpy_enb),      // input wire enb
  .web(openpy_web),      // input wire [0 : 0] web
  .addrb(openpy_addrb),  // input wire [11 : 0] addrb
  .dinb(openpy_dinb),    // input wire [7 : 0] dinb
  .doutb(openpy_doutb)  // output wire [7 : 0] doutb
);

openram8 openlist_parentx_i (
  .clka(clk_ap),    // input wire clka
  .ena(openpx_ena),      // input wire ena
  .wea(openpx_wea),      // input wire [0 : 0] wea
  .addra(openpx_addra),  // input wire [11 : 0] addra
  .dina(openpx_dina),    // input wire [7 : 0] dina
  .douta(openpx_douta),  // output wire [7 : 0] douta
  .clkb(clk_ap),    // input wire clkb
  .enb(openpx_enb),      // input wire enb
  .web(openpx_web),      // input wire [0 : 0] web
  .addrb(openpx_addrb),  // input wire [11 : 0] addrb
  .dinb(openpx_dinb),    // input wire [7 : 0] dinb
  .doutb(openpx_doutb)  // output wire [7 : 0] doutb
);

openram8 openlist_childy_i (
  .clka(clk_ap),    // input wire clka
  .ena(opency_ena),      // input wire ena
  .wea(opency_wea),      // input wire [0 : 0] wea
  .addra(opency_addra),  // input wire [11 : 0] addra
  .dina(opency_dina),    // input wire [7 : 0] dina
  .douta(opency_douta),  // output wire [7 : 0] douta
  .clkb(clk_ap),    // input wire clkb
  .enb(opency_enb),      // input wire enb
  .web(opency_web),      // input wire [0 : 0] web
  .addrb(opency_addrb),  // input wire [11 : 0] addrb
  .dinb(opency_dinb),    // input wire [7 : 0] dinb
  .doutb(opency_doutb)  // output wire [7 : 0] doutb
);

openram8 openlist_childx_i (
  .clka(clk_ap),    // input wire clka
  .ena(opencx_ena),      // input wire ena
  .wea(opencx_wea),      // input wire [0 : 0] wea
  .addra(opencx_addra),  // input wire [11 : 0] addra
  .dina(opencx_dina),    // input wire [7 : 0] dina
  .douta(opencx_douta),  // output wire [7 : 0] douta
  .clkb(clk_ap),    // input wire clkb
  .enb(opencx_enb),      // input wire enb
  .web(opencx_web),      // input wire [0 : 0] web
  .addrb(opencx_addrb),  // input wire [11 : 0] addrb
  .dinb(opencx_dinb),    // input wire [7 : 0] dinb
  .doutb(opencx_doutb)  // output wire [7 : 0] doutb
);

openram12 openlist_g1_i (
  .clka(clk_ap),    // input wire clka
  .ena(openg1_ena),      // input wire ena
  .wea(openg1_wea),      // input wire [0 : 0] wea
  .addra(openg1_addra),  // input wire [11 : 0] addra
  .dina(openg1_dina),    // input wire [7 : 0] dina
  .douta(openg1_douta),  // output wire [7 : 0] douta
  .clkb(clk_ap),    // input wire clkb
  .enb(openg1_enb),      // input wire enb
  .web(openg1_web),      // input wire [0 : 0] web
  .addrb(openg1_addrb),  // input wire [11 : 0] addrb
  .dinb(openg1_dinb),    // input wire [7 : 0] dinb
  .doutb(openg1_doutb)  // output wire [7 : 0] doutb
);

openram12 openlist_g2_i (
  .clka(clk_ap),    // input wire clka
  .ena(openg2_ena),      // input wire ena
  .wea(openg2_wea),      // input wire [0 : 0] wea
  .addra(openg2_addra),  // input wire [11 : 0] addra
  .dina(openg2_dina),    // input wire [7 : 0] dina
  .douta(openg2_douta),  // output wire [7 : 0] douta
  .clkb(clk_ap),    // input wire clkb
  .enb(openg2_enb),      // input wire enb
  .web(openg2_web),      // input wire [0 : 0] web
  .addrb(openg2_addrb),  // input wire [11 : 0] addrb
  .dinb(openg2_dinb),    // input wire [7 : 0] dinb
  .doutb(openg2_doutb)  // output wire [7 : 0] doutb
);

openram12 openlist_h_i (
  .clka(clk_ap),    // input wire clka
  .ena(openh_ena),      // input wire ena
  .wea(openh_wea),      // input wire [0 : 0] wea
  .addra(openh_addra),  // input wire [11 : 0] addra
  .dina(openh_dina),    // input wire [7 : 0] dina
  .douta(openh_douta),  // output wire [7 : 0] douta
  .clkb(clk_ap),    // input wire clkb
  .enb(openh_enb),      // input wire enb
  .web(openh_web),      // input wire [0 : 0] web
  .addrb(openh_addrb),  // input wire [11 : 0] addrb
  .dinb(openh_dinb),    // input wire [7 : 0] dinb
  .doutb(openh_doutb)  // output wire [7 : 0] doutb
);

openram12 openlist_f_i (
  .clka(clk_ap),    // input wire clka
  .ena(openf_ena),      // input wire ena
  .wea(openf_wea),      // input wire [0 : 0] wea
  .addra(openf_addra),  // input wire [11 : 0] addra
  .dina(openf_dina),    // input wire [7 : 0] dina
  .douta(openf_douta),  // output wire [7 : 0] douta
  .clkb(clk_ap),    // input wire clkb
  .enb(openf_enb),      // input wire enb
  .web(openf_web),      // input wire [0 : 0] web
  .addrb(openf_addrb),  // input wire [11 : 0] addrb
  .dinb(openf_dinb),    // input wire [7 : 0] dinb
  .doutb(openf_doutb)  // output wire [7 : 0] doutb
);

//openlist
always @(posedge clk) begin
    if (!rst_n) begin
        openpy_ena<=0;
        openpx_ena<=0;
        opency_ena<=0;
        opencx_ena<=0;
        openg1_ena<=0;
        openg2_ena<=0;
        openh_ena<=0;
        openf_ena<=0;

        openpy_wea<=0;
        openpx_wea<=0;
        opency_wea<=0;
        opencx_wea<=0;
        openg1_wea<=0;
        openg2_wea<=0;
        openh_wea<=0;
        openf_wea<=0;

        openpy_addra<=0;
        openpx_addra<=0;
        opency_addra<=0;
        opencx_addra<=0;
        openg1_addra<=0;
        openg2_addra<=0;
        openh_addra<=0;
        openf_addra<=0;

        openpy_dina<=0;
        openpx_dina<=0;
        opency_dina<=0;
        opencx_dina<=0;
        openg1_dina<=0;
        openg2_dina<=0;
        openh_dina<=0;
        openf_dina<=0;

        openpy_enb<=0;
        openpx_enb<=0;
        opency_enb<=0;
        opencx_enb<=0;
        openg1_enb<=0;
        openg2_enb<=0;
        openh_enb<=0;
        openf_enb<=0;

        openpy_web<=0;
        openpx_web<=0;
        opency_web<=0;
        opencx_web<=0;
        openg1_web<=0;
        openg2_web<=0;
        openh_web<=0;
        openf_web<=0;

        openpy_addrb<=0;
        openpx_addrb<=0;
        opency_addrb<=0;
        opencx_addrb<=0;
        openg1_addrb<=0;
        openg2_addrb<=0;
        openh_addrb<=0;
        openf_addrb<=0;

        openpy_dinb<=0;
        openpx_dinb<=0;
        opency_dinb<=0;
        opencx_dinb<=0;
        openg1_dinb<=0;
        openg2_dinb<=0;
        openh_dinb<=0;
        openf_dinb<=0;
        //以上为openlist-BRAM的初始化

        open_addr1<=0;
        open_addr2<=0;
        open_addr3<=0;

        path_row<=END_ROW;
        path_column<=END_COLUMN;
        
        e2_1<=0;
        e20<=0;
        e21<=0;
        e22<=0;
        e23<=0;
        e24<=0;
        e25<=0;
        e26<=0;
        e27<=0;
        e28<=0;
        e29<=0;
        e31<=0;
        e32<=0;
        e34<=0;
        e37_2<=0;
        open_row<=0;
        openreal_row<=0;
        inopen_flag1<=0;
        inopen_flag2<=0;
        inopen_flag3<=0;
        inopen_flag4<=0;
        inopen_flag5<=0;
        inopen_flag6<=0;
        inopen_flag7<=0;
        inopen_flag8<=0;

    end
    //状态22，膜37
    else if (current_state == S22 && e37_2==0) begin
        open_row<=0;
        openreal_row<=0;
        path_row<=END_ROW;
        path_column<=END_COLUMN;
        e37_2 <= 1;
    end
    else if (current_state == S22 && e37_1 && e37_2) begin
        e37_2 <= 0;
    end
    //状态1,膜2
    else if (current_state == S1 && e2_1 == 0) begin//将起点写入openlist
        openpy_ena<=1;
        openpx_ena<=1;
        opency_ena<=1;
        opencx_ena<=1;
        openg1_ena<=1;
        openg2_ena<=1;
        openh_ena<=1;
        openf_ena<=1;

        openpy_wea<=1;
        openpx_wea<=1;
        opency_wea<=1;
        opencx_wea<=1;
        openg1_wea<=1;
        openg2_wea<=1;
        openh_wea<=1;
        openf_wea<=1;

        openpy_addra<=0;
        openpx_addra<=0;
        opency_addra<=0;
        opencx_addra<=0;
        openg1_addra<=0;
        openg2_addra<=0;
        openh_addra<=0;
        openf_addra<=0;

        openpy_dina<=START_ROW;
        openpx_dina<=START_COLUMN;
        opency_dina<=START_ROW;
        opencx_dina<=START_COLUMN;
        openg1_dina<=0;
        openg2_dina<=0;
        openh_dina<=((END_ROW-START_ROW)+(END_COLUMN-START_COLUMN))<<3;
        openf_dina<=(((END_ROW-START_ROW)+(END_COLUMN-START_COLUMN))<<3);

        openreal_row<=openreal_row+1;
        open_row<=open_row+1;
        e2_1<=1;
    end
    else if (current_state == S1 && e2 && e2_1) begin
        e2_1<=0;
    end

    //状态5，膜20,判断子节点是否在openlist中
    else if (current_state == S5 && e20==0 && open_addr1==0) begin
        open_addr1<=open_addr1+1;

        opency_ena<=1;
        opencx_ena<=1;
        openg1_enb<=1;
        openg2_enb<=1;
        openh_ena<=1;

        opency_wea<=0;
        opencx_wea<=0;
        openg1_web<=0;
        openg2_web<=0;
        openh_wea<=0;

        opency_addra<=open_addr1;
        opencx_addra<=open_addr1;
        openg1_addrb<=open_addr1;
        openg2_addrb<=parent_node[2];//读取父节点的g值
        openh_addra<=open_addr1;
    end
    else if (current_state == S5 && e20==0 && open_addr1!=open_row+1 && open_addr1!=0) begin
        if (child_flag1 && child_node1[0] == opency_douta && child_node1[1] == opencx_douta)begin//子节点1在openlist中
            if (openg2_doutb+11<openg1_doutb) begin//判断是否要更新原来的点的父节点、g和f

                openpy_ena<=1;
                openpx_ena<=1;
                openg1_ena<=1;
                openg2_ena<=1;
                openf_ena<=1;

                openpy_wea<=1;
                openpx_wea<=1;
                openg1_wea<=1;
                openg2_wea<=1;
                openf_wea<=1;

                openpy_addra<=open_addr1-1;
                openpx_addra<=open_addr1-1;
                openg1_addra<=open_addr1-1;
                openg2_addra<=open_addr1-1;
                openf_addra<=open_addr1-1;

                openpy_dina<=parent_node[0];
                openpx_dina<=parent_node[1];
                openg1_dina<=openg2_doutb+11;
                openg2_dina<=openg2_doutb+11;
                openf_dina<=openg2_doutb+11+openh_douta;

                open_addr1<=open_addr1+1;
                inopen_flag1 <= 1;

                opency_ena<=1;
                opencx_ena<=1;
                openg1_enb<=1;
                openg2_enb<=1;
                openh_ena<=1;

                opency_wea<=0;
                opencx_wea<=0;
                openg1_web<=0;
                openg2_web<=0;
                openh_wea<=0;

                opency_addra<=open_addr1;
                opencx_addra<=open_addr1;
                openg1_addrb<=open_addr1;
                openg2_addrb<=parent_node[2];//读取父节点的g值
                openh_addra<=open_addr1;
            end
            else begin 
                opency_ena<=1;
                opencx_ena<=1;
                openg1_enb<=1;
                openg2_enb<=1;
                openh_ena<=1;

                opency_wea<=0;
                opencx_wea<=0;
                openg1_web<=0;
                openg2_web<=0;
                openh_wea<=0;

                opency_addra<=open_addr1;
                opencx_addra<=open_addr1;
                openg1_addrb<=open_addr1;
                openg2_addrb<=parent_node[2];//读取父节点的g值
                openh_addra<=open_addr1;

                inopen_flag1 <= 1;
                open_addr1<=open_addr1+1;
            end
        end
        else if (child_flag2 && child_node2[0] == opency_douta && child_node2[1] == opencx_douta)begin//子节点2在openlist中
            if (openg2_doutb+8<openg1_doutb) begin//新的g比原来的g小，则更新原来的点的父节点和g,f
                openpy_ena<=1;
                openpx_ena<=1;
                openg1_ena<=1;
                openg2_ena<=1;
                openf_ena<=1;

                openpy_wea<=1;
                openpx_wea<=1;
                openg1_wea<=1;
                openg2_wea<=1;
                openf_wea<=1;

                openpy_addra<=open_addr1-1;
                openpx_addra<=open_addr1-1;
                openg1_addra<=open_addr1-1;
                openg2_addra<=open_addr1-1;
                openf_addra<=open_addr1-1;

                openpy_dina<=parent_node[0];
                openpx_dina<=parent_node[1];
                openg1_dina<=openg2_doutb+8;
                openg2_dina<=openg2_doutb+8;
                openf_dina<=openg2_doutb+8+openh_douta;

                open_addr1<=open_addr1+1;
                inopen_flag2 <= 1;

                opency_ena<=1;
                opencx_ena<=1;
                openg1_enb<=1;
                openg2_enb<=1;
                openh_ena<=1;

                opency_wea<=0;
                opencx_wea<=0;
                openg1_web<=0;
                openg2_web<=0;
                openh_wea<=0;

                opency_addra<=open_addr1;
                opencx_addra<=open_addr1;
                openg1_addrb<=open_addr1;
                openg2_addrb<=parent_node[2];//读取父节点的g值
                openh_addra<=open_addr1;
            end
            else begin
                opency_ena<=1;
                opencx_ena<=1;
                openg1_enb<=1;
                openg2_enb<=1;
                openh_ena<=1;

                opency_wea<=0;
                opencx_wea<=0;
                openg1_web<=0;
                openg2_web<=0;
                openh_wea<=0;

                opency_addra<=open_addr1;
                opencx_addra<=open_addr1;
                openg1_addrb<=open_addr1;
                openg2_addrb<=parent_node[2];//读取父节点的g值
                openh_addra<=open_addr1;

                inopen_flag2 <= 1;
                open_addr1<=open_addr1+1;
            end
        end
        else if (child_flag3 && child_node3[0] == opency_douta && child_node3[1] == opencx_douta)begin//子节点3在openlist中
            if (openg2_doutb+11<openg1_doutb) begin//新的g比原来的g小，则更新原来的点的父节点和g,f
                openpy_ena<=1;
                openpx_ena<=1;
                openg1_ena<=1;
                openg2_ena<=1;
                openf_ena<=1;

                openpy_wea<=1;
                openpx_wea<=1;
                openg1_wea<=1;
                openg2_wea<=1;
                openf_wea<=1;

                openpy_addra<=open_addr1-1;
                openpx_addra<=open_addr1-1;
                openg1_addra<=open_addr1-1;
                openg2_addra<=open_addr1-1;
                openf_addra<=open_addr1-1;

                openpy_dina<=parent_node[0];
                openpx_dina<=parent_node[1];
                openg1_dina<=openg2_doutb+11;
                openg2_dina<=openg2_doutb+11;
                openf_dina<=openg2_doutb+11+openh_douta;

                open_addr1<=open_addr1+1;
                inopen_flag3 <= 1;

                opency_ena<=1;
                opencx_ena<=1;
                openg1_enb<=1;
                openg2_enb<=1;
                openh_ena<=1;

                opency_wea<=0;
                opencx_wea<=0;
                openg1_web<=0;
                openg2_web<=0;
                openh_wea<=0;

                opency_addra<=open_addr1;
                opencx_addra<=open_addr1;
                openg1_addrb<=open_addr1;
                openg2_addrb<=parent_node[2];//读取父节点的g值
                openh_addra<=open_addr1;
            end
            else begin
                opency_ena<=1;
                opencx_ena<=1;
                openg1_enb<=1;
                openg2_enb<=1;
                openh_ena<=1;

                opency_wea<=0;
                opencx_wea<=0;
                openg1_web<=0;
                openg2_web<=0;
                openh_wea<=0;

                opency_addra<=open_addr1;
                opencx_addra<=open_addr1;
                openg1_addrb<=open_addr1;
                openg2_addrb<=parent_node[2];//读取父节点的g值
                openh_addra<=open_addr1;

                inopen_flag3 <= 1;
                open_addr1<=open_addr1+1;
            end
        end
        else if (child_flag4 && child_node4[0] == opency_douta && child_node4[1] == opencx_douta)begin//子节点4在openlist中
            if (openg2_doutb+8<openg1_doutb) begin//新的g比原来的g小，则更新原来的点的父节点和g,f
                openpy_ena<=1;
                openpx_ena<=1;
                openg1_ena<=1;
                openg2_ena<=1;
                openf_ena<=1;

                openpy_wea<=1;
                openpx_wea<=1;
                openg1_wea<=1;
                openg2_wea<=1;
                openf_wea<=1;

                openpy_addra<=open_addr1-1;
                openpx_addra<=open_addr1-1;
                openg1_addra<=open_addr1-1;
                openg2_addra<=open_addr1-1;
                openf_addra<=open_addr1-1;

                openpy_dina<=parent_node[0];
                openpx_dina<=parent_node[1];
                openg1_dina<=openg2_doutb+8;
                openg2_dina<=openg2_doutb+8;
                openf_dina<=openg2_doutb+8+openh_douta;

                open_addr1<=open_addr1+1;
                inopen_flag4 <= 1;

                opency_ena<=1;
                opencx_ena<=1;
                openg1_enb<=1;
                openg2_enb<=1;
                openh_ena<=1;

                opency_wea<=0;
                opencx_wea<=0;
                openg1_web<=0;
                openg2_web<=0;
                openh_wea<=0;

                opency_addra<=open_addr1;
                opencx_addra<=open_addr1;
                openg1_addrb<=open_addr1;
                openg2_addrb<=parent_node[2];//读取父节点的g值
                openh_addra<=open_addr1;
            end
            else begin
                opency_ena<=1;
                opencx_ena<=1;
                openg1_enb<=1;
                openg2_enb<=1;
                openh_ena<=1;

                opency_wea<=0;
                opencx_wea<=0;
                openg1_web<=0;
                openg2_web<=0;
                openh_wea<=0;

                opency_addra<=open_addr1;
                opencx_addra<=open_addr1;
                openg1_addrb<=open_addr1;
                openg2_addrb<=parent_node[2];//读取父节点的g值
                openh_addra<=open_addr1;

                inopen_flag4 <= 1;
                open_addr1<=open_addr1+1;
            end
        end
        else if (child_flag5 && child_node5[0] == opency_douta && child_node5[1] == opencx_douta)begin//子节点5在openlist中
            if (openg2_doutb+8<openg1_doutb) begin//新的g比原来的g小，则更新原来的点的父节点和g,f
                openpy_ena<=1;
                openpx_ena<=1;
                openg1_ena<=1;
                openg2_ena<=1;
                openf_ena<=1;

                openpy_wea<=1;
                openpx_wea<=1;
                openg1_wea<=1;
                openg2_wea<=1;
                openf_wea<=1;

                openpy_addra<=open_addr1-1;
                openpx_addra<=open_addr1-1;
                openg1_addra<=open_addr1-1;
                openg2_addra<=open_addr1-1;
                openf_addra<=open_addr1-1;

                openpy_dina<=parent_node[0];
                openpx_dina<=parent_node[1];
                openg1_dina<=openg2_doutb+8;
                openg2_dina<=openg2_doutb+8;
                openf_dina<=openg2_doutb+8+openh_douta;

                open_addr1<=open_addr1+1;
                inopen_flag5 <= 1;

                opency_ena<=1;
                opencx_ena<=1;
                openg1_enb<=1;
                openg2_enb<=1;
                openh_ena<=1;

                opency_wea<=0;
                opencx_wea<=0;
                openg1_web<=0;
                openg2_web<=0;
                openh_wea<=0;

                opency_addra<=open_addr1;
                opencx_addra<=open_addr1;
                openg1_addrb<=open_addr1;
                openg2_addrb<=parent_node[2];//读取父节点的g值
                openh_addra<=open_addr1;
            end
            else begin
                opency_ena<=1;
                opencx_ena<=1;
                openg1_enb<=1;
                openg2_enb<=1;
                openh_ena<=1;

                opency_wea<=0;
                opencx_wea<=0;
                openg1_web<=0;
                openg2_web<=0;
                openh_wea<=0;

                opency_addra<=open_addr1;
                opencx_addra<=open_addr1;
                openg1_addrb<=open_addr1;
                openg2_addrb<=parent_node[2];//读取父节点的g值
                openh_addra<=open_addr1;

                inopen_flag5 <= 1;
                open_addr1<=open_addr1+1;
            end
        end
        else if (child_flag6 && child_node6[0] == opency_douta && child_node6[1] == opencx_douta)begin//子节点6在openlist中
            if (openg2_doutb+11<openg1_doutb) begin//新的g比原来的g小，则更新原来的点的父节点和g,f
                openpy_ena<=1;
                openpx_ena<=1;
                openg1_ena<=1;
                openg2_ena<=1;
                openf_ena<=1;

                openpy_wea<=1;
                openpx_wea<=1;
                openg1_wea<=1;
                openg2_wea<=1;
                openf_wea<=1;

                openpy_addra<=open_addr1-1;
                openpx_addra<=open_addr1-1;
                openg1_addra<=open_addr1-1;
                openg2_addra<=open_addr1-1;
                openf_addra<=open_addr1-1;

                openpy_dina<=parent_node[0];
                openpx_dina<=parent_node[1];
                openg1_dina<=openg2_doutb+11;
                openg2_dina<=openg2_doutb+11;
                openf_dina<=openg2_doutb+11+openh_douta;

                open_addr1<=open_addr1+1;
                inopen_flag6 <= 1;

                opency_ena<=1;
                opencx_ena<=1;
                openg1_enb<=1;
                openg2_enb<=1;
                openh_ena<=1;

                opency_wea<=0;
                opencx_wea<=0;
                openg1_web<=0;
                openg2_web<=0;
                openh_wea<=0;

                opency_addra<=open_addr1;
                opencx_addra<=open_addr1;
                openg1_addrb<=open_addr1;
                openg2_addrb<=parent_node[2];//读取父节点的g值
                openh_addra<=open_addr1;
            end
            else begin
                opency_ena<=1;
                opencx_ena<=1;
                openg1_enb<=1;
                openg2_enb<=1;
                openh_ena<=1;

                opency_wea<=0;
                opencx_wea<=0;
                openg1_web<=0;
                openg2_web<=0;
                openh_wea<=0;

                opency_addra<=open_addr1;
                opencx_addra<=open_addr1;
                openg1_addrb<=open_addr1;
                openg2_addrb<=parent_node[2];//读取父节点的g值
                openh_addra<=open_addr1;

                inopen_flag6 <= 1;
                open_addr1<=open_addr1+1;
            end
        end
        else if (child_flag7 && child_node7[0] == opency_douta && child_node7[1] == opencx_douta)begin//子节点7在openlist中
            if (openg2_doutb+8<openg1_doutb) begin//新的g比原来的g小，则更新原来的点的父节点和g,f
                openpy_ena<=1;
                openpx_ena<=1;
                openg1_ena<=1;
                openg2_ena<=1;
                openf_ena<=1;

                openpy_wea<=1;
                openpx_wea<=1;
                openg1_wea<=1;
                openg2_wea<=1;
                openf_wea<=1;

                openpy_addra<=open_addr1-1;
                openpx_addra<=open_addr1-1;
                openg1_addra<=open_addr1-1;
                openg2_addra<=open_addr1-1;
                openf_addra<=open_addr1-1;

                openpy_dina<=parent_node[0];
                openpx_dina<=parent_node[1];
                openg1_dina<=openg2_doutb+8;
                openg2_dina<=openg2_doutb+8;
                openf_dina<=openg2_doutb+8+openh_douta;

                open_addr1<=open_addr1+1;
                inopen_flag7 <= 1;

                opency_ena<=1;
                opencx_ena<=1;
                openg1_enb<=1;
                openg2_enb<=1;
                openh_ena<=1;

                opency_wea<=0;
                opencx_wea<=0;
                openg1_web<=0;
                openg2_web<=0;
                openh_wea<=0;

                opency_addra<=open_addr1;
                opencx_addra<=open_addr1;
                openg1_addrb<=open_addr1;
                openg2_addrb<=parent_node[2];//读取父节点的g值
                openh_addra<=open_addr1;
            end
            else begin
                opency_ena<=1;
                opencx_ena<=1;
                openg1_enb<=1;
                openg2_enb<=1;
                openh_ena<=1;

                opency_wea<=0;
                opencx_wea<=0;
                openg1_web<=0;
                openg2_web<=0;
                openh_wea<=0;

                opency_addra<=open_addr1;
                opencx_addra<=open_addr1;
                openg1_addrb<=open_addr1;
                openg2_addrb<=parent_node[2];//读取父节点的g值
                openh_addra<=open_addr1;

                inopen_flag7 <= 1;
                open_addr1<=open_addr1+1;
            end
        end
        else if (child_flag8 && child_node8[0] == opency_douta && child_node8[1] == opencx_douta)begin//子节点8在openlist中
            if (openg2_doutb+11<openg1_doutb) begin//新的g比原来的g小，则更新原来的点的父节点和g,f
                openpy_ena<=1;
                openpx_ena<=1;
                openg1_ena<=1;
                openg2_ena<=1;
                openf_ena<=1;

                openpy_wea<=1;
                openpx_wea<=1;
                openg1_wea<=1;
                openg2_wea<=1;
                openf_wea<=1;

                openpy_addra<=open_addr1-1;
                openpx_addra<=open_addr1-1;
                openg1_addra<=open_addr1-1;
                openg2_addra<=open_addr1-1;
                openf_addra<=open_addr1-1;

                openpy_dina<=parent_node[0];
                openpx_dina<=parent_node[1];
                openg1_dina<=openg2_doutb+11;
                openg2_dina<=openg2_doutb+11;
                openf_dina<=openg2_doutb+11+openh_douta;

                open_addr1<=open_addr1+1;
                inopen_flag8 <= 1;

                opency_ena<=1;
                opencx_ena<=1;
                openg1_enb<=1;
                openg2_enb<=1;
                openh_ena<=1;

                opency_wea<=0;
                opencx_wea<=0;
                openg1_web<=0;
                openg2_web<=0;
                openh_wea<=0;

                opency_addra<=open_addr1;
                opencx_addra<=open_addr1;
                openg1_addrb<=open_addr1;
                openg2_addrb<=parent_node[2];//读取父节点的g值
                openh_addra<=open_addr1;
            end
            else begin
                opency_ena<=1;
                opencx_ena<=1;
                openg1_enb<=1;
                openg2_enb<=1;
                openh_ena<=1;

                opency_wea<=0;
                opencx_wea<=0;
                openg1_web<=0;
                openg2_web<=0;
                openh_wea<=0;

                opency_addra<=open_addr1;
                opencx_addra<=open_addr1;
                openg1_addrb<=open_addr1;
                openg2_addrb<=parent_node[2];//读取父节点的g值
                openh_addra<=open_addr1;

                inopen_flag8 <= 1;
                open_addr1<=open_addr1+1;
            end
        end
        else begin//子节点都不在当前行openlist中
            opency_ena<=1;
            opencx_ena<=1;
            openg1_enb<=1;
            openg2_enb<=1;
            openh_ena<=1;

            opency_wea<=0;
            opencx_wea<=0;
            openg1_web<=0;
            openg2_web<=0;
            openh_wea<=0;

            opency_addra<=open_addr1;
            opencx_addra<=open_addr1;
            openg1_addrb<=open_addr1;
            openg2_addrb<=parent_node[2];//读取父节点的g值
            openh_addra<=open_addr1;

            open_addr1<=open_addr1+1;
        end
    end
    else if (current_state == S5 && e20==0 && open_addr1==open_row+1) begin
        openpy_ena<=0;
        openpx_ena<=0;
        openg1_ena<=0;
        openg2_ena<=0;
        openf_ena<=0;

        openpy_wea<=0;
        openpx_wea<=0;
        openg1_wea<=0;
        openg2_wea<=0;
        openf_wea<=0;

        openpy_addra<=0;
        openpx_addra<=0;
        openg1_addra<=0;
        openg2_addra<=0;
        openf_addra<=0;

        openpy_dina<=0;
        openpx_dina<=0;
        openg1_dina<=0;
        openg2_dina<=0;
        openf_dina<=0;

        open_addr1<=0;

        opency_ena<=0;
        opencx_ena<=0;
        openg1_enb<=0;
        openg2_enb<=0;
        openh_ena<=0;

        opency_wea<=0;
        opencx_wea<=0;
        openg1_web<=0;
        openg2_web<=0;
        openh_wea<=0;

        opency_addra<=0;
        opencx_addra<=0;
        openg1_addrb<=0;
        openg2_addrb<=0;
        openh_addra<=0;
        e20<=1;
    end
    else if (current_state == S5 && e20) begin
        e20<=0;
    end
    //状态6，膜21，读取当前子节点的父节点的g值
    else if (current_state == S6 && e21==0) begin
        openg1_enb<=1;
        openg1_web<=0;
        openg1_addrb<=parent_node[2];
        e21<=1;
    end
    else if (current_state == S6 && e21) begin
        e21<=0;
    end
    //状态7，膜22，添加子节点1到openlist
    else if (current_state == S7 && e22==0) begin
        if (child_flag1 && inopen_flag1 == 0) begin//子节点不在openlist中
            openpy_ena<=1;
            openpx_ena<=1;
            opency_ena<=1;
            opencx_ena<=1;
            openg1_ena<=1;
            openg2_ena<=1;
            openh_ena<=1;
            openf_ena<=1;

            openpy_wea<=1;
            openpx_wea<=1;
            opency_wea<=1;
            opencx_wea<=1;
            openg1_wea<=1;
            openg2_wea<=1;
            openh_wea<=1;
            openf_wea<=1;

            openpy_addra<=open_row;
            openpx_addra<=open_row;
            opency_addra<=open_row;
            opencx_addra<=open_row;
            openg1_addra<=open_row;
            openg2_addra<=open_row;
            openh_addra<=open_row;
            openf_addra<=open_row;

            openpy_dina<=parent_node[0];
            openpx_dina<=parent_node[1];
            opency_dina<=child_node1[0];
            opencx_dina<=child_node1[1];
            openg1_dina<=openg1_doutb+11;
            openg2_dina<=openg1_doutb+11;
            openh_dina<=((END_ROW+END_COLUMN)-(child_node1[0]+child_node1[1]))<<3;
            openf_dina<=openg1_doutb+11+(((END_ROW+END_COLUMN)-(child_node1[0]+child_node1[1]))<<3);

            openreal_row<=openreal_row+1;
            open_row<=open_row+1;
            e22<=1; 
        end
        else begin
            inopen_flag1 <= 0;
            e22<=1;
        end
    end
    else if (current_state == S7 && e22) begin
        e22<=0;
    end
    //状态8，膜23，添加子节点2到openlist
    else if (current_state == S8 && e23==0) begin
        if (child_flag2 && inopen_flag2 == 0) begin//子节点不在openlist中
            openpy_ena<=1;
            openpx_ena<=1;
            opency_ena<=1;
            opencx_ena<=1;
            openg1_ena<=1;
            openg2_ena<=1;
            openh_ena<=1;
            openf_ena<=1;

            openpy_wea<=1;
            openpx_wea<=1;
            opency_wea<=1;
            opencx_wea<=1;
            openg1_wea<=1;
            openg2_wea<=1;
            openh_wea<=1;
            openf_wea<=1;

            openpy_addra<=open_row;
            openpx_addra<=open_row;
            opency_addra<=open_row;
            opencx_addra<=open_row;
            openg1_addra<=open_row;
            openg2_addra<=open_row;
            openh_addra<=open_row;
            openf_addra<=open_row;

            openpy_dina<=parent_node[0];
            openpx_dina<=parent_node[1];
            opency_dina<=child_node2[0];
            opencx_dina<=child_node2[1];
            openg1_dina<=openg1_doutb+8;
            openg2_dina<=openg1_doutb+8;
            openh_dina<=((END_ROW+END_COLUMN)-(child_node2[0]+child_node2[1]))<<3;
            openf_dina<=openg1_doutb+8+(((END_ROW+END_COLUMN)-(child_node2[0]+child_node2[1]))<<3);

            openreal_row<=openreal_row+1;
            open_row<=open_row+1;
            e23<=1; 
        end
        else begin
            inopen_flag2 <= 0;
            e23<=1;
        end
    end
    else if (current_state == S8 && e23) begin
        e23<=0;
    end
    //状态9，膜24，添加子节点3到openlist
    else if (current_state == S9 && e24==0) begin
         if (child_flag3 && inopen_flag3 == 0) begin//子节点不在openlist中
            openpy_ena<=1;
            openpx_ena<=1;
            opency_ena<=1;
            opencx_ena<=1;
            openg1_ena<=1;
            openg2_ena<=1;
            openh_ena<=1;
            openf_ena<=1;

            openpy_wea<=1;
            openpx_wea<=1;
            opency_wea<=1;
            opencx_wea<=1;
            openg1_wea<=1;
            openg2_wea<=1;
            openh_wea<=1;
            openf_wea<=1;

            openpy_addra<=open_row;
            openpx_addra<=open_row;
            opency_addra<=open_row;
            opencx_addra<=open_row;
            openg1_addra<=open_row;
            openg2_addra<=open_row;
            openh_addra<=open_row;
            openf_addra<=open_row;

            openpy_dina<=parent_node[0];
            openpx_dina<=parent_node[1];
            opency_dina<=child_node3[0];
            opencx_dina<=child_node3[1];
            openg1_dina<=openg1_doutb+11;
            openg2_dina<=openg1_doutb+11;
            openh_dina<=((END_ROW+END_COLUMN)-(child_node3[0]+child_node3[1]))<<3;
            openf_dina<=openg1_doutb+11+(((END_ROW+END_COLUMN)-(child_node3[0]+child_node3[1]))<<3);

            openreal_row<=openreal_row+1;
            open_row<=open_row+1;
            e24<=1;
        end
        else begin
            inopen_flag3 <= 0;
            e24<=1;
        end
    end
    else if (current_state == S9 && e24) begin
        e24<=0;
    end
    //状态10，膜25，添加子节点4到openlist
    else if (current_state == S10 && e25==0) begin
        if (child_flag4 && inopen_flag4 == 0) begin//子节点不在openlist中
            openpy_ena<=1;
            openpx_ena<=1;
            opency_ena<=1;
            opencx_ena<=1;
            openg1_ena<=1;
            openg2_ena<=1;
            openh_ena<=1;
            openf_ena<=1;

            openpy_wea<=1;
            openpx_wea<=1;
            opency_wea<=1;
            opencx_wea<=1;
            openg1_wea<=1;
            openg2_wea<=1;
            openh_wea<=1;
            openf_wea<=1;

            openpy_addra<=open_row;
            openpx_addra<=open_row;
            opency_addra<=open_row;
            opencx_addra<=open_row;
            openg1_addra<=open_row;
            openg2_addra<=open_row;
            openh_addra<=open_row;
            openf_addra<=open_row;

            openpy_dina<=parent_node[0];
            openpx_dina<=parent_node[1];
            opency_dina<=child_node4[0];
            opencx_dina<=child_node4[1];
            openg1_dina<=openg1_doutb+8;
            openg2_dina<=openg1_doutb+8;
            openh_dina<=((END_ROW+END_COLUMN)-(child_node4[0]+child_node4[1]))<<3;
            openf_dina<=openg1_doutb+8+(((END_ROW+END_COLUMN)-(child_node4[0]+child_node4[1]))<<3);

            openreal_row<=openreal_row+1;
            open_row<=open_row+1;
            e25<=1; 
        end
        else begin
            inopen_flag4 <= 0;
            e25<=1;
        end
    end
    else if (current_state == S10 && e25) begin
        e25<=0;
    end
    //状态11，膜26，添加子节点5到openlist
    else if (current_state == S11 && e26==0) begin
        if (child_flag5 && inopen_flag5 == 0) begin//子节点不在openlist中
            openpy_ena<=1;
            openpx_ena<=1;
            opency_ena<=1;
            opencx_ena<=1;
            openg1_ena<=1;
            openg2_ena<=1;
            openh_ena<=1;
            openf_ena<=1;

            openpy_wea<=1;
            openpx_wea<=1;
            opency_wea<=1;
            opencx_wea<=1;
            openg1_wea<=1;
            openg2_wea<=1;
            openh_wea<=1;
            openf_wea<=1;

            openpy_addra<=open_row;
            openpx_addra<=open_row;
            opency_addra<=open_row;
            opencx_addra<=open_row;
            openg1_addra<=open_row;
            openg2_addra<=open_row;
            openh_addra<=open_row;
            openf_addra<=open_row;

            openpy_dina<=parent_node[0];
            openpx_dina<=parent_node[1];
            opency_dina<=child_node5[0];
            opencx_dina<=child_node5[1];
            openg1_dina<=openg1_doutb+8;
            openg2_dina<=openg1_doutb+8;
            openh_dina<=((END_ROW+END_COLUMN)-(child_node5[0]+child_node5[1]))<<3;
            openf_dina<=openg1_doutb+8+(((END_ROW+END_COLUMN)-(child_node5[0]+child_node5[1]))<<3);

            openreal_row<=openreal_row+1;
            open_row<=open_row+1;
            e26<=1; 
        end
        else begin
            inopen_flag5 <= 0;
            e26<=1;
        end
    end
    else if (current_state == S11 && e26) begin
        e26<=0;
    end
     //状态12，膜27，添加子节点6到openlist
    else if (current_state == S12 && e27==0) begin
        if (child_flag6 && inopen_flag6 == 0) begin//子节点不在openlist中
            openpy_ena<=1;
            openpx_ena<=1;
            opency_ena<=1;
            opencx_ena<=1;
            openg1_ena<=1;
            openg2_ena<=1;
            openh_ena<=1;
            openf_ena<=1;

            openpy_wea<=1;
            openpx_wea<=1;
            opency_wea<=1;
            opencx_wea<=1;
            openg1_wea<=1;
            openg2_wea<=1;
            openh_wea<=1;
            openf_wea<=1;

            openpy_addra<=open_row;
            openpx_addra<=open_row;
            opency_addra<=open_row;
            opencx_addra<=open_row;
            openg1_addra<=open_row;
            openg2_addra<=open_row;
            openh_addra<=open_row;
            openf_addra<=open_row;

            openpy_dina<=parent_node[0];
            openpx_dina<=parent_node[1];
            opency_dina<=child_node6[0];
            opencx_dina<=child_node6[1];
            openg1_dina<=openg1_doutb+11;
            openg2_dina<=openg1_doutb+11;
            openh_dina<=((END_ROW+END_COLUMN)-(child_node6[0]+child_node6[1]))<<3;
            openf_dina<=openg1_doutb+11+(((END_ROW+END_COLUMN)-(child_node6[0]+child_node6[1]))<<3);

            openreal_row<=openreal_row+1;
            open_row<=open_row+1;
            e27<=1;
        end
        else begin
            inopen_flag6 <= 0;
            e27<=1;
        end
    end
    else if (current_state == S12 && e27) begin
        e27<=0;
    end
    //状态13，膜28，添加子节点7到openlist
    else if (current_state == S13 && e28==0) begin
        if (child_flag7 && inopen_flag7 == 0) begin//子节点不在openlist中
            openpy_ena<=1;
            openpx_ena<=1;
            opency_ena<=1;
            opencx_ena<=1;
            openg1_ena<=1;
            openg2_ena<=1;
            openh_ena<=1;
            openf_ena<=1;

            openpy_wea<=1;
            openpx_wea<=1;
            opency_wea<=1;
            opencx_wea<=1;
            openg1_wea<=1;
            openg2_wea<=1;
            openh_wea<=1;
            openf_wea<=1;

            openpy_addra<=open_row;
            openpx_addra<=open_row;
            opency_addra<=open_row;
            opencx_addra<=open_row;
            openg1_addra<=open_row;
            openg2_addra<=open_row;
            openh_addra<=open_row;
            openf_addra<=open_row;

            openpy_dina<=parent_node[0];
            openpx_dina<=parent_node[1];
            opency_dina<=child_node7[0];
            opencx_dina<=child_node7[1];
            openg1_dina<=openg1_doutb+8;
            openg2_dina<=openg1_doutb+8;
            openh_dina<=((END_ROW+END_COLUMN)-(child_node7[0]+child_node7[1]))<<3;
            openf_dina<=openg1_doutb+8+(((END_ROW+END_COLUMN)-(child_node7[0]+child_node7[1]))<<3);

            openreal_row<=openreal_row+1;
            open_row<=open_row+1;
            e28<=1;
        end
        else begin
            inopen_flag7 <= 0;
            e28<=1;
        end
    end
    else if (current_state == S13 && e28) begin
        e28<=0;
    end
    //状态14，膜29，添加子节点8到openlist
    else if (current_state == S14 && e29==0) begin
        if (child_flag8 && inopen_flag8 == 0) begin//子节点不在openlist中
            openpy_ena<=1;
            openpx_ena<=1;
            opency_ena<=1;
            opencx_ena<=1;
            openg1_ena<=1;
            openg2_ena<=1;
            openh_ena<=1;
            openf_ena<=1;

            openpy_wea<=1;
            openpx_wea<=1;
            opency_wea<=1;
            opencx_wea<=1;
            openg1_wea<=1;
            openg2_wea<=1;
            openh_wea<=1;
            openf_wea<=1;

            openpy_addra<=open_row;
            openpx_addra<=open_row;
            opency_addra<=open_row;
            opencx_addra<=open_row;
            openg1_addra<=open_row;
            openg2_addra<=open_row;
            openh_addra<=open_row;
            openf_addra<=open_row;

            openpy_dina<=parent_node[0];
            openpx_dina<=parent_node[1];
            opency_dina<=child_node8[0];
            opencx_dina<=child_node8[1];
            openg1_dina<=openg1_doutb+11;
            openg2_dina<=openg1_doutb+11;
            openh_dina<=((END_ROW+END_COLUMN)-(child_node8[0]+child_node8[1]))<<3;
            openf_dina<=openg1_doutb+11+(((END_ROW+END_COLUMN)-(child_node8[0]+child_node8[1]))<<3);

            openreal_row<=openreal_row+1;
            open_row<=open_row+1;
            e29<=1;
        end
        else begin
            inopen_flag8 <= 0;
            e29<=1;
        end
    end
    else if (current_state == S14 && e29) begin
        e29<=0;
    end

    //状态16，膜31，将父节点从openlist中删除,f值赋值为fff
    else if (current_state == S16 && e31==0) begin
        openf_ena<=1;
        openf_wea<=1;
        openf_addra<=parent_node[2];
        openf_dina<=12'hfff;
        openreal_row<=openreal_row-1;
        e31<=1; 
    end
    else if (current_state == S16 && e31) begin
        e31<=0;
    end

    //状态17，膜32，在openlist中找新的父节点
    else if (current_state == S17 && open_addr2 == 0 && e32==0) begin
        open_addr2<=open_addr2+1;

        opency_ena<=1;
        opencx_ena<=1;
        openf_ena<=1;

        opency_wea<=0;
        opencx_wea<=0;
        openf_wea<=0;

        opency_addra<=open_addr2;
        opencx_addra<=open_addr2;
        openf_addra<=open_addr2;
    end
    else if (current_state == S17 && open_addr2 != open_row && open_addr2!=0 && e32==0) begin
        if (openf_douta < min_f) begin
            open_addr2<=open_addr2+1;
            opency_ena<=1;
            opencx_ena<=1;
            openf_ena<=1;

            opency_wea<=0;
            opencx_wea<=0;
            openf_wea<=0;

            opency_addra<=open_addr2;
            opencx_addra<=open_addr2;
            openf_addra<=open_addr2;
        end
        else begin
            open_addr2<=open_addr2+1;
            opency_ena<=1;
            opencx_ena<=1;
            openf_ena<=1;

            opency_wea<=0;
            opencx_wea<=0;
            openf_wea<=0;

            opency_addra<=open_addr2;
            opencx_addra<=open_addr2;
            openf_addra<=open_addr2;
        end
    end
    else if (current_state == S17 && open_addr2 == open_row && e32==0) begin
        if (openf_douta < min_f) begin
            open_addr2<=0;
            opency_ena<=0;
            opencx_ena<=0;
            openf_ena<=0;

            opency_wea<=0;
            opencx_wea<=0;
            openf_wea<=0;

            opency_addra<=0;
            opencx_addra<=0;
            openf_addra<=0;
            e32<=1;
        end
        else begin
            open_addr2<=0;
            opency_ena<=0;
            opencx_ena<=0;
            openf_ena<=0;

            opency_wea<=0;
            opencx_wea<=0;
            openf_wea<=0;

            opency_addra<=0;
            opencx_addra<=0;
            openf_addra<=0;
            e32<=1;
        end
    end
    else if (current_state == S17 && e32) begin
        e32<=0;
    end

    //状态19，膜34,回溯整条路径
    else if (current_state == S19 && open_addr3==0 && e34==0) begin
        open_addr3<=open_addr3+1;
        openpy_ena<=1;
        openpx_ena<=1;
        opency_ena<=1;
        opencx_ena<=1;

        openpy_wea<=0;
        openpx_wea<=0;
        opency_wea<=0;
        opencx_wea<=0;

        openpy_addra<=open_addr3;
        openpx_addra<=open_addr3;
        opency_addra<=open_addr3;
        opencx_addra<=open_addr3;
    end
    else if (current_state == S19 && open_addr3!=open_row+1 && open_addr3!=0 && e34==0) begin
        if (opency_douta == path_row && opencx_douta == path_column)begin//在openlist中找到
            path_row <= openpy_douta;
            path_column <= openpx_douta;
            e34<=1;
            open_addr3<=0;
            openpy_ena<=0;
            openpx_ena<=0;
            opency_ena<=0;
            opencx_ena<=0;

            openpy_wea<=0;
            openpx_wea<=0;
            opency_wea<=0;
            opencx_wea<=0;

            openpy_addra<=0;
            openpx_addra<=0;
            opency_addra<=0;
            opencx_addra<=0;
        end
        else begin
            open_addr3<=open_addr3+1;
            openpy_ena<=1;
            openpx_ena<=1;
            opency_ena<=1;
            opencx_ena<=1;

            openpy_wea<=0;
            openpx_wea<=0;
            opency_wea<=0;
            opencx_wea<=0;

            openpy_addra<=open_addr3;
            openpx_addra<=open_addr3;
            opency_addra<=open_addr3;
            opencx_addra<=open_addr3;
        end
    end
    else if (current_state == S19 && e34) begin
        e34<=0;
    end


end

//判断结束
always @(posedge clk) begin
    if (!rst_n) begin
        e33<=0;
        findpath_flag<=0;
    end
    //状态18，膜33，判断算法是否结束
    else if (current_state == S18 && e33==0) begin
        if (parent_node[0] == END_ROW && parent_node[1] == END_COLUMN && openreal_row != 0) begin//父节点为终点或者h等于0
            e33<=2'd3;//算法结束并且找到最短路径，下一个状态为S17
            findpath_flag<=1;
        end
        else if (openreal_row == 0) begin//openlist为空，算法结束，没有找到路径,下一个状态为S22
            e33<=2'd2;
            findpath_flag<=0;
        end
        else begin//算法没有结束，下一个状态为S2
            e33<=2'd1;
        end
    end
    else if (current_state == S18 && e33!=0) begin
        e33<=0;
    end
end

//串口发送
always @(posedge clk) begin
    if(!rst_n) begin
        tx_data <= 0;
        tx_flag <= 0;
        tx_cnt <= 0;
        e35<=0;
    end
    //状态20，膜35
    else if (current_state == S20 && tx_done && e35==0) begin
        case (tx_cnt)
            0:begin
                tx_data <= path_row;
                tx_flag<=1;
                tx_cnt <= 2'd1;
                e35<=0;
            end
            1:begin
                tx_data <= 0;
                tx_flag <= 0;
                tx_cnt <= 2'd2;
                e35<=0;
            end
            2:begin
                tx_data <= path_column;
                tx_flag <= 1'b1;
                tx_cnt <= 2'd3;
                e35<=0;
            end
            3:begin
                tx_data <= 0;
                tx_flag <= 0;
                tx_cnt <= 0;
                e35<=1;
            end
        endcase
    end
    else if (current_state == S20 && e35) begin
        e35<=0;
    end
end

//判断路径坐标是否发送完
always @(posedge clk) begin
    if (!rst_n) begin
        e36<=0;
    end
    //状态21，膜36
    else if (current_state == S21 && path_row==START_ROW && path_column==START_COLUMN && e36==0) begin//回溯到起点，路径发送完，跳转到S22
        e36<=2;
    end
    else if (current_state == S21 && (path_row!=START_ROW || path_column!=START_COLUMN) && e36==0) begin//没有回溯到起点，路径发送未完成，跳转到S17
        e36<=1;
    end
    else if (current_state == S21 && e36!=0) begin
        e36<=0;
    end
end

//例化ILA模块
ila_astar ila_astar_i(
    .clk(clk_ila),
    .probe0(led0),//1
    .probe1(findpath_flag)//1
);

endmodule
