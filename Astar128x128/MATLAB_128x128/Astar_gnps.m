clc
clear
close all

%% 连接串口
delete(instrfindall) %%关闭没用的，这句很重要 
scom = serial('COM15');
scom.InputBufferSize =8000;
scom.OutputBufferSize =8000;
scom.ReadAsyncMode='continuous';
scom.BaudRate =115200;
scom.Parity ='none';
scom.StopBits =1;
scom.DataBits =8;
scom.Terminator ='CR';
scom.FlowControl ='none';
scom.timeout =10;
scom.BytesAvailableFcnMode = 'byte';
scom.BytesAvailableFcnCount = 8000;

try
    fopen(scom);
    fprintf(1,'串口打开成功\n'); 
catch
     fprintf(1,'串口打开失败\n'); 
     msgbox('串口打开失败!','Error','error');
end
    
%% 生成随机障碍物地图

% 栅格地图的行数、列数定义
row = 127;%行y
column = 127;%列x
start_point = [2, 2];%起点
terminal_point_FPGA = [row-2, column-2];%终点
terminal_point = [row-1, column-1];%终点
map_array=zeros(row+1,column+1);
map_array(1,:)=1;
map_array(:,1)=1;
map_array(row,:)=1;
map_array(:,column)=1;
for i=1:4096
    x=randi(floor(column/2))*2+1;
    y=randi(floor(row/2))*2+1;
    map_array(y,x)=1;
    for j=1:100
        neighbours=[];
        if x>2%3
            neighbours = [neighbours;[y,x-2]];
        end
        if x<column-2%123
            neighbours = [neighbours;[y,x+2]];
        end
        if y>2
            neighbours = [neighbours;[y-2,x]];
        end
        if y<row-2
            neighbours = [neighbours;[y+2,x]];
        end
        if size(neighbours,1)>0
            xy=neighbours(randi(size(neighbours,1)),:);
            y1=xy(1);
            x1=xy(2);
            if y1>0 && x1>0
                if map_array(y1,x1)==0
                    map_array(y1,x1)=1;
                    map_array(y1-floor((y-y1)/2),x1-floor((x-x1)/2))=1;
                    x=x1;
                    y=y1;
                end
            end
        end
    end
end

%% 保存地图
% map=fopen('map.txt','wt');%写入文件路径
% [m,n]=size(map_array);
% for i=1:1:m
%     for j=1:1:n
%         if j==n
%             fprintf(map,'%d\n',map_array(i,j));
%         else
%             fprintf(map,'%d ',map_array(i,j));
%         end
%     end
% end
% fclose(map);

%% 读地图
[m,n]=size(map_array);
map_txt = textread('map.txt', '%d', -1);
for i=1:1:m
    for j=1:1:n
        map_read(i,j) = map_txt((i-1)*m+j);
    end
end


%% 发送地图
for i=1:row+1
    for j=1:8:column+1
        fwrite(scom,(map_read(i,j)*1+map_read(i,j+1)*2+map_read(i,j+2)*4+map_read(i,j+3)*8+map_read(i,j+4)*16+map_read(i,j+5)*32+map_read(i,j+6)*64+map_read(i,j+7)*128)); % 以二进制形式向串口对象写入数据A
%          fprintf('%s\n',dec2hex((map_array(i,j)*1+map_array(i,j+1)*2+map_array(i,j+2)*4+map_array(i,j+3)*8+map_array(i,j+4)*16+map_array(i,j+5)*32+map_array(i,j+6)*64+map_array(i,j+7)*128))); 
    end
end


%% 画地图
%画栅格横线
for i = 0:row
    plot([0,column], [i, i], 'k');
    hold on
end
 %画栅格竖线   
for j = 0:column
     plot([j, j], [0, row], 'k');
end

axis equal%等比坐标轴，使得每个坐标轴都具有均匀的刻度间隔
xlim([0, column]);
ylim([0, row]); %xy轴上下限  

% 绘制障碍物、起止点颜色块
fill([start_point(1)-1, start_point(1), start_point(1), start_point(1)-1],...
    [start_point(2)-1, start_point(2)-1 , start_point(2), start_point(2)], 'b');

fill([terminal_point(1)-1, terminal_point(1), terminal_point(1), terminal_point(1)-1],...
    [terminal_point(2)-1, terminal_point(2)-1 , terminal_point(2), terminal_point(2)], 'r');

for i=1:row
    for j=1:column
        if map_read(i,j)==1
            fill([i-1, i, i, i-1],...
            [j-1, j-1 , j, j], 'k');
        end
    end
end

%% 接收路径
path = terminal_point_FPGA;
while (path(end,1)+1~=start_point(1,1)||path(end,2)+1~=start_point(1,2))
    path(end+1,1:2) = fread(scom,[1 2],'uint8');
end

%% 画路径
path(:,1) = path(:,1)+1;
path(:,2) = path(:,2)+1;
for i = 1:size(path,1)%返回矩阵行数
    temp = path(i,1:2);
    fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
        [temp(2)-1, temp(2)-1 , temp(2), temp(2)], 'g');
end

path(:,1) = path(:,1)-0.5;
path(:,2) = path(:,2)-0.5;
plot(path(:,1), path(:,2),'b','LineWidth',2);

% 绘制障碍物、起止点颜色块
fill([start_point(1)-1, start_point(1), start_point(1), start_point(1)-1],...
[start_point(2)-1, start_point(2)-1 , start_point(2), start_point(2)], 'b');

fill([terminal_point(1)-1, terminal_point(1), terminal_point(1), terminal_point(1)-1],...
[terminal_point(2)-1, terminal_point(2)-1 , terminal_point(2), terminal_point(2)], 'r');

