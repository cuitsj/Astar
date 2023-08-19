clc
clear
close all

%% 生成随机障碍物地图

% 栅格地图的行数、列数定义
row = 15;%行y
column = 15;%列x
start_point = [2, 2];%起点
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

tic
%% 预处理

% 初始化closeList
closeList = [start_point,start_point];
parent_node=start_point;
child_nodes = child_nodes_cal(parent_node, map_array, closeList); %子节点搜索函数 

% 初始化openList
% openList = child_nodes;
for i = 1:size(child_nodes,1)
    openList(i,1:2) = parent_node;
    openList(i,3:4) = child_nodes(i,:);
%     openList_path{i,2} = [start_point;openList(i,:)];%从初始点到第i个子节点
    buff = parent_node - openList(i,1:2);
    if buff(1)==0 || buff(2)==0
        g=8;
    else
        g=11;
    end
    h = (abs(terminal_point(1) - openList(i,1)) + abs(terminal_point(2) - openList(i,2)))*8;
    %终点横坐标距离加纵坐标距离
    f = g + h;
    openList(i,5:7) = [g, h, f];
end

% for i = 1:size(openList, 1)
%      %g = norm(start_point - openList(i,1:2));%norm求范数，返回最大奇异值；abs求绝对值
%     buff = start_point - openList(i,1:2);
%     if buff(1)==0 || buff(2)==0
%         g=10;
%     else
%         g=14;
%     end
% %     h = abs(terminal_point(1) - openList(i,1)) + abs(terminal_point(2) - openList(i,2));
%     h = (abs(terminal_point(1) - openList(i,1)) + abs(terminal_point(2) - openList(i,2)))*10;
%     %终点横坐标距离加纵坐标距离
%     f = g + h;
%     openList_cost(i,:) = [g, h, f];
% end

%% 开始搜索
% 从openList开始搜索移动代价最小的节点
[~, min_idx] = min(openList(:,7));%输出openlist表中最小值的位置
parent_node = openList(min_idx,3:4);%父节点为代价最小节点


%% 进入循环
flag = 1;
while flag   
    
    % 找出父节点的忽略closeList的子节点
    child_nodes = child_nodes_cal(parent_node, map_array, closeList); 
    
    % 判断这些子节点是否在openList中，若在，则比较更新；没在则追加到openList中
    for i = 1:size(child_nodes,1)
        child_node = child_nodes(i,:);
        
        %g = openList_cost(min_idx, 1) + norm(parent_node - child_node);%按照新父节点计算此子节点的g,h值
        buff = parent_node - child_node;
        if buff(1)==0 || buff(2)==0
            g=8;
        else
            g=11;
        end
        g = openList(min_idx, 5) + g;%按照新父节点计算此子节点的g,h值
%         h = abs(child_node(1) - terminal_point(1)) + abs(child_node(2) - terminal_point(2));
        h = (abs(child_node(1) - terminal_point(1)) + abs(child_node(2) - terminal_point(2)))*8;
        f = g+h;
        [in_flag,openList_idx] = ismember(child_node, openList(:,3:4), 'rows');%ismember函数表示子节点在open表中则返回1，判断flag,输出此子节点在openlist表中的位置
        if in_flag   % 若在，比较更新g和f        
            if g < openList(openList_idx,5)
                openList(openList_idx, 5) = g;%将openlist_cost表中第id个位置的第一个数更新为以新父节点计算的g值
                openList(openList_idx, 7) = f;
                openList(openList_idx,1:2) = parent_node;
            end
        else         % 若不在，追加到openList
            openList(end+1,1:2) = parent_node;
            openList(end,3:4) = child_node;
            openList(end, 5:7) = [g, h, f];
        end
    end
   
   
    % 从openList移除移动代价最小的节点到 closeList
    closeList(end+1,: ) =  openList(min_idx,1:4);
    openList(min_idx,:) = [];%openlist表中已跳出的最小值位置设为空
%     openList_cost(min_idx,:) = [];
%     openList_path(min_idx,:) = [];
 
     % 重新搜索：从openList搜索移动代价最小的节点（重复步骤）
    [~, min_idx] = min(openList(:,7));
    parent_node = openList(min_idx,3:4);
    
    
    % 判断是否搜索到终点
    if parent_node == terminal_point
       % closeList(end+1,: ) =  openList(min_idx,:);
%         closeList_path(end+1,:) = openList_path(min_idx,:);
        flag = 0;
        path_flag = 1;
    elseif size(openList,1) == 0
        flag = 0;
        path_flag = 0;
        disp('Path finding failed!');
    end
end
%找到路径才画路径
if path_flag == 1
    path=[parent_node];
    while parent_node(1) ~= start_point(1) || parent_node(2) ~= start_point(2)
        [inopen_flag,openList_idx] = ismember(parent_node, openList(:,3:4), 'rows');
        [inclose_flag,closeList_idx] = ismember(parent_node, closeList(:,3:4), 'rows');
        if inopen_flag
            parent_node = openList(openList_idx,1:2);
            path(end+1,:)=openList(openList_idx,1:2);
        elseif inclose_flag
            parent_node = closeList(closeList_idx,1:2);
            path(end+1,:)=closeList(closeList_idx,1:2);
        end
    end
end
toc   
%% 画图
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
        if map_array(i,j)==1
            fill([i-1, i, i, i-1],...
            [j-1, j-1 , j, j], 'k');
        end
    end
end

% for i = 1:size(map_coords,1)%返回矩阵行数
%     temp = map_coords(i,:);
%     fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
%         [temp(2)-1, temp(2)-1 , temp(2), temp(2)], 'k');
% end

for i = 1:size(openList,1)%返回矩阵行数
    temp = openList(i,3:4);
    fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
        [temp(2)-1, temp(2)-1 , temp(2), temp(2)], 'y');
end

for i = 1:size(closeList,1)%返回矩阵行数
    temp = closeList(i,3:4);
    fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
        [temp(2)-1, temp(2)-1 , temp(2), temp(2)], 'g');
end

if path_flag==1
%     for i = 1:size(path,1)%返回矩阵行数
%         temp = path(i,:);
%         fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
%             [temp(2)-1, temp(2)-1 , temp(2), temp(2)], 'b');
%     end
       path(:,1) = path(:,1)-0.5;
    path(:,2) = path(:,2)-0.5;
    %scatter(path(:,1), path(:,2), 'r');%绘制散点图
    plot(path(:,1), path(:,2),'r','LineWidth',2);
end


% for i = 1:size(closeList_path{end,2},1)%返回矩阵行数
%     temp = closeList_path{end,2}(i,:);
%     fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
%         [temp(2)-1, temp(2)-1 , temp(2), temp(2)], 'b');
% end
% path_opt = closeList_path{end,2};
% path_opt(:,1) = path_opt(:,1)-0.5;
% path_opt(:,2) = path_opt(:,2)-0.5;
% scatter(path_opt(:,1), path_opt(:,2), 'r');%绘制散点图
% plot(path_opt(:,1), path_opt(:,2), 'r');

% 绘制障碍物、起止点颜色块
fill([start_point(1)-1, start_point(1), start_point(1), start_point(1)-1],...
    [start_point(2)-1, start_point(2)-1 , start_point(2), start_point(2)], 'b');

fill([terminal_point(1)-1, terminal_point(1), terminal_point(1), terminal_point(1)-1],...
    [terminal_point(2)-1, terminal_point(2)-1 , terminal_point(2), terminal_point(2)], 'r');



% for i = 1:size(closeList,1)%返回矩阵行数
%     temp = closeList(i,:);
%     fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
%         [temp(2)-1, temp(2)-1 , temp(2), temp(2)], 'r');
% end


function child_nodes = child_nodes_cal(parent_node, map_array, closeList)

child_nodes = [];

% 第1个子节点
child_node = [parent_node(1)-1, parent_node(2)+1];
if map_array(child_node(1),child_node(2)) == 0
    child_nodes = [child_nodes; child_node];
end
% if ~ismember(child_node, map_coords, 'rows')
%     child_nodes = [child_nodes; child_node];
% end

% 第2个子节点
child_node = [parent_node(1), parent_node(2)+1];
if map_array(child_node(1),child_node(2)) == 0
    child_nodes = [child_nodes; child_node];
end

% 第3个子节点
child_node = [parent_node(1)+1, parent_node(2)+1];
if map_array(child_node(1),child_node(2)) == 0
    child_nodes = [child_nodes; child_node];
end


% 第4个子节点
child_node = [parent_node(1)-1, parent_node(2)];
if map_array(child_node(1),child_node(2)) == 0
    child_nodes = [child_nodes; child_node];
end

% 第5个子节点
child_node = [parent_node(1)+1, parent_node(2)];
if map_array(child_node(1),child_node(2)) == 0
    child_nodes = [child_nodes; child_node];
end

% 第6个子节点
child_node = [parent_node(1)-1, parent_node(2)-1];
if map_array(child_node(1),child_node(2)) == 0
    child_nodes = [child_nodes; child_node];
end

% 第7个子节点
child_node = [parent_node(1), parent_node(2)-1];
if map_array(child_node(1),child_node(2)) == 0
    child_nodes = [child_nodes; child_node];
end

% 第8个子节点
child_node = [parent_node(1)+1, parent_node(2)-1];
if map_array(child_node(1),child_node(2)) == 0
    child_nodes = [child_nodes; child_node];
end

%% 排除已经存在于closeList的节点
delete_idx = [];
for i = 1:size(child_nodes, 1)
    if ismember(child_nodes(i,:), closeList(:,3:4) , 'rows')
        delete_idx(end+1,:) = i;
    end
end
child_nodes(delete_idx, :) = [];
end
