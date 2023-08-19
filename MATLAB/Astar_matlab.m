clc
clear
close all

%% ��������ϰ����ͼ

% դ���ͼ����������������
row = 15;%��y
column = 15;%��x
start_point = [2, 2];%���
terminal_point = [row-1, column-1];%�յ�
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
%% Ԥ����

% ��ʼ��closeList
closeList = [start_point,start_point];
parent_node=start_point;
child_nodes = child_nodes_cal(parent_node, map_array, closeList); %�ӽڵ��������� 

% ��ʼ��openList
% openList = child_nodes;
for i = 1:size(child_nodes,1)
    openList(i,1:2) = parent_node;
    openList(i,3:4) = child_nodes(i,:);
%     openList_path{i,2} = [start_point;openList(i,:)];%�ӳ�ʼ�㵽��i���ӽڵ�
    buff = parent_node - openList(i,1:2);
    if buff(1)==0 || buff(2)==0
        g=8;
    else
        g=11;
    end
    h = (abs(terminal_point(1) - openList(i,1)) + abs(terminal_point(2) - openList(i,2)))*8;
    %�յ�������������������
    f = g + h;
    openList(i,5:7) = [g, h, f];
end

% for i = 1:size(openList, 1)
%      %g = norm(start_point - openList(i,1:2));%norm�����������������ֵ��abs�����ֵ
%     buff = start_point - openList(i,1:2);
%     if buff(1)==0 || buff(2)==0
%         g=10;
%     else
%         g=14;
%     end
% %     h = abs(terminal_point(1) - openList(i,1)) + abs(terminal_point(2) - openList(i,2));
%     h = (abs(terminal_point(1) - openList(i,1)) + abs(terminal_point(2) - openList(i,2)))*10;
%     %�յ�������������������
%     f = g + h;
%     openList_cost(i,:) = [g, h, f];
% end

%% ��ʼ����
% ��openList��ʼ�����ƶ�������С�Ľڵ�
[~, min_idx] = min(openList(:,7));%���openlist������Сֵ��λ��
parent_node = openList(min_idx,3:4);%���ڵ�Ϊ������С�ڵ�


%% ����ѭ��
flag = 1;
while flag   
    
    % �ҳ����ڵ�ĺ���closeList���ӽڵ�
    child_nodes = child_nodes_cal(parent_node, map_array, closeList); 
    
    % �ж���Щ�ӽڵ��Ƿ���openList�У����ڣ���Ƚϸ��£�û����׷�ӵ�openList��
    for i = 1:size(child_nodes,1)
        child_node = child_nodes(i,:);
        
        %g = openList_cost(min_idx, 1) + norm(parent_node - child_node);%�����¸��ڵ������ӽڵ��g,hֵ
        buff = parent_node - child_node;
        if buff(1)==0 || buff(2)==0
            g=8;
        else
            g=11;
        end
        g = openList(min_idx, 5) + g;%�����¸��ڵ������ӽڵ��g,hֵ
%         h = abs(child_node(1) - terminal_point(1)) + abs(child_node(2) - terminal_point(2));
        h = (abs(child_node(1) - terminal_point(1)) + abs(child_node(2) - terminal_point(2)))*8;
        f = g+h;
        [in_flag,openList_idx] = ismember(child_node, openList(:,3:4), 'rows');%ismember������ʾ�ӽڵ���open�����򷵻�1���ж�flag,������ӽڵ���openlist���е�λ��
        if in_flag   % ���ڣ��Ƚϸ���g��f        
            if g < openList(openList_idx,5)
                openList(openList_idx, 5) = g;%��openlist_cost���е�id��λ�õĵ�һ��������Ϊ���¸��ڵ�����gֵ
                openList(openList_idx, 7) = f;
                openList(openList_idx,1:2) = parent_node;
            end
        else         % �����ڣ�׷�ӵ�openList
            openList(end+1,1:2) = parent_node;
            openList(end,3:4) = child_node;
            openList(end, 5:7) = [g, h, f];
        end
    end
   
   
    % ��openList�Ƴ��ƶ�������С�Ľڵ㵽 closeList
    closeList(end+1,: ) =  openList(min_idx,1:4);
    openList(min_idx,:) = [];%openlist��������������Сֵλ����Ϊ��
%     openList_cost(min_idx,:) = [];
%     openList_path(min_idx,:) = [];
 
     % ������������openList�����ƶ�������С�Ľڵ㣨�ظ����裩
    [~, min_idx] = min(openList(:,7));
    parent_node = openList(min_idx,3:4);
    
    
    % �ж��Ƿ��������յ�
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
%�ҵ�·���Ż�·��
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
%% ��ͼ
%��դ�����
for i = 0:row
    plot([0,column], [i, i], 'k');
    hold on
end
 %��դ������   
for j = 0:column
     plot([j, j], [0, row], 'k');
end

axis equal%�ȱ������ᣬʹ��ÿ�������ᶼ���о��ȵĿ̶ȼ��
xlim([0, column]);
ylim([0, row]); %xy��������  

% �����ϰ����ֹ����ɫ��
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

% for i = 1:size(map_coords,1)%���ؾ�������
%     temp = map_coords(i,:);
%     fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
%         [temp(2)-1, temp(2)-1 , temp(2), temp(2)], 'k');
% end

for i = 1:size(openList,1)%���ؾ�������
    temp = openList(i,3:4);
    fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
        [temp(2)-1, temp(2)-1 , temp(2), temp(2)], 'y');
end

for i = 1:size(closeList,1)%���ؾ�������
    temp = closeList(i,3:4);
    fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
        [temp(2)-1, temp(2)-1 , temp(2), temp(2)], 'g');
end

if path_flag==1
%     for i = 1:size(path,1)%���ؾ�������
%         temp = path(i,:);
%         fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
%             [temp(2)-1, temp(2)-1 , temp(2), temp(2)], 'b');
%     end
       path(:,1) = path(:,1)-0.5;
    path(:,2) = path(:,2)-0.5;
    %scatter(path(:,1), path(:,2), 'r');%����ɢ��ͼ
    plot(path(:,1), path(:,2),'r','LineWidth',2);
end


% for i = 1:size(closeList_path{end,2},1)%���ؾ�������
%     temp = closeList_path{end,2}(i,:);
%     fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
%         [temp(2)-1, temp(2)-1 , temp(2), temp(2)], 'b');
% end
% path_opt = closeList_path{end,2};
% path_opt(:,1) = path_opt(:,1)-0.5;
% path_opt(:,2) = path_opt(:,2)-0.5;
% scatter(path_opt(:,1), path_opt(:,2), 'r');%����ɢ��ͼ
% plot(path_opt(:,1), path_opt(:,2), 'r');

% �����ϰ����ֹ����ɫ��
fill([start_point(1)-1, start_point(1), start_point(1), start_point(1)-1],...
    [start_point(2)-1, start_point(2)-1 , start_point(2), start_point(2)], 'b');

fill([terminal_point(1)-1, terminal_point(1), terminal_point(1), terminal_point(1)-1],...
    [terminal_point(2)-1, terminal_point(2)-1 , terminal_point(2), terminal_point(2)], 'r');



% for i = 1:size(closeList,1)%���ؾ�������
%     temp = closeList(i,:);
%     fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
%         [temp(2)-1, temp(2)-1 , temp(2), temp(2)], 'r');
% end


function child_nodes = child_nodes_cal(parent_node, map_array, closeList)

child_nodes = [];

% ��1���ӽڵ�
child_node = [parent_node(1)-1, parent_node(2)+1];
if map_array(child_node(1),child_node(2)) == 0
    child_nodes = [child_nodes; child_node];
end
% if ~ismember(child_node, map_coords, 'rows')
%     child_nodes = [child_nodes; child_node];
% end

% ��2���ӽڵ�
child_node = [parent_node(1), parent_node(2)+1];
if map_array(child_node(1),child_node(2)) == 0
    child_nodes = [child_nodes; child_node];
end

% ��3���ӽڵ�
child_node = [parent_node(1)+1, parent_node(2)+1];
if map_array(child_node(1),child_node(2)) == 0
    child_nodes = [child_nodes; child_node];
end


% ��4���ӽڵ�
child_node = [parent_node(1)-1, parent_node(2)];
if map_array(child_node(1),child_node(2)) == 0
    child_nodes = [child_nodes; child_node];
end

% ��5���ӽڵ�
child_node = [parent_node(1)+1, parent_node(2)];
if map_array(child_node(1),child_node(2)) == 0
    child_nodes = [child_nodes; child_node];
end

% ��6���ӽڵ�
child_node = [parent_node(1)-1, parent_node(2)-1];
if map_array(child_node(1),child_node(2)) == 0
    child_nodes = [child_nodes; child_node];
end

% ��7���ӽڵ�
child_node = [parent_node(1), parent_node(2)-1];
if map_array(child_node(1),child_node(2)) == 0
    child_nodes = [child_nodes; child_node];
end

% ��8���ӽڵ�
child_node = [parent_node(1)+1, parent_node(2)-1];
if map_array(child_node(1),child_node(2)) == 0
    child_nodes = [child_nodes; child_node];
end

%% �ų��Ѿ�������closeList�Ľڵ�
delete_idx = [];
for i = 1:size(child_nodes, 1)
    if ismember(child_nodes(i,:), closeList(:,3:4) , 'rows')
        delete_idx(end+1,:) = i;
    end
end
child_nodes(delete_idx, :) = [];
end
