clc
clear
close all

%% ���ɵ�ͼ

% դ���ͼ����������������
m = 127;%��y
n = 127;%��x
start_node = [2, 2];%���
target_node = [m-1, n-1];%�յ�
maze=zeros(m,n);
maze(1,:)=1;
maze(:,1)=1;
maze(m,:)=1;
maze(:,n)=1;
for i=1:4096
    x=randi(floor(n/2))*2+1;
    y=randi(floor(m/2))*2+1;
    maze(y,x)=1;
    for j=1:100
        neighbours=[];
        if x>2%3
            neighbours = [neighbours;[y,x-2]];
        end
        if x<n-2%123
            neighbours = [neighbours;[y,x+2]];
        end
        if y>2
            neighbours = [neighbours;[y-2,x]];
        end
        if y<m-2
            neighbours = [neighbours;[y+2,x]];
        end
        if size(neighbours,1)>0
            xy=neighbours(randi(size(neighbours,1)),:);
            y1=xy(1);
            x1=xy(2);
            if y1>0 && x1>0
                if maze(y1,x1)==0
                    maze(y1,x1)=1;
                    maze(y1-floor((y-y1)/2),x1-floor((x-x1)/2))=1;
                    x=x1;
                    y=y1;
                end
            end
        end
    end
end
obs=[];
for i=1:m
    for j=1:n
        if maze(i,j)==1
            obs=[obs;[i,j]];
        end
    end
end


%% Ԥ����

% ��ʼ��closeList
closeList = start_node;
closeList_path = {start_node,start_node};
closeList_cost = 0;
child_nodes = child_nodes_cal(start_node, obs, closeList); %�ӽڵ��������� 

% ��ʼ��openList
openList = child_nodes;
for i = 1:size(openList,1)
    openList_path{i,1} = openList(i,:);
    openList_path{i,2} = [start_node;openList(i,:)];%�ӳ�ʼ�㵽��i���ӽڵ�
end

for i = 1:size(openList, 1)
    g = norm(start_node - openList(i,1:2));%norm�����������������ֵ��abs�����ֵ
    h = abs(target_node(1) - openList(i,1)) + abs(target_node(2) - openList(i,2));
    %�յ�������������������
    f = g + h;
    openList_cost(i,:) = [g, h, f];
end

%% ��ʼ����
% ��openList��ʼ�����ƶ�������С�Ľڵ�
[~, min_idx] = min(openList_cost(:,3));%���openlist_cost������Сֵ��λ��
parent_node = openList(min_idx,:);%���ڵ�Ϊ������С�ڵ�


%% ����ѭ��
flag = 1;
while flag   
    
    % �ҳ����ڵ�ĺ���closeList���ӽڵ�
    child_nodes = child_nodes_cal(parent_node, obs, closeList); 
    
    % �ж���Щ�ӽڵ��Ƿ���openList�У����ڣ���Ƚϸ��£�û����׷�ӵ�openList��
    for i = 1:size(child_nodes,1)
        child_node = child_nodes(i,:);
        [in_flag,openList_idx] = ismember(child_node, openList, 'rows');%ismember������ʾ�ӽڵ���open�����򷵻�1���ж�flag,������ӽڵ���openlist���е�λ��
        g = openList_cost(min_idx, 1) + norm(parent_node - child_node);%�����¸��ڵ������ӽڵ��g,hֵ
        h = abs(child_node(1) - target_node(1)) + abs(child_node(2) - target_node(2));
        f = g+h;
        
        if in_flag   % ���ڣ��Ƚϸ���g��f        
            if g < openList_cost(openList_idx,1)
                openList_cost(openList_idx, 1) = g;%��openlist_cost���е�id��λ�õĵ�һ��������Ϊ���¸��ڵ�����gֵ
                openList_cost(openList_idx, 3) = f;
                openList_path{openList_idx,2} = [openList_path{min_idx,2}; child_node];
            end
        else         % �����ڣ�׷�ӵ�openList
            openList(end+1,:) = child_node;
            openList_cost(end+1, :) = [g, h, f];
            openList_path{end+1, 1} = child_node;
            openList_path{end, 2} = [openList_path{min_idx,2}; child_node];
        end
    end
   
   
    % ��openList�Ƴ��ƶ�������С�Ľڵ㵽 closeList
    closeList(end+1,: ) =  openList(min_idx,:);
    closeList_cost(end+1,1) =   openList_cost(min_idx,3);
    closeList_path(end+1,:) = openList_path(min_idx,:);
    openList(min_idx,:) = [];%openlist��������������Сֵλ����Ϊ��
    openList_cost(min_idx,:) = [];
    openList_path(min_idx,:) = [];
 
     % ������������openList�����ƶ�������С�Ľڵ㣨�ظ����裩
    [~, min_idx] = min(openList_cost(:,3));
    parent_node = openList(min_idx,:);
    
    
    % �ж��Ƿ��������յ�
    if parent_node == target_node
        closeList(end+1,: ) =  openList(min_idx,:);
        closeList_cost(end+1,1) =   openList_cost(min_idx,1);
        closeList_path(end+1,:) = openList_path(min_idx,:);
        flag = 0;
    end
end
    
    
%% ��ͼ
%��դ�����
for i = 0:m
    plot([0,n], [i, i], 'k');
    hold on
end
 %��դ������   
for j = 0:n
     plot([j, j], [0, m], 'k');
end

axis equal%�ȱ������ᣬʹ��ÿ�������ᶼ���о��ȵĿ̶ȼ��
xlim([0, n]);
ylim([0, m]); %xy��������  

% �����ϰ����ֹ����ɫ��
fill([start_node(1)-1, start_node(1), start_node(1), start_node(1)-1],...
    [start_node(2)-1, start_node(2)-1 , start_node(2), start_node(2)], 'b');

fill([target_node(1)-1, target_node(1), target_node(1), target_node(1)-1],...
    [target_node(2)-1, target_node(2)-1 , target_node(2), target_node(2)], 'r');

for i = 1:size(obs,1)%���ؾ�������
    temp = obs(i,:);
    fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
        [temp(2)-1, temp(2)-1 , temp(2), temp(2)], 'k');
end

for i = 1:size(openList,1)%���ؾ�������
    temp = openList(i,:);
    fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
        [temp(2)-1, temp(2)-1 , temp(2), temp(2)], 'y');
end
for i = 1:size(closeList,1)%���ؾ�������
    temp = closeList(i,:);
    fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
        [temp(2)-1, temp(2)-1 , temp(2), temp(2)], 'g');
end
for i = 1:size(closeList_path{end,2},1)%���ؾ�������
    temp = closeList_path{end,2}(i,:);
    fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
        [temp(2)-1, temp(2)-1 , temp(2), temp(2)], 'b');
end
% path_opt = closeList_path{end,2};
% path_opt(:,1) = path_opt(:,1)-0.5;
% path_opt(:,2) = path_opt(:,2)-0.5;
% scatter(path_opt(:,1), path_opt(:,2), 'r');%����ɢ��ͼ
% plot(path_opt(:,1), path_opt(:,2), 'r');

% �����ϰ����ֹ����ɫ��
fill([start_node(1)-1, start_node(1), start_node(1), start_node(1)-1],...
    [start_node(2)-1, start_node(2)-1 , start_node(2), start_node(2)], 'b');

fill([target_node(1)-1, target_node(1), target_node(1), target_node(1)-1],...
    [target_node(2)-1, target_node(2)-1 , target_node(2), target_node(2)], 'r');



% for i = 1:size(closeList,1)%���ؾ�������
%     temp = closeList(i,:);
%     fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
%         [temp(2)-1, temp(2)-1 , temp(2), temp(2)], 'r');
% end


function child_nodes = child_nodes_cal(parent_node, obs, closeList)

child_nodes = [];

% ��1���ӽڵ�
child_node = [parent_node(1)-1, parent_node(2)+1];
if ~ismember(child_node, obs, 'rows')
    child_nodes = [child_nodes; child_node];
end

% ��2���ӽڵ�
child_node = [parent_node(1), parent_node(2)+1];
if ~ismember(child_node, obs, 'rows')
    child_nodes = [child_nodes; child_node];
end

% ��3���ӽڵ�
child_node = [parent_node(1)+1, parent_node(2)+1];
if ~ismember(child_node, obs, 'rows')
    child_nodes = [child_nodes; child_node];
end


% ��4���ӽڵ�
child_node = [parent_node(1)-1, parent_node(2)];
if ~ismember(child_node, obs, 'rows')
    child_nodes = [child_nodes; child_node];
end

% ��5���ӽڵ�
child_node = [parent_node(1)+1, parent_node(2)];
if ~ismember(child_node, obs, 'rows')
    child_nodes = [child_nodes; child_node];
end

% ��6���ӽڵ�
child_node = [parent_node(1)-1, parent_node(2)-1];
if ~ismember(child_node, obs, 'rows')
    child_nodes = [child_nodes; child_node];
end

% ��7���ӽڵ�
child_node = [parent_node(1), parent_node(2)-1];
if ~ismember(child_node, obs, 'rows')
    child_nodes = [child_nodes; child_node];
end

% ��8���ӽڵ�
child_node = [parent_node(1)+1, parent_node(2)-1];
if ~ismember(child_node, obs, 'rows')
    child_nodes = [child_nodes; child_node];
end

%% �ų��Ѿ�������closeList�Ľڵ�
delete_idx = [];
for i = 1:size(child_nodes, 1)
    if ismember(child_nodes(i,:), closeList , 'rows')
        delete_idx(end+1,:) = i;
    end
end
child_nodes(delete_idx, :) = [];
end
  