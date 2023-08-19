clc
clear
close all

%% ���Ӵ���
delete(instrfindall) %%�ر�û�õģ�������Ҫ 
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
    fprintf(1,'���ڴ򿪳ɹ�\n'); 
catch
     fprintf(1,'���ڴ�ʧ��\n'); 
     msgbox('���ڴ�ʧ��!','Error','error');
end
    
%% ��������ϰ����ͼ

% դ���ͼ����������������
row = 127;%��y
column = 127;%��x
start_point = [2, 2];%���
terminal_point_FPGA = [row-2, column-2];%�յ�
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

%% �����ͼ
% map=fopen('map.txt','wt');%д���ļ�·��
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

%% ����ͼ
[m,n]=size(map_array);
map_txt = textread('map.txt', '%d', -1);
for i=1:1:m
    for j=1:1:n
        map_read(i,j) = map_txt((i-1)*m+j);
    end
end


%% ���͵�ͼ
for i=1:row+1
    for j=1:8:column+1
        fwrite(scom,(map_read(i,j)*1+map_read(i,j+1)*2+map_read(i,j+2)*4+map_read(i,j+3)*8+map_read(i,j+4)*16+map_read(i,j+5)*32+map_read(i,j+6)*64+map_read(i,j+7)*128)); % �Զ�������ʽ�򴮿ڶ���д������A
%          fprintf('%s\n',dec2hex((map_array(i,j)*1+map_array(i,j+1)*2+map_array(i,j+2)*4+map_array(i,j+3)*8+map_array(i,j+4)*16+map_array(i,j+5)*32+map_array(i,j+6)*64+map_array(i,j+7)*128))); 
    end
end


%% ����ͼ
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
        if map_read(i,j)==1
            fill([i-1, i, i, i-1],...
            [j-1, j-1 , j, j], 'k');
        end
    end
end

%% ����·��
path = terminal_point_FPGA;
while (path(end,1)+1~=start_point(1,1)||path(end,2)+1~=start_point(1,2))
    path(end+1,1:2) = fread(scom,[1 2],'uint8');
end

%% ��·��
path(:,1) = path(:,1)+1;
path(:,2) = path(:,2)+1;
for i = 1:size(path,1)%���ؾ�������
    temp = path(i,1:2);
    fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
        [temp(2)-1, temp(2)-1 , temp(2), temp(2)], 'g');
end

path(:,1) = path(:,1)-0.5;
path(:,2) = path(:,2)-0.5;
plot(path(:,1), path(:,2),'b','LineWidth',2);

% �����ϰ����ֹ����ɫ��
fill([start_point(1)-1, start_point(1), start_point(1), start_point(1)-1],...
[start_point(2)-1, start_point(2)-1 , start_point(2), start_point(2)], 'b');

fill([terminal_point(1)-1, terminal_point(1), terminal_point(1), terminal_point(1)-1],...
[terminal_point(2)-1, terminal_point(2)-1 , terminal_point(2), terminal_point(2)], 'r');

