function [ velocity,displacement,data,v,Magnitude] = ZV_Integrate( Mt,M,wsize,threshold,MOrientation)
%Zero-Velocity 轨迹积分方法 v1.1
%   返回速度，位移，data三个矩阵，速度位移为n*3；
%   输入参数为时间向量，加速度矩阵与窗口大小
%   第四个为阈值向量，元素依次为窗口内方差阈值，速度模阈值
windowsize=wsize;
n=size(M);
length=n(1);    %长度
width=n(2);     %列数 应为3
if(size(Mt(:,1))~=length)
    return ;
end
Magnitude=sqrt(sum(M.^2,2));    %求模
data=zeros(length,width);            %data用于存储达标值
dataindex=1;
indexmap=zeros(length,1);           %与下标形成映射
v = zeros(length,1);

%筛选data
for i=1:length
    if i >= windowsize
        w=Magnitude(i-windowsize+1:i,:);
        v(i) = var(w);
        if v(i)<threshold(1) && Magnitude(i)<threshold(2)
            data(i,:)=M(i,:)./10;           %缩小10倍
        else
            data(i,:)=M(i,:);
            indexmap(dataindex)=i;   %映射下标为调整准备
            dataindex=dataindex+1;  %index停留在下一行
        end
    end
end

% 调整index映射,两头缩紧
% indexmap(dataindex)=indexmap(dataindex-1);
% indexmap(1)=indexmap(2)-1;
indexmap=indexmap(1:dataindex-1);

% 调整data值,准备积分
sumdata=sum(data);  %被缩小的部分也算在内
percentdata=sumdata./(dataindex-1);
% 平均分配多余值，使得加速度总和为0
for i= 1 : dataindex-1
    data(indexmap(i),:)=data(indexmap(i),:)-percentdata;
end

% 积分得速度与位移
velocity=zeros(dataindex,width);
displacement=zeros(dataindex,width);

% V1.1:解决了积分时间片问题
for i= 2 : length
   velocity(i,:)=velocity(i-1,:)+(data(i-1,:)+data(i,:))*0.5*(Mt(i,1)-Mt(i-1,1));
end

for i= 2 : length
    displacement(i,:)=displacement(i-1,:)+(velocity(i-1,:)+velocity(i,:))*0.5*(Mt(i,1)-Mt(i-1,1));
%  TODO: 将手机中心轨迹投射到笔尖，这部分问题较大，先不考虑
%     displacement(i,1) = displacement(i,1) - threshold(3) * sin(MOrientation(1));
%     displacement(i,2) = displacement(i,2) - threshold(3) * cos(MOrientation(1)) * cos(MOrientation(2));
%     displacement(i,3) = displacement(i,3) - threshold(3) * sin(MOrientation(2));
end

% V1.0: 考虑积分时间片的问题，法2省掉了一部分时间，法1则根据映射使用了中间段的完整时间片，有的时间片大于0.02
% for i=2 : dataindex
%     velocity(i,:)=velocity(i-1,:)+(data(i-1,:)+data(i,1))*0.5*(Mt(indexmap(i),1)-Mt(indexmap(i-1),1));
%     %velocity(i,:)=velocity(i-1,:)+(data(i-1,:)+data(i,1))*0.5*0.02;
% end
% for i=2 : dataindex
%     displacement(i,:)=displacement(i-1,:)+(velocity(i-1,:)+velocity(i,1))*0.5*(Mt(indexmap(i),1)-Mt(indexmap(i-1),1));
%     %displacement(i,:)=displacement(i-1,:)+(velocity(i-1,:)+velocity(i,1))*0.5*0.02;
% end

end

