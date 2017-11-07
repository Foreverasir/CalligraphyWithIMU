function [ velocity,displacement,data,v,Magnitude] = ZV_Integrate( Mt,M,wsize,threshold,MOrientation)
%Zero-Velocity �켣���ַ��� v1.1
%   �����ٶȣ�λ�ƣ�data���������ٶ�λ��Ϊn*3��
%   �������Ϊʱ�����������ٶȾ����봰�ڴ�С
%   ���ĸ�Ϊ��ֵ������Ԫ������Ϊ�����ڷ�����ֵ���ٶ�ģ��ֵ
windowsize=wsize;
n=size(M);
length=n(1);    %����
width=n(2);     %���� ӦΪ3
if(size(Mt(:,1))~=length)
    return ;
end
Magnitude=sqrt(sum(M.^2,2));    %��ģ
data=zeros(length,width);            %data���ڴ洢���ֵ
dataindex=1;
indexmap=zeros(length,1);           %���±��γ�ӳ��
v = zeros(length,1);

%ɸѡdata
for i=1:length
    if i >= windowsize
        w=Magnitude(i-windowsize+1:i,:);
        v(i) = var(w);
        if v(i)<threshold(1) && Magnitude(i)<threshold(2)
            data(i,:)=M(i,:)./10;           %��С10��
        else
            data(i,:)=M(i,:);
            indexmap(dataindex)=i;   %ӳ���±�Ϊ����׼��
            dataindex=dataindex+1;  %indexͣ������һ��
        end
    end
end

% ����indexӳ��,��ͷ����
% indexmap(dataindex)=indexmap(dataindex-1);
% indexmap(1)=indexmap(2)-1;
indexmap=indexmap(1:dataindex-1);

% ����dataֵ,׼������
sumdata=sum(data);  %����С�Ĳ���Ҳ������
percentdata=sumdata./(dataindex-1);
% ƽ���������ֵ��ʹ�ü��ٶ��ܺ�Ϊ0
for i= 1 : dataindex-1
    data(indexmap(i),:)=data(indexmap(i),:)-percentdata;
end

% ���ֵ��ٶ���λ��
velocity=zeros(dataindex,width);
displacement=zeros(dataindex,width);

% V1.1:����˻���ʱ��Ƭ����
for i= 2 : length
   velocity(i,:)=velocity(i-1,:)+(data(i-1,:)+data(i,:))*0.5*(Mt(i,1)-Mt(i-1,1));
end

for i= 2 : length
    displacement(i,:)=displacement(i-1,:)+(velocity(i-1,:)+velocity(i,:))*0.5*(Mt(i,1)-Mt(i-1,1));
%  TODO: ���ֻ����Ĺ켣Ͷ�䵽�ʼ⣬�ⲿ������ϴ��Ȳ�����
%     displacement(i,1) = displacement(i,1) - threshold(3) * sin(MOrientation(1));
%     displacement(i,2) = displacement(i,2) - threshold(3) * cos(MOrientation(1)) * cos(MOrientation(2));
%     displacement(i,3) = displacement(i,3) - threshold(3) * sin(MOrientation(2));
end

% V1.0: ���ǻ���ʱ��Ƭ�����⣬��2ʡ����һ����ʱ�䣬��1�����ӳ��ʹ�����м�ε�����ʱ��Ƭ���е�ʱ��Ƭ����0.02
% for i=2 : dataindex
%     velocity(i,:)=velocity(i-1,:)+(data(i-1,:)+data(i,1))*0.5*(Mt(indexmap(i),1)-Mt(indexmap(i-1),1));
%     %velocity(i,:)=velocity(i-1,:)+(data(i-1,:)+data(i,1))*0.5*0.02;
% end
% for i=2 : dataindex
%     displacement(i,:)=displacement(i-1,:)+(velocity(i-1,:)+velocity(i,1))*0.5*(Mt(indexmap(i),1)-Mt(indexmap(i-1),1));
%     %displacement(i,:)=displacement(i-1,:)+(velocity(i-1,:)+velocity(i,1))*0.5*0.02;
% end

end

