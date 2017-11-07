function [ result,startIndex,stopIndex ] = StrokeJudgement( Ma,Mg,StaticA,Threshold,V,D,data )
%�ʻ��жϺ��� v1.1
%   ���رʻ����� result = 1�ᣬ2����3Ʋ��4�࣬5��
%   ����������ֻ�����ϵ��������ٶȣ�������������̬����ϵ�ļ��ٶȣ��˰治�漰ת��
%   Threshold ������Ԫ������Ϊ�����жϱ�׼�ľ���ֵ(1)��������ٶȼ�ֵ�жϵľ���ֵ(2,3,4)
%   �жϱ�׼��
%   1.�᣺X�������˶������ֽ�����ߺ���gxΪ�� < -1
%   2.����Z�������˶������ֽ����������gzΪ�� < -1
%   3.Ʋ��gxΪ����gzΪ��
%   4.�ࣺgxΪ����gzΪ��
%   5.�㣺
%   6.����
%   7.�᣺
%   8.���䣺
%   9.���ۣ�
%   ����һ������typePool�����ر�׼����ͶƱ������һЩͶƱֵ�����ɸı�һЩ�߼������ɷ�չΪ����ѧϰ����

% Ŀǰ����
typePool=zeros(1,5);
Length=length(Ma(:,1));
% Ĭ��״̬����β��Ϊ����Ļ�����ֹ��
startIndex=1;

% S-G�˲�Ԥ����
Ma=sgolayfilt(Ma,10,21);
Mg=sgolayfilt(Mg,10,21);
StaticA=sgolayfilt(StaticA,10,21);

% ��ȡ���岨��, ������Ҫ������start�㣬��δ���
[peaksStaticAX,peaksStaticAXIndex]=findpeaks(StaticA(:,1),'minpeakheight',Threshold(2));
[peaksStaticAY,peaksStaticAYIndex]=findpeaks(StaticA(:,2),'minpeakheight',Threshold(3));
[peaksStaticAZ,peaksStaticAZIndex]=findpeaks(StaticA(:,3),'minpeakheight',Threshold(4));

[peaks_StaticAX,peaks_StaticAXIndex]=findpeaks(-StaticA(:,1),'minpeakheight',Threshold(2));
[peaks_StaticAY,peaks_StaticAYIndex]=findpeaks(-StaticA(:,2),'minpeakheight',Threshold(3));
[peaks_StaticAZ,peaks_StaticAZIndex]=findpeaks(-StaticA(:,3),'minpeakheight',Threshold(4));


% ����ƫ�������
[row_MgX]=find(Mg(:,1)<-Threshold(1));
[row_MgZ]=find(Mg(:,3)<-Threshold(1));
[rowMgX]=find(Mg(:,1)>Threshold(1));
[rowMgZ]=find(Mg(:,3)>Threshold(1));
% �жϺ� ��
if length(row_MgX) > 20
    typePool(1) = typePool(1) + 1;
    if length(row_MgZ) > 20
        typePool(4) = typePool(4) + 2;
    end
end
% �ж��� Ʋ
if length(row_MgZ) > 20
    typePool(2) = typePool(2) + 1;
    if length(rowMgX) > 20
        typePool(3) = typePool(3) + 2;
    end
end
% �ж�������β
if length(row_MgZ) > 10 && length(rowMgZ) > 20 && row_MgZ(end) < rowMgZ(1)
    typePool(2) = typePool(2) + 1;
end
% �жϺ����β
if length(row_MgX) > 10 && length(rowMgX) > 20 && row_MgX(end) < rowMgZ(1)
    typePool(1) = typePool(1) + 1;
end

% ����λ��������ж�
% TODO:���ݳ��̻�δ����
distance = sqrt(sum(D(:,[1, 3]) .^ 2,2));
[maxDistance,maxDistanceIndex] = max(distance);
% ����н�
maxDistanceX = D(maxDistanceIndex,1);
maxDistanceZ = D(maxDistanceIndex,3);
rawAngle = atan(maxDistanceZ ./ maxDistanceX);
degAngle = rad2deg(rawAngle);
if degAngle < 25 && degAngle> -25 && maxDistanceX > 0
    typePool(1) = typePool(1) + 2;
elseif ((degAngle > 75 && degAngle< 90) || (degAngle < -75 && degAngle > -90)) && maxDistanceZ > 0
    typePool(2) = typePool(2) + 2;
elseif degAngle > 25 && degAngle < 75 && maxDistanceZ > 0 && maxDistanceX > 0
    typePool(4) = typePool(4) + 2;
elseif degAngle < -25 && degAngle>-75 && maxDistanceZ > 0 && maxDistanceX < 0
    typePool(3) = typePool(3) + 2;       
elseif degAngle <= -25 && degAngle >-75 && maxDistanceX > 0 && maxDistanceZ < 0
    typePool(7) = typePool(7) + 2;
elseif degAngle > 25 && degAngle < 75 && maxDistanceX < 0 && maxDistanceZ < 0
    typePool(6) = typePool(6) + 2;

end

%[minMa,minMaIndex]=min(Ma);
%[minMg,minMgIndex]=min(Mg);
%[minStaticA,minStaticAIndex] = min(StaticA);
%[maxMa,maxMaIndex]=max(Ma);
%[maxMg,maxMgIndex]=max(Mg);
%[maxStaticA,maxStaticAIndex] = max(StaticA);

[maxType,maxTypeIndex] = max(typePool);
if maxType == 0
    result = 0;
    stopIndex = Length;
else
    result=maxTypeIndex;
    % ���ݱʻ�ȷʵ�ٴλ��ֵ���ֹ�㣬Ŀǰ��Խ�ֹ��
    if result == 1
        [~,maxIndex] = max(D(:,1));
    elseif result == 2
        [~,maxIndex] = max(D(:,3));  
    elseif result == 3
        [~,maxIndexZ] = max(D(:,3));
        [~,minIndexX] = min(D(:,1));
        maxIndex = 0.5 * (maxIndexZ + minIndexX);
    elseif result == 4
        [~,maxIndexZ] = max(D(:,3)); 
        [~,maxIndexX] = max(D(:,1));
        maxIndex = 0.5 * (maxIndexZ + maxIndexX);
    end
    stopIndex=maxIndex;
end

end