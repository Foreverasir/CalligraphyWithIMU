function [ result,startIndex,stopIndex ] = StrokeJudgement( Ma,Mg,StaticA,Threshold,V,D,data )
%笔画判断函数 v1.1
%   返回笔画类型 result = 1横，2竖，3撇，4捺，5点
%   输入参数：手机坐标系的三轴加速度，三轴重力，静态坐标系的加速度，此版不涉及转角
%   Threshold 行向量元素依次为重力判断标准的绝对值(1)，三轴加速度极值判断的绝对值(2,3,4)
%   判断标准：
%   1.横：X轴正向运动，积分结果连线横向，gx为负 < -1
%   2.竖：Z轴正向运动，积分结果连线纵向，gz为负 < -1
%   3.撇：gx为正，gz为负
%   4.捺：gx为负，gz为负
%   5.点：
%   6.钩：
%   7.提：
%   8.竖弯：
%   9.横折：
%   设置一个类别池typePool，多重标准进行投票，调整一些投票值，即可改变一些逻辑，这点可发展为机器学习问题

% 目前五类
typePool=zeros(1,5);
Length=length(Ma(:,1));
% 默认状态下首尾即为理想的积分起止点
startIndex=1;

% S-G滤波预处理
Ma=sgolayfilt(Ma,10,21);
Mg=sgolayfilt(Mg,10,21);
StaticA=sgolayfilt(StaticA,10,21);

% 获取波峰波谷, 这里主要服务于start点，尚未完成
[peaksStaticAX,peaksStaticAXIndex]=findpeaks(StaticA(:,1),'minpeakheight',Threshold(2));
[peaksStaticAY,peaksStaticAYIndex]=findpeaks(StaticA(:,2),'minpeakheight',Threshold(3));
[peaksStaticAZ,peaksStaticAZIndex]=findpeaks(StaticA(:,3),'minpeakheight',Threshold(4));

[peaks_StaticAX,peaks_StaticAXIndex]=findpeaks(-StaticA(:,1),'minpeakheight',Threshold(2));
[peaks_StaticAY,peaks_StaticAYIndex]=findpeaks(-StaticA(:,2),'minpeakheight',Threshold(3));
[peaks_StaticAZ,peaks_StaticAZIndex]=findpeaks(-StaticA(:,3),'minpeakheight',Threshold(4));


% 重力偏向的特征
[row_MgX]=find(Mg(:,1)<-Threshold(1));
[row_MgZ]=find(Mg(:,3)<-Threshold(1));
[rowMgX]=find(Mg(:,1)>Threshold(1));
[rowMgZ]=find(Mg(:,3)>Threshold(1));
% 判断横 捺
if length(row_MgX) > 20
    typePool(1) = typePool(1) + 1;
    if length(row_MgZ) > 20
        typePool(4) = typePool(4) + 2;
    end
end
% 判断竖 撇
if length(row_MgZ) > 20
    typePool(2) = typePool(2) + 1;
    if length(rowMgX) > 20
        typePool(3) = typePool(3) + 2;
    end
end
% 判断竖的收尾
if length(row_MgZ) > 10 && length(rowMgZ) > 20 && row_MgZ(end) < rowMgZ(1)
    typePool(2) = typePool(2) + 1;
end
% 判断横的收尾
if length(row_MgX) > 10 && length(rowMgX) > 20 && row_MgX(end) < rowMgZ(1)
    typePool(1) = typePool(1) + 1;
end

% 根据位移情况做判断
% TODO:根据长短还未加入
distance = sqrt(sum(D(:,[1, 3]) .^ 2,2));
[maxDistance,maxDistanceIndex] = max(distance);
% 计算夹角
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
    % 根据笔画确实再次积分的起止点，目前针对截止点
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