% 主程序脚本
% 绘图标志位
PigtureFlag= true;
%% Read the csv file
M=csvread('data\mu5.csv',1,1);
M=M(20:size(M(:,1))-20,:);
index=4; %0:acc,1:linearAcc,2:gyro,3:gravity,4:ChangedAcc,5:mOrientation
d=2^2;
m0=M(:,1);

m1=M(:,2+index*3);
m2=M(:,3+index*3);
m3=M(:,4+index*3);
% 注意这里不再加对M的宽度判断
mOrientation=M(:,17:19);

%% 预处理
n=size(m0);
N=2^nextpow2(n(1)); %原离散信号有N点

%去掉首尾后需要调整m0时间
starttime=m0(1,1);
for i=1 : n(1)
    m0(i,1)=m0(i,1)-starttime;
end

%% filter
% S-G滤波
sgm1=sgolayfilt(m1,10,21);
sgm2=sgolayfilt(m2,10,21);
sgm3=sgolayfilt(m3,10,21);

% 低通滤波
% lm1=lowp(m1,20,30,0.1,30,100);
% lm2=lowp(m2,20,30,0.1,30,100);
% lm3=lowp(m3,20,30,0.1,30,100);

% 三轴方差
vsize = 20;
vxyz = zeros(n(1),3);
sgm = [sgm1,sgm2,sgm3];
for i=1:n(1)
    if i >= vsize
        w=sgm(i-vsize+1 : i , :);
        vxyz(i,:) = var(w);
    end
end


%% 坐标转换后初步积分并绘制
% 第三个参数为窗口大小，第四个为阈值向量，元素依次为窗口内方差阈值，速度模阈值，笔杆长度
[V1,D1,data1,variance,magnitude]=ZV_Integrate(m0,[sgm1,sgm2,sgm3],20,[0.06, 0.4, 0.2],mOrientation);

if PigtureFlag
    % 二维、三维轨迹图
    figure
    plot(D1(:,1),D1(:,3),'b*')
    grid on;
    xlabel('X');  ylabel('Z');
    set(gca, 'yDir','reverse');
    figure
    plot3(D1(:,1),D1(:,2),D1(:,3),'bo');
    grid on;
    xlabel('X');  ylabel('Y');  zlabel('Z'); 
    %加速度比较、速度、位移图像
    figure
    set(gcf,'color','w') 
    subplot(3,1,1)
    %plot(m0,[sgm1,sgm2,sgm3]);
    %plot(m0,magnitude);
    plot(m0,vxyz);
    grid on;
    legend('X','Y','Z');
    subplot(3,1,2);
    plot(m0,data1);
    grid on;
    legend('X','Y','Z');
    subplot(3,1,3);
    plot(m0,variance);
    grid on;
    legend('Variance');
    
    figure
    set(gcf,'color','w') 
    subplot(2,1,1);
    plot(m0,V1);
    grid on;
    legend('X','Y','Z');
    subplot(2,1,2);
    plot(m0,D1);
    grid on;
    legend('X','Y','Z');
end

%% 笔画判断
% 传入一个经验参数Vector,用于存放阈值等，这些值后续可能通过学习获得
% Vector 行向量元素依次为重力判断标准的绝对值，三轴加速度极值判断的绝对值
empricalVector = [1.6,0.6,0.4,0.6];
[typeResult,startIndex,stopIndex] = StrokeJudgement(M(:,5:7),M(:,11:13),M(:,14:16),empricalVector,V1,D1,data1);

%% 再积分
%根据笔画判断重新积分
[V2,D2,data2] = IntegrateAfterJudgement(typeResult,startIndex,stopIndex,m0,[sgm1,sgm2,sgm3],5,[0.06,0.4,0.2],mOrientation);

%% 再判断与特征获取
% 会出现之前判断（依赖重力）未出结果的状态，此时开始处理
% 根据位移值 XZ平面 确立最远距离点
distance = sqrt(sum(D2(:,[1, 3]) .^ 2,2));
[maxDistance,maxDistanceIndex] = max(distance);
% 计算夹角
rawAngle = atan(D2(maxDistanceIndex,3) ./ D2(maxDistanceIndex,1));
degAngle = rad2deg(rawAngle);
% 此处经验参数暂时不统一管理
if typeResult == 0
    if degAngle < 25 && degAngle> -25
        typeResult = 1;
    elseif (degAngle > 75 && degAngle< 90) || (degAngle < -75 && degAngle > -90)
        typeResult = 2;
    elseif degAngle > 25 && degAngle < 75
        typeResult = 4;
    elseif degAngle < -25 && degAngle>-75
        typeResult = 3;
    else
        typeResult = 0; % Still No Type        
    end
    stopIndex = maxDistanceIndex;
    [V2,D2,data2] = IntegrateAfterJudgement(typeResult,startIndex,stopIndex,m0,[sgm1,sgm2,sgm3],5,[0.06,0.4,0.2],mOrientation);
end

% 二维、三维轨迹图
if PigtureFlag
    figure
    set(gcf,'color','w')
    plot(D2(:,1),D2(:,3),'*','color',[0.5,0,0.6]);
    grid on;
    xlabel('X');  ylabel('Z');
    set(gca, 'yDir','reverse');
    figure
    set(gcf,'color','w')
    plot3(D2(:,1),D2(:,2),D2(:,3),'o','color',[0.5,0,0.6]);
    grid on;
    xlabel('X');  ylabel('Y');  zlabel('Z'); 
end

%求导
% d1=myDiff(m0,lm1);
% d2=myDiff(m0,lm2);
% d3=myDiff(m0,lm3);
% dd1=myDiff(m0,d1);
% dd2=myDiff(m0,d2);
% dd3=myDiff(m0,d3);
% figure
% subplot(3,2,1);
% plot(m0,d1);
% subplot(3,2,2);
% plot(m0,dd1);
% subplot(3,2,3);
% plot(m0,d2);
% subplot(3,2,4);
% plot(m0,dd2);
% subplot(3,2,5);
% plot(m0,d3);
% subplot(3,2,6);
% plot(m0,dd3);

%% 傅里叶变换
% flm1=fft(lm1);
% flm2=fft(lm2);
% flm3=fft(lm3);

%%短时傅里叶分析
% [sflm1,T1,F1,C1]=spectrogram(lm1,N/d,N/(2*d),N/d,100);
% [sflm2,T2,F2,C2]=spectrogram(lm2,N/d,N/(2*d),N/d,100);
% [sflm3,T3,F3,C3]=spectrogram(lm3,N/d,N/(2*d),N/d,100);

% figure
% imagesc(T1,F1,C1);  
% set(gca,'YDir','normal')  
% colorbar;  
% xlabel('时间 t/s');  
% ylabel('频率 f/Hz');  
% title('短时傅里叶时频图x'); 
% 
% figure
% imagesc(T2,F2,C2);  
% set(gca,'YDir','normal')  
% colorbar;  
% xlabel('时间 t/s');  
% ylabel('频率 f/Hz');  
% title('短时傅里叶时频图y'); 
% 
% figure
% imagesc(T3,F3,C3);  
% set(gca,'YDir','normal')  
% colorbar;  
% xlabel('时间 t/s');  
% ylabel('频率 f/Hz');  
% title('短时傅里叶时频图z'); 

%%绘图
% figure
% subplot(2,2,1);
% plot(m0,lm1);
% if(index>0)
%     ylim([-2 2]);
% end
% xlabel('时间 s');  
% ylabel('a m/s^2');  
% title('低通滤波后X轴线性加速度')
% subplot(2,2,2);
% plot(abs(flm1));
% axis([0 n(1)/2 0 40]);
% xlabel('频率');  
% ylabel('幅值');  
% title('X轴线性加速度信号快速傅里叶变换')

% subplot(3,2,3);
% plot(m0,lm2);
% if(index>0)
%     ylim([-2 2]);
% end
% xlabel('时间 s');  
% ylabel('a m/s^2');  
% title('低通滤波后Y轴线性加速度')
% subplot(3,2,4);
% plot(abs(flm2));
% axis([0 n(1)/2 0 40]);
% xlabel('频率');  
% ylabel('幅值');  
% title('Y轴线性加速度信号快速傅里叶变换')

% subplot(2,2,3);
% plot(m0,lm3);
% if(index>0)
%     ylim([-2 2]);
% end
% xlabel('时间 s');  
% ylabel('a m/s^2');  
% title('低通滤波后Z轴线性加速度')
% subplot(2,2,4);
% plot(abs(flm3));
% axis([0 n(1)/2 0 40]);
% xlabel('频率');  
% ylabel('幅值');  
% title('Z轴线性加速度信号快速傅里叶变换')

%%积分操作
% s1=mytrapz(m0,sgm1);
% s2=mytrapz(m0,sgm2);
% s3=mytrapz(m0,sgm3);
% figure
% subplot(3,1,1);
% plot(m0,s1);
% subplot(3,1,2);
% plot(m0,s2);
% subplot(3,1,3);
% plot(m0,s3);