function [ V, D, data ] = IntegrateAfterJudgement( type, startIndex, stopIndex, Mt, M, wsize, threshold,MO )
%根据笔画判断结果对积分进行调整与修正 v1.0
%   目前很直观，对数据矩阵截取，对笔画无关加速度削弱即可
%   type: 1横，2竖，3撇，4捺，5点
if type > 0
    newMt = Mt(startIndex : stopIndex,:);
    newM = M(startIndex : stopIndex,:);
    
    if type == 1
        newM(:, 3) = newM(:, 3) ./ 10;
    end
    if type == 2
        newM(:, 1) = newM(:, 1) ./ 10;
    end

    [ V, D, data ] = ZV_Integrate(newMt, newM, wsize, threshold,MO);
else
    [ V, D, data ] = ZV_Integrate(Mt, M, wsize, threshold,MO);
end

%% 绘图观察
% figure
% subplot(3,1,1);

% plot(newMt,data);
% grid on;
% legend('X','Y','Z');
% subplot(3,1,2);
% plot(newMt,V);
% grid on;
% legend('X','Y','Z');
% subplot(3,1,3);
% plot(newMt,D);
% grid on;
% legend('X','Y','Z');

end

