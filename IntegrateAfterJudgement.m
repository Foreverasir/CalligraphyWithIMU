function [ V, D, data ] = IntegrateAfterJudgement( type, startIndex, stopIndex, Mt, M, wsize, threshold,MO )
%���ݱʻ��жϽ���Ի��ֽ��е��������� v1.0
%   Ŀǰ��ֱ�ۣ������ݾ����ȡ���Աʻ��޹ؼ��ٶ���������
%   type: 1�ᣬ2����3Ʋ��4�࣬5��
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

%% ��ͼ�۲�
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

