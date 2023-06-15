% author = 'xujiafeng';
% data = '2022.11.17';
% license = 'GPL';
clear;
clc;
close all;

% 文件名
filename = 'data.txt';
% 要画的电机编号: 1,2,3...11,12
can_id = 1;

data = load(filename);
t = data(:,1) - data(1,1);
t_set = data(:, end-1);
t_get = data(:, end);
psid = can_id * 6 - 4;
prid = can_id * 6 - 3;
vsid = can_id * 6 - 2;
vrid = can_id * 6 - 1;
csid = can_id * 6;
crid = can_id * 6 + 1;

figure;
hold on;
box on;
grid on;
plot(t, t,'r','Linewidth',3.0)
plot(t, t_set,'b','Linewidth',3.0)
plot(t, t_get,'g','Linewidth',3.0)
xlabel('Time (s)');
ylabel('Different Time (s)');
h = legend('Time Record Data','Time Set Value','Time Get Reading','location','northeast');

in_one_fig = true;
figure('units','normalized','position',[0.1,0.1,0.8,0.35])
if in_one_fig
subplot(1,3,1);
end
hold on;
box on;
grid on;
plot(t, data(:,psid),'r','Linewidth',3.0)
plot(t, data(:,prid),'b','Linewidth',3.0)
xlabel('Time (s)');
ylabel('Position (rad)');
% h = legend('command','read','location','northeast');
% set(h,'fontsize',30,'fontname','Times');
set(gca,'fontsize',30,'fontname','Times');
% saveas(gcf, ['motor_', num2str(can_id), '_position.bmp']);

if in_one_fig
subplot(1,3,2);
else
figure
end
hold on;
box on;
grid on;
plot(t, data(:,vsid),'r','Linewidth',3.0)
plot(t, data(:,vrid),'b','Linewidth',3.0)
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
% h = legend('command','read','location','northeast');
% set(h,'fontsize',30,'fontname','Times');
set(gca,'fontsize',30,'fontname','Times');
% saveas(gcf, ['motor_', num2str(can_id), '_velocity.bmp']);

if in_one_fig
subplot(1,3,3);
else
figure
end
hold on;
box on;
grid on;
plot(t, data(:,csid),'r','Linewidth',3.0)
plot(t, data(:,crid),'b','Linewidth',3.0)
xlabel('Time (s)');
ylabel('Current (A)');
h = legend('command','read','location','northeast');
set(h,'fontsize',30,'fontname','Times');
set(gca,'fontsize',30,'fontname','Times');
% saveas(gcf, ['motor_', num2str(can_id), '_current.bmp']);

saveas(gcf, ['motor_', num2str(can_id), '_track.bmp']);
