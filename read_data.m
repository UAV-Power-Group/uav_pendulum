clear all; 
clc;
data = importdata('theta_P_D_bias.txt');

time = data(:,1);
theta_dot = data(:,2);
theta = data(:,3);
subplot(2,1,1);
plot(time, theta, 'r');
hold on;
plot(time, theta_dot, 'b');
% x轴
xlabel('时间'); 
% y轴
ylabel('角度'); 
%图例
legend('Theta','ThetaDot')


data_command = importdata('command_pd_bias.txt');
time_sp = data_command(:,1);
roll_sp = data_command(:,2);

for i = 2:length(time_sp)
    time_sp(i) = time_sp(i) - time_sp(1);
    roll_sp(i) = roll_sp(i);
end

time_sp(1) = 0;
subplot(2,1,2);
plot(time_sp, roll_sp);
% x轴
xlabel('时间'); 
% y轴
ylabel('推力'); 