clear; matlabrc; clc; close all;
addpath(genpath('controllers'))
addpath(genpath('dynamics'))
addpath(genpath('tools'))

% Initial Control gains:
k_ria = 20;  %(inter-agent position)
k_via = 30;  %(inter-agent velocities)
k_rvl = 50; %(virtual-leader position)
k_vvl = 20;  %(virtual-leader velocity)
k_obs = 30;   %(obstacle position)
obs_dist = 50;
gains = [k_ria,k_via,k_rvl,k_vvl,k_obs,obs_dist]';
% simulate_dev(gains,1,1);

% Optimize:
options = optimoptions('fmincon','FiniteDifferenceStepSize',[1e-2 1e-1 1e-1 1e-1 1e-2 1e-2]);
% A = [4 0 0 0 30/31.5];
% b = 0;
A =[];
b = [];
Aeq = [];
beq = [];
lb = [0 0 0 0 0  20];
baseline = 1;
FOV = 50;
resH = 500;
ub = [100 100 100 100 100 (baseline/2)/tand(((FOV/2)/(resH/2))/2)];

ts_history = [];
min_dist = [];
save OUT ts_history min_dist
[optimized_gains,~,~,output] = fmincon(@simulate, gains, A,b,Aeq,beq,lb,ub,[], options);

%% Plot
load OUT
figure()
plot(min_dist,'*r')
plot(ts_history,'LineWidth',2)
grid on
xlabel('Iteration')
ylabel('Setting Time (sec)')
title('Convergence History')
% saveas(gcf,'../Report/figs/converge','png')

%% Plot:
% Generate plots of animation:
[ts1, total_error1] = simulate_dev(gains,1,0);
% saveas(gcf,'../Report/figs/trajectory_original','png')
close all

[ts2, total_error2] = simulate_dev(optimized_gains,1,0);
% saveas(gcf,'../Report/figs/trajectory_optimized','png')
close all

% Gains from the GA:
ga_gains = [91.9202,1.5019,82.4238,14.4036,34.2422,409.6430];
[ts3, total_error3] = simulate_dev(ga_gains,1,0);


%% 
figure()
plot(linspace(0,ts1,numel(total_error1)),total_error1,'LineWidth',2); hold on
plot(linspace(0,ts2,numel(total_error2)),total_error2,'LineWidth',2)
plot(linspace(0,ts3,numel(total_error3)),total_error3,'LineWidth',2)
grid on 
ylabel('Total Error Metric')
xlabel('Time (sec)')
title('Error History')
legend('Origina','Inter-Point','Genetic')
% saveas(gcf,'../Report/figs/error','png')