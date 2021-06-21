% compare 2 outLog.csv
% Author : Jiajun Wang
% 2020-07-20


% read  "outLog.csv" or " hipm*.csv "

% clear;

logData_A = [];

outLog_webots_csv = '../local/outLog.csv';
data_A = importdata(outLog_webots_csv); 
logData_A = data_A(:,1:end);

logData_B = [];

outLog_test_csv = '../local/outLogTest.csv';
data_B = importdata(outLog_test_csv); 
logData_B = data_B(:,1:end);


t_A = logData_A(:, 186);
tauA_opt_A = logData_A(:, 60:63);
t_B = logData_B(:, 186);
tauA_opt_B = logData_B(:, 60:63);

u_diff = tauA_opt_B - tauA_opt_A;

ax = [];

% u_diff
f = figure;clf;     
set(f, 'WindowStyle', 'docked');

ax = [ax, subplot(2, 2, 1)]; 
hold on;
plot(t_B, u_diff(:,1), 'r+');
title('uFR');
xlabel('t (sec)');
ylabel('u-err (Nm)');
legend('uFR'); 

ax = [ax, subplot(2, 2, 2)];
hold on;
plot(t_B, u_diff(:,2), 'r+');
title('uRR');
xlabel('t (sec)');
ylabel('u-err (Nm)');
legend('uRR');

ax = [ax, subplot(2, 2, 3)];
hold on;
plot(t_B, u_diff(:,3), 'r+');
title('uFL');
xlabel('t (sec)');
ylabel('u-err (Nm)');
legend('uFL');

ax = [ax, subplot(2, 2, 4)];
hold on;
plot(t_B, u_diff(:,4), 'r+');
title('uRL');
xlabel('t (sec)');
ylabel('u-err (Nm)');
legend('uRL');

f.Name = 'uError = u-Webots - u-Test';
% end of u_diff
    
    