% plot outLog.csv
% Author : Jiajun Wang
% 2020-07-20


% read  "outLog.csv" or " hipm*.csv "

clear;

numFiles = 1;

logData = [];

data_flag = 0;      % 0: simulation; 1: UBTMaster
test_flag = 0;      % 0: webots; 1: logTest

if data_flag == 0
    if test_flag == 0
        outLog_csv = '../local/outLog.csv';
    elseif test_flag == 1
        outLog_csv = '../local/outLogTest.csv';
        % test timekeeping
        timeLog_csv = '../local/timeLogTest.csv';
        timeData = importdata(timeLog_csv);
    end
    data = importdata(outLog_csv); 
    logData = data(:,1:end);
elseif data_flag == 1     
    for i = 1 : numFiles
        str = dec2hex(i);
        str = lower(str);
        data_name = sprintf(['../local/hipmLog_', str, '.csv']);
        data = importdata(data_name);
%         logData = [logData; data(:,2:end)];   % begin = 45
        logData = [logData; data(:,1:end)];     % begin = 0
    end
end

% --------------------------- Parse ------------------------------------

%
% parse & mapping
%

% if log ActJointPos/ActJointVel/ActJointTorque, set "begin = 45", else "begin = 0"
% ActPos = logData(:, 1:6); ActVel = logData(:, 7:12); ActTor = logData(:, 13:18);
% SetPos = logData(:, 19:24); SetVel = logData(:, 25:30); SetTor = logData(:, 31:36);
% imu = logData(:, 37:45);
% begin = 45;

begin = 0;

q_ = logData(:, begin+1:begin+11); qDot_ = logData(:, begin+12:begin+22); 
q_d_ = logData(:, begin+23:begin+33); qDot_d_ = logData(:, begin+34:begin+44); 
fPf_opt = logData(:, begin+45:begin+48); 
tauA_opt = logData(:, begin+49:begin+52);
c_R_W = logData(:, begin+53:begin+54); c_L_W = logData(:, begin+55:begin+56);  
c_R_S = logData(:, begin+57:begin+58); c_L_S = logData(:, begin+59:begin+60);  
c_U_S = logData(:, begin+61:begin+62); c_Com_S = logData(:, begin+63:begin+64);
cDot_R_W = logData(:, begin+65:begin+66); cDot_L_W = logData(:, begin+67:begin+68);    
cDot_R_S = logData(:, begin+69:begin+70); cDot_L_S = logData(:, begin+71:begin+72);  
cDot_U_S = logData(:, begin+73:begin+74); cDot_Com_S = logData(:, begin+75:begin+76);
c_R_W_d = logData(:, begin+77:begin+78); c_L_W_d = logData(:, begin+79:begin+80);  
c_R_S_d = logData(:, begin+81:begin+82); c_L_S_d = logData(:, begin+83:begin+84);  
c_U_S_d = logData(:, begin+85:begin+86); c_Com_S_d = logData(:, begin+87:begin+88);
cDot_R_W_d = logData(:, begin+89:begin+90); cDot_L_W_d = logData(:, begin+91:begin+92);  
cDot_R_S_d = logData(:, begin+93:begin+94); cDot_L_S_d = logData(:, begin+95:begin+96);  
cDot_U_S_d = logData(:, begin+97:begin+98); cDot_Com_S_d = logData(:, begin+99:begin+100);
u_R_A = logData(:, begin+101:begin+102); u_L_A = logData(:, begin+103:begin+104); 
fc_grf_R_S = logData(:, begin+105:begin+106); fc_grf_L_S = logData(:, begin+107:begin+108); 
fc_grf_R_S_ff = logData(:, begin+109:begin+110); fc_grf_L_S_ff = logData(:, begin+111:begin+112);
hgDotRef = logData(:, begin+113:begin+115);
qfDDotRef = logData(:, begin+116:begin+118);
cPfDDotRef = logData(:, begin+119:begin+122);
forcePfRef = logData(:, begin+123:begin+126);
xRddot_CL = logData(:, begin+127); zRddot_CL = logData(:, begin+128);
xLddot_CL = logData(:, begin+129); zLddot_CL = logData(:, begin+130);
timeCs = logData(:, begin+131); s = logData(:, begin+132); s_td = logData(:, begin+133);
s_R = logData(:, begin+134); s_L = logData(:, begin+135);
vx_cmd = logData(:, begin+136); vx_tgt = logData(:, begin+137);
H_tgt = logData(:, begin+138); 
vx_est = logData(:, begin+139); vx_est_nf = logData(:, begin+140); 
vx_preStep = logData(:, begin+141); vx_double = logData(:, begin+142);
vz_est = logData(:, begin+143); vz_est_nf = logData(:, begin+144); 
stanceLeg = logData(:, begin+145);
emergency_flag = logData(:, begin+146);
simpleStatus = logData(:, begin+147); nWSR_res = logData(:, begin+148); 
time_QP = logData(:, begin+149); cputime = logData(:, begin+150);
time_WBC = logData(:, begin+151);
cDDot_U_S_d = logData(:, begin+152:begin+153);
qDDot_opt = logData(:, begin+154:begin+156);
cPfDDot_opt = logData(:, begin+157:begin+160);
% rename
pitch = q_(:, 3); pitchDot = qDot_(:, 3);
pitch_d = q_d_(:, 3); pitchDot_d = qDot_d_(:, 3);

% joint names
joint_names = {    
                'fbTx'
                'fbTz'
                'fbRy'
                'hipFR'
                'kneeFR'
                'hipRR'
                'kneeRR'
                'hipFL'
                'kneeFL'
                'hipRL'
                'kneeRL'};
   
if data_flag == 0 && test_flag == 1
    test_time_controller = timeData(:,2);
end

% ------------------------------- mean ------------------------------
mean_QP = mean(time_QP);
mean_cputime = mean(cputime)*1e3;
mean_WBC = mean(time_WBC);

% ------------------------------- plot ------------------------------

t = timeCs;
ax = [];


if begin ~= 0
    % SetPos - ActPos
    f = figure;clf;     
    set(f, 'WindowStyle', 'docked');
    j = 1;
    for i = [5, 4, 3, 6, 1, 2]         % urdf : 012345  <-->  real : 501432
        ax = [ax, subplot(2, 3, j)]; 
        hold on;
        plot(t, stanceLeg, 'y--');
        plot(t, ActPos(:,i), 'k');
        plot(t, SetPos(:,i), 'b');
        plot(t, SetPos(:,i)-ActPos(:,i), 'r');
        title(['SetPos vs ActPos - ', num2str(i-1,'%d')]);
        xlabel('t (sec)');
        ylabel('angle (rad)');
        legend('stanceLeg', 'ActPos', 'SetPos', 'SetPos-ActPos'); 
        j = j+1;
    end      
    f.Name = 'SetPos vs ActPos';

    % SetVel - ActVel
    f = figure;clf;     
    set(f, 'WindowStyle', 'docked');
    j = 1;
    for i = [5, 4, 3, 6, 1, 2]          % urdf : 012345  <-->  real : 501432
        ax = [ax, subplot(2, 3, j)]; 
        hold on;
        plot(t, 5*stanceLeg, 'y--');
        plot(t, ActVel(:,i), 'k');
        plot(t, SetVel(:,i), 'b');
        plot(t, SetVel(:,i)-ActVel(:,i), 'r');
        title(['SetVel vs ActVel - ', num2str(i-1,'%d')]);
        xlabel('t (sec)');
        ylabel('angle velocity (rad/s)');
        legend('5*stanceLeg', 'ActVel', 'SetVel', 'SetVel-ActVel'); 
        j = j+1;
    end      
    f.Name = 'SetVel vs ActVel';

    % SetTor - ActTor
    f = figure;clf;     
    set(f, 'WindowStyle', 'docked');
    j = 1;
    for i = [5, 4, 3, 6, 1, 2]          % urdf : 012345  <-->  real : 501432
        ax = [ax, subplot(2, 3, j)]; 
        hold on;
        plot(t, 10*stanceLeg, 'y--');
        plot(t, ActTor(:,i), 'k');
        plot(t, SetTor(:,i), 'b');
        plot(t, SetTor(:,i)-ActTor(:,i), 'r');
        title(['SetTor vs ActTor - ', num2str(i-1,'%d')]);
        xlabel('t (sec)');
        ylabel('torque (Nm)');
        legend('10*stanceLeg', 'ActTor', 'SetTor', 'SetTor-ActTor'); 
        j = j+1;
    end      
    f.Name = 'SetTor vs ActTor';

end

% q-des vs q-act
f = figure;clf;   
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
j = 1;
for i = [10, 8, 6, 4]
    ax = [ax, subplot(2, 2, j)]; 
    hold on;
    plot(t, stanceLeg, 'y--');
    plot(t, q_(:,i), 'k');
    plot(t, q_d_(:,i), 'b');
    plot(t, q_d_(:,i)-q_(:,i), 'r');
    title(joint_names(i));
    xlabel('t (sec)');
    ylabel('angle (rad)');
    legend('stanceLeg', 'q-act', 'q-des', 'des-act'); 
    j = j+1;
end      
f.Name = 'q-des vs q-act';

% qDot-des vs qDot-act
f = figure;clf;    
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
j = 1;
for i = [10, 8, 6, 4]
    ax = [ax, subplot(2, 2, j)]; 
    hold on;
    plot(t, stanceLeg, 'y--');
    plot(t, qDot_(:,i), 'k');
    plot(t, qDot_d_(:,i), 'b');
    plot(t, qDot_d_(:,i)-qDot_(:,i), 'r');
    title(joint_names(i));
    xlabel('t (sec)');
    ylabel('angle velocity (rad/s)');
    legend('stanceLeg', 'qDot-act', 'qDot-des', 'des-act'); 
    j = j+1;
end      
f.Name = 'qDot-des vs qDot-act';

% tauA-des vs tauA-act
f = figure;clf;   
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(2, 2, 1)]; 
hold on;
plot(t, 20*stanceLeg, 'y--');
plot(t, u_L_A(:,2), 'k');
plot(t, tauA_opt(:,4), 'b');
% plot(t, tauA_opt(:,4) - u_L_A(:,2), 'r');
title(joint_names(10));
xlabel('t (sec)');
ylabel('torque (Nm)');
% legend('20*stanceLeg', 'tauA-act', 'tauA-des', 'des-act');  
legend('20*stanceLeg', 'tauA-act', 'tauA-des'); 
ax = [ax, subplot(2, 2, 2)]; 
hold on;
plot(t, 20*stanceLeg, 'y--');
plot(t, u_L_A(:,1), 'k');
plot(t, tauA_opt(:,3), 'b');
% plot(t, tauA_opt(:,3) - u_L_A(:,1), 'r');
title(joint_names(8));
xlabel('t (sec)');
ylabel('torque (Nm)');
legend('20*stanceLeg', 'tauA-act', 'tauA-des');  
% legend('20*stanceLeg', 'tauA-act', 'tauA-des', 'des-act');  
ax = [ax, subplot(2, 2, 3)]; 
hold on;
plot(t, 20*stanceLeg, 'y--');
plot(t, u_R_A(:,2), 'k');
plot(t, tauA_opt(:,2), 'b');
% plot(t, tauA_opt(:,2) - u_R_A(:,2), 'r');
title(joint_names(6));
xlabel('t (sec)');
ylabel('torque (Nm)');
legend('20*stanceLeg', 'tauA-act', 'tauA-des'); 
% legend('20*stanceLeg', 'tauA-act', 'tauA-des', 'des-act'); 
ax = [ax, subplot(2, 2, 4)]; 
hold on;
plot(t, 20*stanceLeg, 'y--');
plot(t, u_R_A(:,1), 'k');
plot(t, tauA_opt(:,1), 'b');
% plot(t, tauA_opt(:,1) - u_R_A(:,1), 'r');
title(joint_names(4));
xlabel('t (sec)');
ylabel('torque (Nm)');
legend('20*stanceLeg', 'tauA-act', 'tauA-des'); 
% legend('20*stanceLeg', 'tauA-act', 'tauA-des', 'des-act'); 
f.Name = 'tauA-des vs tauA-act';

% GRF-des vs GRF-act
f = figure;clf;     
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(2, 2, 1)]; 
hold on;
plot(t, 20*stanceLeg, 'y--');
plot(t, fc_grf_L_S(:,1), 'k:');
plot(t, fc_grf_L_S_ff(:,1), 'm');
plot(t, fPf_opt(:,3), 'b');
title('hGRF Left');
xlabel('t (sec)');
ylabel('force (N)');
legend('20*stanceLeg', 'hGRF-Left-act', 'hGRF-Left-ff', 'hGRF-Left-des');  
ax = [ax, subplot(2, 2, 2)]; 
hold on;
plot(t, 50*stanceLeg, 'y--');
plot(t, fc_grf_L_S(:,2), 'k:');
plot(t, fc_grf_L_S_ff(:,2), 'm');
plot(t, fPf_opt(:,4), 'b');
title('vGRF Left');
xlabel('t (sec)');
ylabel('force (N)');
legend('50*stanceLeg', 'vGRF-Left-act', 'vGRF-Left-ff', 'vGRF-Left-des');  
ax = [ax, subplot(2, 2, 3)]; 
hold on;
plot(t, 20*stanceLeg, 'y--');
plot(t, fc_grf_R_S(:,1), 'k:');
plot(t, fc_grf_R_S_ff(:,1), 'm');
plot(t, fPf_opt(:,1), 'b');
title('hGRF Right');
xlabel('t (sec)');
ylabel('force (N)');
legend('20*stanceLeg', 'hGRF-Right-act', 'hGRF-Right-ff', 'hGRF-Right-des');  
ax = [ax, subplot(2, 2, 4)]; 
hold on;
plot(t, 50*stanceLeg, 'y--');
plot(t, fc_grf_R_S(:,2), 'k:');
plot(t, fc_grf_R_S_ff(:,2), 'm');
plot(t, fPf_opt(:,2), 'b');
title('vGRF Right');
xlabel('t (sec)');
ylabel('force (N)');
legend('50*stanceLeg', 'vGRF-Right-act', 'vGRF-Right-ff', 'vGRF-Right-des');  
f.Name = 'GRF-des vs GRF-act';

% GRF-act and GRF-ff
f = figure;clf;  
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(2, 2, 1)]; 
hold on;
plot(t, 50*stanceLeg, 'y--');
plot(t, fc_grf_L_S(:,1), 'r');
plot(t, fc_grf_L_S(:,2), 'b');
title('GRF Left');
xlabel('t (sec)');
ylabel('force (N)');
legend('50*stanceLeg', 'hGRF-act', 'vGRF-act');  
ax = [ax, subplot(2, 2, 2)]; 
hold on;
plot(t, 50*stanceLeg, 'y--');
plot(t, fc_grf_L_S_ff(:,1), 'm');
plot(t, fc_grf_L_S_ff(:,2), 'k');
title('GRF-ff Left');
xlabel('t (sec)');
ylabel('force (N)');
legend('50*stanceLeg', 'hGRF-ff', 'vGRF-ff');  
ax = [ax, subplot(2, 2, 3)]; 
hold on;
plot(t, 50*stanceLeg, 'y--');
plot(t, fc_grf_R_S(:,1), 'r');
plot(t, fc_grf_R_S(:,2), 'b');
title('GRF Right');
xlabel('t (sec)');
ylabel('force (N)');
legend('50*stanceLeg', 'hGRF-act', 'vGRF-act');  
ax = [ax, subplot(2, 2, 4)]; 
hold on;
plot(t, 50*stanceLeg, 'y--');
plot(t, fc_grf_R_S_ff(:,1), 'm');
plot(t, fc_grf_R_S_ff(:,2), 'k');
title('GRF-ff Right');
xlabel('t (sec)');
ylabel('force (N)');
legend('50*stanceLeg', 'hGRF-ff', 'vGRF-ff'); 
f.Name = 'GRF-act and GRF-ff';

% S Com pos vel 
f = figure;clf;  
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(2, 2, 1)]; 
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, c_Com_S_d(:,1), 'b');
plot(t, c_Com_S(:,1), 'k');
plot(t, c_Com_S_d(:,1) - c_Com_S(:,1), 'r');
title('S Com x');
xlabel('t (s)');
ylabel('x (m)');
legend('0.5*stanceLeg', 'x-Com-S-d', 'x-Com-S', 'x-Com-S-d - x-Com-S'); 
ax = [ax, subplot(2, 2, 2)]; 
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, c_Com_S_d(:,2), 'b');
plot(t, c_Com_S(:,2), 'k');
plot(t, c_Com_S_d(:,2) - c_Com_S(:,2), 'r');
title('S Com z');
xlabel('t (s)');
ylabel('z (m)');
legend('0.5*stanceLeg', 'z-Com-S-d', 'z-Com-S', 'z-Com-S-d - z-Com-S'); 
ax = [ax, subplot(2, 2, 3)];
hold on;
plot(t, 0.01*stanceLeg, 'y--');
plot(t, cDot_Com_S_d(:,1), 'b');
plot(t, cDot_Com_S(:,1), 'k');
plot(t, cDot_Com_S_d(:,1) - cDot_Com_S(:,1), 'r');
title('S Com vx');
xlabel('t (s)');
ylabel('vx (m/s)');
legend('0.01*stanceLeg', 'xDot-Com-S-d', 'xDot-Com-S', 'xDot-Com-S-d - xDot-Com-S');   
ax = [ax, subplot(2, 2, 4)];
hold on;
plot(t, 0.01*stanceLeg, 'y--');
plot(t, cDot_Com_S_d(:,2), 'b');
plot(t, cDot_Com_S(:,2), 'k');
plot(t, cDot_Com_S_d(:,2) - cDot_Com_S(:,2), 'r');
title('S Com vz');
xlabel('t (s)');
ylabel('vz (m/s)');
legend('0.01*stanceLeg', 'zDot-Com-S-d', 'zDot-Com-S', 'zDot-Com-S-d - zDot-Com-S'); 
f.Name = 'S Com pos vel';

% S U pos vel
f = figure;clf;  
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(2, 2, 1)]; 
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, c_U_S_d(:,1), 'b');
plot(t, c_U_S(:,1), 'k');
plot(t, c_U_S_d(:,1) - c_U_S(:,1), 'r');
title('S U x');
xlabel('t (s)');
ylabel('x (m)');
legend('0.5*stanceLeg', 'x-U-S-d', 'x-U-S', 'x-U-S-d - x-U-S'); 
ax = [ax, subplot(2, 2, 2)]; 
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, c_U_S_d(:,2), 'b');
plot(t, c_U_S(:,2), 'k');
plot(t, c_U_S_d(:,2) - c_U_S(:,2), 'r');
title('S U z');
xlabel('t (s)');
ylabel('z (m)');
legend('0.5*stanceLeg', 'z-U-S-d', 'z-U-S', 'z-U-S-d - z-U-S'); 
ax = [ax, subplot(2, 2, 3)];
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, cDot_U_S_d(:,1), 'b');
plot(t, cDot_U_S(:,1), 'k');
plot(t, cDot_U_S_d(:,1) - cDot_U_S(:,1), 'r');
title('S U vx');
xlabel('t (s)');
ylabel('vx (m/s)');
legend('0.5*stanceLeg', 'xDot-U-S-d', 'xDot-U-S', 'xDot-U-S-d - xDot-U-S');   
ax = [ax, subplot(2, 2, 4)];
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, cDot_U_S_d(:,2), 'b');
plot(t, cDot_U_S(:,2), 'k');
plot(t, cDot_U_S_d(:,2) - cDot_U_S(:,2), 'r');
title('S U vz');
xlabel('t (s)');
ylabel('vz (m/s)');
legend('0.5*stanceLeg', 'zDot-U-S-d', 'zDot-U-S', 'zDot-U-S-d - zDot-U-S'); 
f.Name = 'S U pos vel';

% S U pos vel
f = figure;clf;  
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(1, 2, 1)]; 
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, cDDot_U_S_d(:,1), 'b');
title('S U x');
xlabel('t (s)');
ylabel('accX (m/s^2)');
legend('0.5*stanceLeg', 'xDDot-U-S-d'); 
ax = [ax, subplot(1, 2, 2)]; 
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, cDDot_U_S_d(:,2), 'b');
title('S U z');
xlabel('t (s)');
ylabel('accZ (m/s^2)');
legend('0.5*stanceLeg', 'zDDot-U-S-d');  
f.Name = 'S U acc';

% Internal Closed-loops
f = figure;clf;   
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked'); 
ax = [ax, subplot(2, 2, 1)]; 
hold on;
plot(t, xLddot_CL, 'r');
title('xddot-Left-ClosedLoop');
xlabel('t (sec)');
ylabel('xddot (m/s^2)');
legend('xDDot-L-CL');  
ax = [ax, subplot(2, 2, 2)]; 
hold on;
plot(t, zLddot_CL, 'r');
title('zddot-Left-ClosedLoop');
xlabel('t (sec)');
ylabel('zddot (m/s^2)');
legend('zDDot-L-CL');
ax = [ax, subplot(2, 2, 3)]; 
hold on;
plot(t, xRddot_CL, 'r');
title('xddot-Right-ClosedLoop');
xlabel('t (sec)');
ylabel('xddot (m/s^2)');
legend('xDDot-R-CL');  
ax = [ax, subplot(2, 2, 4)]; 
hold on;
plot(t, zRddot_CL, 'r');
title('zddot-Right-ClosedLoop');
xlabel('t (sec)');
ylabel('zddot (m/s^2)');
legend('zDDot-R-CL'); 
f.Name = 'Internal Closed-loops';

% W foot pos L R
f = figure;clf;    
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(2, 2, 1)]; 
hold on;
plot(t, 0.1*stanceLeg, 'y--');
plot(t, c_L_W_d(:,1), 'b');
plot(t, c_L_W(:,1), 'k');
plot(t, c_L_W_d(:,1) - c_L_W(:,1), 'r');
title('W footL x');
xlabel('t (s)');
ylabel('x (m)');
legend('0.1*stanceLeg', 'x-L-W-d', 'x-L-W', 'x-L-W-d - x-L-W'); 
ax = [ax, subplot(2, 2, 2)]; 
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, c_L_W_d(:,2), 'b');
plot(t, c_L_W(:,2), 'k');
plot(t, c_L_W_d(:,2) - c_L_W(:,2), 'r');
title('W footL z');
xlabel('t (s)');
ylabel('z (m)');
legend('0.5*stanceLeg', 'z-L-W-d', 'z-L-W', 'z-L-W-d - z-L-W'); 
ax = [ax, subplot(2, 2, 3)];
hold on;
plot(t, 0.1*stanceLeg, 'y--');
plot(t, c_R_W_d(:,1), 'b');
plot(t, c_R_W(:,1), 'k');
plot(t, c_R_W_d(:,1) - c_R_W(:,1), 'r');
title('W footR x');
xlabel('t (s)');
ylabel('x (m)');
legend('0.1*stanceLeg', 'x-R-W-d', 'x-R-W', 'x-R-W-d - x-R-W');   
ax = [ax, subplot(2, 2, 4)];
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, c_R_W_d(:,2), 'b');
plot(t, c_R_W(:,2), 'k');
plot(t, c_R_W_d(:,2) - c_R_W(:,2), 'r');
title('W footR z');
xlabel('t (s)');
ylabel('z (m)');
legend('0.5*stanceLeg', 'z-R-W-d', 'z-R-W', 'z-R-W-d - z-R-W'); 
f.Name = 'W foot pos L R';

% W foot error L R
f = figure;clf;   
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(2, 2, 1)]; 
hold on;
plot(t, 0.1*stanceLeg, 'y--');
plot(t, c_L_W_d(:,1), 'b');
plot(t, c_L_W(:,1), 'k');
title('W L x');
xlabel('t (s)');
ylabel('x (m)');
legend('0.1*stanceLeg', 'x-L-W-d', 'x-L-W'); 
ax = [ax, subplot(2, 2, 2)]; 
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, c_L_W_d(:,2), 'b');
plot(t, c_L_W(:,2), 'k');
title('W L z');
xlabel('t (s)');
ylabel('z (m)');
legend('0.5*stanceLeg', 'z-L-W-d', 'z-L-W'); 
ax = [ax, subplot(2, 2, 3)];
hold on;
plot(t, 0.01*stanceLeg, 'y--');
plot(t, c_L_W_d(:,1) - c_L_W(:,1), 'r');
title('W error L x');
xlabel('t (s)');
ylabel('x (m)');
legend('0.01*stanceLeg', 'x-L-W-d - x-L-W');   
ax = [ax, subplot(2, 2, 4)];
hold on;
plot(t, 0.01*stanceLeg, 'y--');
plot(t, c_L_W_d(:,2) - c_L_W(:,2), 'r');
title('W error L z');
xlabel('t (s)');
ylabel('z (m)');
legend('0.01*stanceLeg', 'z-L-W-d - z-L-W'); 
f.Name = 'W foot error L R';

% W foot pvel L R
f = figure;clf;   
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(2, 2, 1)]; 
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, cDot_L_W_d(:,1), 'b');
plot(t, cDot_L_W(:,1), 'k');
plot(t, cDot_L_W_d(:,1) - cDot_L_W(:,1), 'r');
title('W footL vx');
xlabel('t (s)');
ylabel('vx (m/s)');
legend('0.5*stanceLeg', 'xDot-L-W-d', 'xDot-L-W', 'xDot-L-W-d - xDot-L-W'); 
ax = [ax, subplot(2, 2, 2)]; 
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, cDot_L_W_d(:,2), 'b');
plot(t, cDot_L_W(:,2), 'k');
plot(t, cDot_L_W_d(:,2) - cDot_L_W(:,2), 'r');
title('W footL vz');
xlabel('t (s)');
ylabel('vz (m/s)');
legend('0.5*stanceLeg', 'zDot-L-W-d', 'zDot-L-W', 'zDot-L-W-d - zDot-L-W'); 
ax = [ax, subplot(2, 2, 3)];
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, cDot_R_W_d(:,1), 'b');
plot(t, cDot_R_W(:,1), 'k');
plot(t, cDot_R_W_d(:,1) - cDot_R_W(:,1), 'r');
title('W footR vx');
xlabel('t (s)');
ylabel('vx (m/s)');
legend('0.5*stanceLeg', 'xDot-R-W-d', 'xDot-R-W', 'xDot-R-W-d - xDot-R-W');   
ax = [ax, subplot(2, 2, 4)];
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, cDot_R_W_d(:,2), 'b');
plot(t, cDot_R_W(:,2), 'k');
plot(t, cDot_R_W_d(:,2) - cDot_R_W(:,2), 'r');
title('W footR vz');
xlabel('t (s)');
ylabel('vz (m/s)');
legend('0.5*stanceLeg', 'zDot-R-W-d', 'zDot-R-W', 'zDot-R-W-d - zDot-R-W'); 
f.Name = 'W foot vel L R';

% S foot pos L R
f = figure;clf;    
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(2, 2, 1)]; 
hold on;
plot(t, 0.1*stanceLeg, 'y--');
plot(t, c_L_S_d(:,1), 'b');
plot(t, c_L_S(:,1), 'k');
plot(t, c_L_S_d(:,1) - c_L_S(:,1), 'r');
title('S footL x');
xlabel('t (s)');
ylabel('x (m)');
legend('0.1*stanceLeg', 'x-L-S-d', 'x-L-S', 'x-L-S-d - x-L-S'); 
ax = [ax, subplot(2, 2, 2)]; 
hold on;
plot(t, 0.1*stanceLeg, 'y--');
plot(t, c_L_S_d(:,2), 'b');
plot(t, c_L_S(:,2), 'k');
plot(t, c_L_S_d(:,2) - c_L_S(:,2), 'r');
title('S footL z');
xlabel('t (s)');
ylabel('z (m)');
legend('0.1*stanceLeg', 'z-L-S-d', 'z-L-S', 'z-L-S-d - z-L-S'); 
ax = [ax, subplot(2, 2, 3)];
hold on;
plot(t, 0.1*stanceLeg, 'y--');
plot(t, c_R_S_d(:,1), 'b');
plot(t, c_R_S(:,1), 'k');
plot(t, c_R_S_d(:,1) - c_R_S(:,1), 'r');
title('S footR x');
xlabel('t (s)');
ylabel('x (m)');
legend('0.1*stanceLeg', 'x-R-S-d', 'x-R-S', 'x-R-S-d - x-R-S');   
ax = [ax, subplot(2, 2, 4)];
hold on;
plot(t, 0.1*stanceLeg, 'y--');
plot(t, c_R_S_d(:,2), 'b');
plot(t, c_R_S(:,2), 'k');
plot(t, c_R_S_d(:,2) - c_R_S(:,2), 'r');
title('S footR z');
xlabel('t (s)');
ylabel('z (m)');
legend('0.1*stanceLeg', 'z-R-S-d', 'z-R-S', 'z-R-S-d - z-R-S'); 
f.Name = 'S foot pos L R';

% S foot error L R
f = figure;clf;    
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(2, 2, 1)]; 
hold on;
plot(t, 0.1*stanceLeg, 'y--');
plot(t, c_L_S_d(:,1), 'b');
plot(t, c_L_S(:,1), 'k');
title('S L x');
xlabel('t (s)');
ylabel('x (m)');
legend('0.1*stanceLeg', 'x-L-S-d', 'x-L-S'); 
ax = [ax, subplot(2, 2, 2)]; 
hold on;
plot(t, 0.1*stanceLeg, 'y--');
plot(t, c_L_S_d(:,2), 'b');
plot(t, c_L_S(:,2), 'k');
title('S L z');
xlabel('t (s)');
ylabel('z (m)');
legend('0.1*stanceLeg', 'z-L-S-d', 'z-L-S'); 
ax = [ax, subplot(2, 2, 3)];
hold on;
plot(t, 0.01*stanceLeg, 'y--');
plot(t, c_L_S_d(:,1) - c_L_S(:,1), 'r');
title('S error L x');
xlabel('t (s)');
ylabel('x (m)');
legend('0.01*stanceLeg', 'x-L-S-d - x-L-S');   
ax = [ax, subplot(2, 2, 4)];
hold on;
plot(t, 0.01*stanceLeg, 'y--');
plot(t, c_L_S_d(:,2) - c_L_S(:,2), 'r');
title('S error L z');
xlabel('t (s)');
ylabel('z (m)');
legend('0.01*stanceLeg', 'z-L-S-d - z-L-S'); 
f.Name = 'S foot error L R';

% S foot vel L R
f = figure;clf;     
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(2, 2, 1)]; 
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, cDot_L_S_d(:,1), 'b');
plot(t, cDot_L_S(:,1), 'k');
plot(t, cDot_L_S_d(:,1) - cDot_L_S(:,1), 'r');
title('S footL vx');
xlabel('t (s)');
ylabel('vx (m/s)');
legend('0.5*stanceLeg', 'xDot-L-S-d', 'xDot-L-S', 'xDot-L-S-d - xDot-L-S'); 
ax = [ax, subplot(2, 2, 2)]; 
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, cDot_L_S_d(:,2), 'b');
plot(t, cDot_L_S(:,2), 'k');
plot(t, cDot_L_S_d(:,2) - cDot_L_S(:,2), 'r');
title('S footL vz');
xlabel('t (s)');
ylabel('vz (m/s)');
legend('0.5*stanceLeg', 'zDot-L-S-d', 'zDot-L-S', 'zDot-L-S-d - zDot-L-S'); 
ax = [ax, subplot(2, 2, 3)];
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, cDot_R_S_d(:,1), 'b');
plot(t, cDot_R_S(:,1), 'k');
plot(t, cDot_R_S_d(:,1) - cDot_R_S(:,1), 'r');
title('S footR vx');
xlabel('t (s)');
ylabel('vx (m/s)');
legend('0.5*stanceLeg', 'xDot-R-S-d', 'xDot-R-S', 'xDot-R-S-d - xDot-R-S');   
ax = [ax, subplot(2, 2, 4)];
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, cDot_R_S_d(:,2), 'b');
plot(t, cDot_R_S(:,2), 'k');
plot(t, cDot_R_S_d(:,2) - cDot_R_S(:,2), 'r');
title('S footR vz');
xlabel('t (s)');
ylabel('vz (m/s)');
legend('0.5*stanceLeg', 'zDot-R-S-d', 'zDot-R-S', 'zDot-R-S-d - zDot-R-S'); 
f.Name = 'S foot vel L R';

% Centroidal-Moment-Dot Ref
f = figure;clf;   
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(1, 3, 1)]; 
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, hgDotRef(:,1), 'b');
title('angular momentum dot');
xlabel('t (s)');
ylabel('AMdot (Nm)');
legend('0.5*stanceLeg', 'angular-momentum-dot-Ref'); 
ax = [ax, subplot(1, 3, 2)]; 
hold on;
plot(t, 10*stanceLeg, 'y--');
plot(t, hgDotRef(:,2), 'b');
title('x linear momentum dot');
xlabel('t (s)');
ylabel('LMdot (N)');
legend('10*stanceLeg', 'x-linear-momentum-dot-Ref'); 
ax = [ax, subplot(1, 3, 3)]; 
hold on;
plot(t, 100*stanceLeg, 'y--');
plot(t, hgDotRef(:,3), 'b');
title('z linear momentum dot');
xlabel('t (s)');
ylabel('LMdot (N)');
legend('100*stanceLeg', 'z-linear-momentum-dot-Ref'); 
f.Name = 'Centroidal-Moment-Dot Ref';

% Upper-body-pose-DDot Ref
f = figure;clf;   
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(1, 3, 1)]; 
hold on;
plot(t, 5*stanceLeg, 'y--');
plot(t, qfDDotRef(:,1), 'b');
plot(t, qDDot_opt(:,1), 'r:');
title('x-ddot UpperBody');
xlabel('t (s)');
ylabel('x-ddot (m/s^2)');
legend('5*stanceLeg', 'x-ddot-UB-Ref', 'x-ddot-UB-Opt'); 
ax = [ax, subplot(1, 3, 2)]; 
hold on;
plot(t, 5*stanceLeg, 'y--');
plot(t, qfDDotRef(:,2), 'b');
plot(t, qDDot_opt(:,2), 'r:');
title('z-ddot UpperBody');
xlabel('t (s)');
ylabel('z-ddot (m/s^2)');
legend('5*stanceLeg', 'z-ddot-UB-Ref', 'z-ddot-UB-Opt'); 
ax = [ax, subplot(1, 3, 3)]; 
hold on;
plot(t, 5*stanceLeg, 'y--');
plot(t, qfDDotRef(:,3), 'b');
plot(t, qDDot_opt(:,3), 'r:');
title('pitch-ddot UpperBody');
xlabel('t (s)');
ylabel('pitch-ddot (rad/s^2)');
legend('5*stanceLeg', 'pitch-ddot-UB-Ref', 'pitch-ddot-UB-Opt'); 
f.Name = 'Upper-body-pose-DDot Ref';

% Point-foot-cDDot Ref
f = figure;clf;   
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(2, 2, 1)]; 
hold on;
plot(t, 10*stanceLeg, 'y--');
plot(t, cPfDDotRef(:,3), 'b'); 
plot(t, cPfDDot_opt(:,3), 'r:');
title('x-ddot PointFoot Left');
xlabel('t (s)');
ylabel('x-ddot (m/s^2)');
legend('10*stanceLeg', 'x-ddot-PF-Ref', 'x-ddot-PF-Opt'); 
ax = [ax, subplot(2, 2, 2)]; 
hold on;
plot(t, 10*stanceLeg, 'y--');
plot(t, cPfDDotRef(:,4), 'b');
plot(t, cPfDDot_opt(:,4), 'r:');
title('z-ddot PointFoot Left');
xlabel('t (s)');
ylabel('z-ddot (m/s^2)');
legend('10*stanceLeg', 'z-ddot-PF-Ref', 'z-ddot-PF-Opt'); 
ax = [ax, subplot(2, 2, 3)]; 
hold on;
plot(t, 10*stanceLeg, 'y--');
plot(t, cPfDDotRef(:,1), 'b');
plot(t, cPfDDot_opt(:,1), 'r:');
title('x-ddot PointFoot Right');
xlabel('t (s)');
ylabel('x-ddot (m/s^2)');
legend('10*stanceLeg', 'x-ddot-PF-Ref', 'x-ddot-PF-Opt'); 
ax = [ax, subplot(2, 2, 4)]; 
hold on;
plot(t, 10*stanceLeg, 'y--');
plot(t, cPfDDotRef(:,2), 'b');
plot(t, cPfDDot_opt(:,2), 'r:');
title('z-ddot PointFoot Right');
xlabel('t (s)');
ylabel('z-ddot (m/s^2)');
legend('10*stanceLeg', 'z-ddot-PF-Ref', 'z-ddot-PF-Opt'); 
f.Name = 'Point-foot-cDDot Ref';

% Point-foot-force Ref
f = figure;clf;  
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(2, 2, 1)]; 
hold on;
plot(t, 10*stanceLeg, 'y--');
plot(t, forcePfRef(:,3), 'b');
title('fx PointFoot Left');
xlabel('t (s)');
ylabel('fx (N)');
legend('10*stanceLeg', 'fx-PF-Ref'); 
ax = [ax, subplot(2, 2, 2)]; 
hold on;
plot(t, 200*stanceLeg, 'y--');
plot(t, forcePfRef(:,4), 'b');
title('fz PointFoot Left');
xlabel('t (s)');
ylabel('fz (N)');
legend('200*stanceLeg', 'fz-PF-Ref'); 
ax = [ax, subplot(2, 2, 3)]; 
hold on;
plot(t, 10*stanceLeg, 'y--');
plot(t, forcePfRef(:,1), 'b');
title('fx PointFoot Right');
xlabel('t (s)');
ylabel('fx (N)');
legend('10*stanceLeg', 'fx-PF-Ref'); 
ax = [ax, subplot(2, 2, 4)]; 
hold on;
plot(t, 200*stanceLeg, 'y--');
plot(t, forcePfRef(:,2), 'b');
title('fz PointFoot Right');
xlabel('t (s)');
ylabel('fz (N)');
legend('200*stanceLeg', 'fz-PF-Ref'); 
f.Name = 'Point-foot-force Ref';

% s s_td s_L s_R 
f = figure;clf; 
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, axes(f)];
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, s, 'k');
plot(t, s_td, 'm');
plot(t, s_L, 'r');
plot(t, s_R, 'b');
plot(t, 10*pitch, 'g');
title('s');
xlabel('t (sec)');
ylabel('s');
legend('0.5*stanceLeg', 's', 's-td', 's-L', 's-R', '10 * pitch');    
f.Name = 's s-st s-sw s-td';

% vx estimation
f = figure;clf;  
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, axes(f)];
hold on;
plot(t, stanceLeg, 'y--');
plot(t, vx_cmd, 'g');
plot(t, vx_tgt, 'm');
plot(t, vx_est, 'b');
plot(t, vx_est_nf, 'c');
plot(t, vx_preStep, 'r');
plot(t, vx_double, 'k');
title('vx estimation');
xlabel('t (sec)');
ylabel('vx (m/s)');
legend('stanceLeg', 'vx-cmd', 'vx-tgt', 'vx-est', 'vx-nf', 'vx-odo-pre', 'vx-double');       
f.Name = 'vx estimation';

% vz estimation
f = figure;clf;  
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, axes(f)];
hold on;
plot(t, 0.001*stanceLeg, 'y--');
plot(t, vz_est, 'b');
plot(t, vz_est_nf, 'k');
title('vx estimation');
xlabel('t (sec)');
ylabel('vz (m/s)');
legend('0.1*stanceLeg', 'vz-est', 'vz-nf');       
f.Name = 'vz estimation';

% Upper body pitch pitchVel
f = figure;clf;   
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(1, 2, 1)]; 
hold on;
plot(t, stanceLeg, 'y--');
plot(t, rad2deg(pitch), 'k');
plot(t, rad2deg(pitch_d), 'b');
title('pitch');
xlabel('t (s)');
ylabel('angle (deg)');
legend('stanceLeg', 'pitch-act', 'pitch-des'); 
ax = [ax, subplot(1, 2, 2)]; 
hold on;
plot(t, 10*stanceLeg, 'y--');
plot(t, rad2deg(pitchDot), 'k');
plot(t, rad2deg(pitchDot_d), 'b');
title('pitch-vel');
xlabel('t (s)');
ylabel('angle-vel (deg/s)');
legend('10*stanceLeg', 'pitchDot-act', 'pitchDot-des'); 
f.Name = 'pitch pitchVel Upper body';

% velocity vs time
f = figure;clf; 
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, axes(f)];
hold on;
% plot(t, stanceLeg, 'y--');
% plot(t, vx_cmd, 'g');
plot(t, vx_tgt, 'm--');
plot(t, vx_est, 'b:');
% plot(t, vx_est_nf, 'b:');
% plot(t, vx_preStep, 'k');
plot(t, vx_double, 'k');
title('velocity vs time');
xlabel('t (sec)');
ylabel('v (m/s)');
legend( 'vx-tgt','vx-est', 'vx-ave-pre');       
f.Name = 'velocity vs time';

% CoM height vs time
f = figure;clf; 
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, axes(f)];
hold on;
% plot(t, stanceLeg, 'y--');
plot(t, H_tgt,'m--');
plot(t, c_Com_S(:, 2), 'k');
title('height vs time');
xlabel('t (sec)');
ylabel('cmd');
ylim([0.0, 0.6]);
legend( 'H-tgt', 'h-est');       
f.Name = 'CoM height vs time';

% upper body height vs time
f = figure;clf; 
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, axes(f)];
hold on;
% plot(t, stanceLeg, 'y--');
plot(t, H_tgt,'m--');
plot(t, c_U_S(:, 2), 'k');
title('height vs time');
xlabel('t (sec)');
ylabel('cmd');
ylim([0.0, 0.6]);
legend( 'H-tgt', 'h-est');       
f.Name = 'upperbody height vs time';

% Knee Angle
f = figure;clf;     
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(2, 2, 1)];
hold on;
plot(t, 100*stanceLeg, 'y--');
plot(t, rad2deg(q_(:, 11)), 'b');
plot(t, 120*emergency_flag, 'r');
title('Knee-Rear-Left');
xlabel('t (sec)');
ylabel('angle(deg)');
legend('100*stanceLeg', 'knee-RL', '120*emergency-flag');
ax = [ax, subplot(2, 2, 2)];
hold on;
plot(t, 100*stanceLeg, 'y--');
plot(t, rad2deg(q_(:, 9)), 'b');
plot(t, 120*emergency_flag, 'r');
title('Knee-Fore-Left');
xlabel('t (sec)');
ylabel('angle(deg)');
legend('100*stanceLeg', 'knee-FL', '120*emergency-flag');  
ax = [ax, subplot(2, 2, 3)]; 
hold on;
plot(t, 100*stanceLeg, 'y--');
plot(t, rad2deg(q_(:, 7)), 'b');
plot(t, 120*emergency_flag, 'r');
title('Knee-Rear-Right');
xlabel('t (sec)');
ylabel('angle(deg)');
legend('100*stanceLeg', 'knee-RR', '120*emergency-flag');
ax = [ax, subplot(2, 2, 4)];
hold on;
hold on;
plot(t, 100*stanceLeg, 'y--');
plot(t, rad2deg(q_(:, 5)), 'b');
plot(t, 120*emergency_flag, 'r');
title('Knee-Fore-Right');
xlabel('t (sec)');
ylabel('angle(deg)');
legend('100*stanceLeg', 'knee-FR', '120*emergency-flag'); 
f.Name = 'Knee Angle';

% Remoter CMD vs time
% f = figure;clf;     
% % data cursor setting
% dcm_obj = datacursormode(gcf);
% set(dcm_obj,'UpdateFcn',@NewCallback);
% % end of data cursor setting
% set(f, 'WindowStyle', 'docked');
% ax = [ax, axes(f)];
% hold on;
% plot(t, vx_cmd, 'r');
% plot(t, vx_tgt, 'b');
% plot(t, H_delta_cmd, 'k');
% plot(t, deltaH_signal, 'm');
% title('cmd vs time');
% xlabel('t (sec)');
% ylabel('cmd');
% legend( 'vx-cmd', 'vx-tgt', 'H-delta-cmd', 'deltaH-signal');       
% f.Name = 'Remoter CMD vs time';

% tauA-des vs tauA-act
f = figure;clf; 
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(2, 2, 1)]; 
hold on;
plot(t, tauA_opt(:,4), 'b');
title(joint_names(10));
xlabel('t (sec)');
ylabel('torque (Nm)');
legend('tau-des'); 
ax = [ax, subplot(2, 2, 2)]; 
hold on;
plot(t, tauA_opt(:,3), 'b');
title(joint_names(8));
xlabel('t (sec)');
ylabel('torque (Nm)');
legend('tau-des');  
ax = [ax, subplot(2, 2, 3)]; 
hold on;
plot(t, tauA_opt(:,2), 'b');
title(joint_names(6));
xlabel('t (sec)');
ylabel('torque (Nm)');
legend('tau-des'); 
ax = [ax, subplot(2, 2, 4)]; 
hold on;
plot(t, tauA_opt(:,1), 'b');
title(joint_names(4));
xlabel('t (sec)');
ylabel('torque (Nm)');
legend('tau-des'); 
f.Name = 'joint torque desired';

% QP time 
f = figure;clf; 
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(3, 1, 1)]; 
hold on;
plot(t, time_QP, 'k');
title('QP timekeeping');
xlabel('t (s)');
ylabel('t (ms)');
legend( 'time-QP' );  
ax = [ax, subplot(3, 1, 2)]; 
hold on;
plot(t, cputime*1e3, 'k');
title('qpOASES');
xlabel('t (s)');
ylabel('cputime (ms)');
legend( 'cputime' );   
ax = [ax, subplot(3, 1, 3)]; 
hold on;
plot(t, simpleStatus, 'r');
title('simpleStatus');
xlabel('t (s)');
ylabel('simpleStatus');
legend( 'simpleStatus' );  
f.Name = 'QP time';

% QP time & nWSR
f = figure;clf; 
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(2, 1, 1)]; 
hold on;
plot(t, cputime*1e3, 'k');
title('QP timekeeping');
xlabel('t (s)');
ylabel('cputime (ms)');
legend( 'time-QP' );  
ax = [ax, subplot(2, 1, 2)]; 
hold on;
plot(t, nWSR_res, 'k');
title('QP nWSR');
xlabel('t (s)');
ylabel('nWSR ');
legend( 'nWSR-QP' );  
f.Name = 'QP time & nWSRg';


% time
f = figure;clf; 
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, axes(f)];
hold on;
% plot(t, stanceLeg, 'y--');
% plot(t, time_RoDy,'m--');
% plot(t, time_TcUpdate, 'r');
plot(t, time_QP,'g--');
plot(t, time_WBC, 'k');
title('time');
xlabel('t (sec)');
ylabel('t (ms)');
legend( 'time-QP', 'time-WBC');       
f.Name = 'time';

if data_flag == 0 && test_flag == 1
    % test time
    f = figure;clf; 
    % data cursor setting
    dcm_obj = datacursormode(gcf);
    set(dcm_obj,'UpdateFcn',@NewCallback);
    % end of data cursor setting
    set(f, 'WindowStyle', 'docked');
    ax = [ax, subplot(2, 1, 1)]; 
    hold on;
    plot(t, test_time_controller, 'b');
    plot(t, time_WBC, 'k');
    title('timekeeping');
    xlabel('t (s)');
    ylabel('t (ms)');
    legend('time-controller', 'time-QP');  
    ax = [ax, subplot(2, 1, 2)]; 
    hold on;
    plot(t, test_time_controller - time_WBC, 'r');
    title('error');
    xlabel('t (s)');
    ylabel('t (ms)');
    legend( 'time-controller - time-QP' );  
    f.Name = 'test timekeeping';
end

% Synchronize limits of specified 2-D axes
linkaxes(ax, 'x');


% Rec = plotJointTorqueVelocity(logData, begin);

