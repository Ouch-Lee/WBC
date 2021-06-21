% plot outLog.csv
% Author : Jiajun Wang
% 2020-07-20com


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
        data_name = sprintf(['wbcLog_', str, '.csv']);
        data = importdata(data_name);
        logData = [logData; data(:,1:end)];
    end
end

% --------------------------- Parse ------------------------------------

%
% parse & mapping
%
q_ = logData(:, 1:11); qDot_ = logData(:, 12:22); 
q_d_ = logData(:, 23:33); qDot_d_ = logData(:, 34:44); 
qDDot_opt = logData(:, 45:55); 
fPf_opt = logData(:, 56:59); 
tauA_opt = logData(:, 60:63);
c_R_W = logData(:, 64:65); c_L_W = logData(:, 66:67);  
c_R_G = logData(:, 68:69); c_L_G = logData(:, 70:71);  
c_R_S = logData(:, 72:73); c_L_S = logData(:, 74:75);  
c_U_S = logData(:, 76:77); c_Com_S = logData(:, 78:79);
cDot_R_W = logData(:, 80:81); cDot_L_W = logData(:, 82:83);  
cDot_R_G = logData(:, 84:85); cDot_L_G = logData(:, 86:87);  
cDot_R_S = logData(:, 88:89); cDot_L_S = logData(:, 90:91);  
cDot_U_S = logData(:, 92:93); cDot_Com_S = logData(:, 94:95);
c_R_W_d = logData(:, 96:97); c_L_W_d = logData(:, 98:99);  
c_R_G_d = logData(:, 100:101); c_L_G_d = logData(:, 102:103);  
c_R_S_d = logData(:, 104:105); c_L_S_d = logData(:, 106:107);  
c_U_S_d = logData(:, 108:109); c_Com_S_d = logData(:, 110:111);
cDot_R_W_d = logData(:, 112:113); cDot_L_W_d = logData(:, 114:115);  
cDot_R_G_d = logData(:, 116:117); cDot_L_G_d = logData(:, 118:119);  
cDot_R_S_d = logData(:, 120:121); cDot_L_S_d = logData(:, 122:123);  
cDot_U_S_d = logData(:, 124:125); cDot_Com_S_d = logData(:, 126:127);
cDDot_R_S_d = logData(:, 128:129); cDDot_L_S_d = logData(:, 130:131);
cDDot_Com_S_d = logData(:, 132:133);
u_R_A = logData(:, 134:135); u_L_A = logData(:, 136:137); 
c_Com_W = logData(:, 138:139); cDot_Com_W = logData(:, 140:141); 
fc_grf_R_S = logData(:, 142:143); fc_grf_L_S = logData(:, 144:145); 
fc_grf_R_S_ff = logData(:, 146:147); fc_grf_L_S_ff = logData(:, 148:149);
hgDotRef = logData(:, 150:152);
qfDDotRef = logData(:, 153:155);
cPfDDotRef = logData(:, 156:159);
forcePfRef = logData(:, 160:163);
% omegaNew = logData(:, 164:181);
time_RoDy = logData(:, 164); time_TcUpdate = logData(:, 165); time_WBC = logData(:, 166);
cDDot_U_S_d = logData(:, 167:168);
xRddot_CL = logData(:, 182); zRddot_CL = logData(:, 183); xLddot_CL = logData(:, 184); zLddot_CL = logData(:, 185);
timeCs = logData(:, 186); Time = logData(:, 187);
t_ = logData(:, 188); s = logData(:, 189); tt = logData(:, 190); ss = logData(:, 191); t_td = logData(:, 192); s_td = logData(:, 193);
s_R = logData(:, 194); s_L = logData(:, 195);
vx_cmd = logData(:, 196); vx_tgt = logData(:, 197);
H_tgt = logData(:, 198); H_ret = logData(:, 199); H_tgt_nestStep = logData(:, 200); H_ret_nestStep = logData(:, 201);
Ts = logData(:, 202); Td = logData(:, 203); Ts_nestStep = logData(:, 204); Td_nestStep = logData(:, 205);
vx_est = logData(:, 206); vx_est_nf = logData(:, 207); vx_full_pre = logData(:, 208); vx_double = logData(:, 209); x_est_total = logData(:, 210);
vz_est = logData(:, 211); vz_est_nf = logData(:, 212); vz_full_pre = logData(:, 213); z_est_total = logData(:, 214);
tick = logData(:, 215); steps_total = logData(:, 216); state = logData(:, 217); stanceLeg = logData(:, 218);
remark_flag = logData(:, 219); TD_flag = logData(:, 220); H_delta_cmd = logData(:, 221); deltaH_signal = logData(:, 222);
emergency_flag = logData(:, 223);
simpleStatus = logData(:, 224); nWSR_res = logData(:, 225); time_QP = logData(:, 226); cputime = logData(:, 227);

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
mean_RoDy = mean(time_RoDy)
mean_TcUpdate = mean(time_TcUpdate)
mean_QP = mean(time_QP)
mean_cputime = mean(cputime)*1e3
mean_WBC = mean(time_WBC)

% ------------------------------- plot ------------------------------

% t = Time;
t = timeCs;
ax = [];

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
plot(t, 50*stanceLeg, 'y--');
plot(t, fc_grf_L_S(:,1), 'k:');
plot(t, fc_grf_L_S_ff(:,1), 'm');
plot(t, fPf_opt(:,3), 'b');
title('hGRF Left');
xlabel('t (sec)');
ylabel('force (N)');
legend('50*stanceLeg', 'hGRF-Left-act', 'hGRF-Left-ff', 'hGRF-Left-des');  
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
plot(t, 50*stanceLeg, 'y--');
plot(t, fc_grf_R_S(:,1), 'k:');
plot(t, fc_grf_R_S_ff(:,1), 'm');
plot(t, fPf_opt(:,1), 'b');
title('hGRF Right');
xlabel('t (sec)');
ylabel('force (N)');
legend('50*stanceLeg', 'hGRF-Right-act', 'hGRF-Right-ff', 'hGRF-Right-des');  
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
plot(t, 0.5*stanceLeg, 'y--');
plot(t, cDot_Com_S_d(:,1), 'b');
plot(t, cDot_Com_S(:,1), 'k');
plot(t, cDot_Com_S_d(:,1) - cDot_Com_S(:,1), 'r');
title('S Com vx');
xlabel('t (s)');
ylabel('vx (m/s)');
legend('0.5*stanceLeg', 'xDot-Com-S-d', 'xDot-Com-S', 'xDot-Com-S-d - xDot-Com-S');   
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
plot(t, 0.01*stanceLeg, 'y--');
plot(t, cDot_U_S_d(:,2), 'b');
plot(t, cDot_U_S(:,2), 'k');
plot(t, cDot_U_S_d(:,2) - cDot_U_S(:,2), 'r');
title('S U vz');
xlabel('t (s)');
ylabel('vz (m/s)');
legend('0.01*stanceLeg', 'zDot-U-S-d', 'zDot-U-S', 'zDot-U-S-d - zDot-U-S'); 
f.Name = 'S U pos vel';

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
title('W error R x');
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

% G foot pos L R
f = figure;clf; 
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(2, 2, 1)]; 
hold on;
plot(t, 0.1*stanceLeg, 'y--');
plot(t, c_L_G_d(:,1), 'b');
plot(t, c_L_G(:,1), 'k');
plot(t, c_L_G_d(:,1) - c_L_G(:,1), 'r');
title('G footL x');
xlabel('t (s)');
ylabel('x (m)');
legend('0.1*stanceLeg', 'x-L-G-d', 'x-L-G', 'x-L-G-d - x-L-G'); 
ax = [ax, subplot(2, 2, 2)]; 
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, c_L_G_d(:,2), 'b');
plot(t, c_L_G(:,2), 'k');
plot(t, c_L_G_d(:,2) - c_L_G(:,2), 'r');
title('G footL z');
xlabel('t (s)');
ylabel('z (m)');
legend('0.5*stanceLeg', 'z-L-G-d', 'z-L-G', 'z-L-G-d - z-L-G'); 
ax = [ax, subplot(2, 2, 3)];
hold on;
plot(t, 0.1*stanceLeg, 'y--');
plot(t, c_R_G_d(:,1), 'b');
plot(t, c_R_G(:,1), 'k');
plot(t, c_R_G_d(:,1) - c_R_G(:,1), 'r');
title('G footR x');
xlabel('t (s)');
ylabel('x (m)');
legend('0.1*stanceLeg', 'x-R-G-d', 'x-R-G', 'x-R-G-d - x-R-G');   
ax = [ax, subplot(2, 2, 4)];
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, c_R_G_d(:,2), 'b');
plot(t, c_R_G(:,2), 'k');
plot(t, c_R_G_d(:,2) - c_R_G(:,2), 'r');
title('G footR z');
xlabel('t (s)');
ylabel('z (m)');
legend('0.5*stanceLeg', 'z-R-G-d', 'z-R-G', 'z-R-G-d - z-R-G'); 
f.Name = 'G foot pos L R';

% G foot error L R
f = figure;clf;   
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(2, 2, 1)]; 
hold on;
plot(t, 0.1*stanceLeg, 'y--');
plot(t, c_L_G_d(:,1), 'b');
plot(t, c_L_G(:,1), 'k');
title('G L x');
xlabel('t (s)');
ylabel('x (m)');
legend('0.1*stanceLeg', 'x-L-G-d', 'x-L-G'); 
ax = [ax, subplot(2, 2, 2)]; 
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, c_L_G_d(:,2), 'b');
plot(t, c_L_G(:,2), 'k');
title('G L z');
xlabel('t (s)');
ylabel('z (m)');
legend('0.5*stanceLeg', 'z-L-G-d', 'z-L-G'); 
ax = [ax, subplot(2, 2, 3)];
hold on;
plot(t, 0.01*stanceLeg, 'y--');
plot(t, c_L_G_d(:,1) - c_L_G(:,1), 'r');
title('G error R x');
xlabel('t (s)');
ylabel('x (m)');
legend('0.01*stanceLeg', 'x-L-G-d - x-L-G');   
ax = [ax, subplot(2, 2, 4)];
hold on;
plot(t, 0.01*stanceLeg, 'y--');
plot(t, c_L_G_d(:,2) - c_L_G(:,2), 'r');
title('G error L z');
xlabel('t (s)');
ylabel('z (m)');
legend('0.01*stanceLeg', 'z-L-G-d - z-L-G'); 
f.Name = 'G foot error L R';

% G foot vel L R
f = figure;clf;   
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');
ax = [ax, subplot(2, 2, 1)]; 
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, cDot_L_G_d(:,1), 'b');
plot(t, cDot_L_G(:,1), 'k');
plot(t, cDot_L_G_d(:,1) - cDot_L_G(:,1), 'r');
title('G footL vx');
xlabel('t (s)');
ylabel('vx (m/s)');
legend('0.5*stanceLeg', 'xDot-L-G-d', 'xDot-L-G', 'xDot-L-G-d - xDot-L-G'); 
ax = [ax, subplot(2, 2, 2)]; 
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, cDot_L_G_d(:,2), 'b');
plot(t, cDot_L_G(:,2), 'k');
plot(t, cDot_L_G_d(:,2) - cDot_L_G(:,2), 'r');
title('G footL vz');
xlabel('t (s)');
ylabel('vz (m/s)');
legend('0.5*stanceLeg', 'zDot-L-G-d', 'zDot-L-G', 'zDot-L-G-d - zDot-L-G'); 
ax = [ax, subplot(2, 2, 3)];
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, cDot_R_G_d(:,1), 'b');
plot(t, cDot_R_G(:,1), 'k');
plot(t, cDot_R_G_d(:,1) - cDot_R_G(:,1), 'r');
title('G footR vx');
xlabel('t (s)');
ylabel('vx (m/s)');
legend('0.5*stanceLeg', 'xDot-R-G-d', 'xDot-R-G', 'xDot-R-G-d - xDot-R-G');   
ax = [ax, subplot(2, 2, 4)];
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, cDot_R_G_d(:,2), 'b');
plot(t, cDot_R_G(:,2), 'k');
plot(t, cDot_R_G_d(:,2) - cDot_R_G(:,2), 'r');
title('G footR vz');
xlabel('t (s)');
ylabel('vz (m/s)');
legend('0.5*stanceLeg', 'zDot-R-G-d', 'zDot-R-G', 'zDot-R-G-d - zDot-R-G'); 
f.Name = 'G foot vel L R';

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
plot(t, 0.5*stanceLeg, 'y--');
plot(t, qfDDotRef(:,1), 'b');
title('x-ddot UpperBody');
xlabel('t (s)');
ylabel('x-ddot (m/s^2)');
legend('0.5*stanceLeg', 'x-ddot-UB-Ref'); 
ax = [ax, subplot(1, 3, 2)]; 
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, qfDDotRef(:,2), 'b');
title('z-ddot UpperBody');
xlabel('t (s)');
ylabel('z-ddot (m/s^2)');
legend('0.5*stanceLeg', 'z-ddot-UB-Ref'); 
ax = [ax, subplot(1, 3, 3)]; 
hold on;
plot(t, 0.5*stanceLeg, 'y--');
plot(t, qfDDotRef(:,3), 'b');
title('pitch-ddot UpperBody');
xlabel('t (s)');
ylabel('pitch-ddot (rad/s^2)');
legend('0.5*stanceLeg', 'pitch-ddot-UB-Ref'); 
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
title('x-ddot PointFoot Left');
xlabel('t (s)');
ylabel('x-ddot (m/s^2)');
legend('10*stanceLeg', 'x-ddot-PF-Ref'); 
ax = [ax, subplot(2, 2, 2)]; 
hold on;
plot(t, 10*stanceLeg, 'y--');
plot(t, cPfDDotRef(:,4), 'b');
title('z-ddot PointFoot Left');
xlabel('t (s)');
ylabel('z-ddot (m/s^2)');
legend('10*stanceLeg', 'z-ddot-PF-Ref'); 
ax = [ax, subplot(2, 2, 3)]; 
hold on;
plot(t, 10*stanceLeg, 'y--');
plot(t, cPfDDotRef(:,1), 'b');
title('x-ddot PointFoot Right');
xlabel('t (s)');
ylabel('x-ddot (m/s^2)');
legend('10*stanceLeg', 'x-ddot-PF-Ref'); 
ax = [ax, subplot(2, 2, 4)]; 
hold on;
plot(t, 10*stanceLeg, 'y--');
plot(t, cPfDDotRef(:,2), 'b');
title('z-ddot PointFoot Right');
xlabel('t (s)');
ylabel('z-ddot (m/s^2)');
legend('10*stanceLeg', 'z-ddot-PF-Ref'); 
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
title('fx PointFoot');
xlabel('t (s)');
ylabel('fx (N)');
legend('10*stanceLeg', 'fx-PF-Ref'); 
ax = [ax, subplot(2, 2, 2)]; 
hold on;
plot(t, 200*stanceLeg, 'y--');
plot(t, forcePfRef(:,4), 'b');
title('fz PointFoot');
xlabel('t (s)');
ylabel('fz (N)');
legend('200*stanceLeg', 'fz-PF-Ref'); 
ax = [ax, subplot(2, 2, 3)]; 
hold on;
plot(t, 10*stanceLeg, 'y--');
plot(t, forcePfRef(:,1), 'b');
title('fx PointFoot');
xlabel('t (s)');
ylabel('fx (N)');
legend('10*stanceLeg', 'fx-PF-Ref'); 
ax = [ax, subplot(2, 2, 4)]; 
hold on;
plot(t, 200*stanceLeg, 'y--');
plot(t, forcePfRef(:,2), 'b');
title('fz PointFoot');
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
plot(t, vx_full_pre, 'r');
plot(t, vx_double, 'k');
title('vx estimation');
xlabel('t (sec)');
ylabel('vx (m/s)');
legend('stanceLeg', 'vx-cmd', 'vx-tgt', 'vx-est', 'vx-nf', 'vx-full-pre', 'vx-double');       
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
legend('0.001*stanceLeg', 'vz-est', 'vz-nf');       
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
plot(t, 0.1*stanceLeg, 'y--');
plot(t, rad2deg(pitch), 'k');
plot(t, rad2deg(pitch_d), 'b');
title('pitch');
xlabel('t (s)');
ylabel('angle (deg)');
legend('0.1*stanceLeg', 'pitch-act', 'pitch-des'); 
ax = [ax, subplot(1, 2, 2)]; 
hold on;
plot(t, stanceLeg, 'y--');
plot(t, rad2deg(pitchDot), 'k');
plot(t, rad2deg(pitchDot_d), 'b');
title('pitch-vel');
xlabel('t (s)');
ylabel('angle-vel (deg/s)');
legend('stanceLeg', 'pitchDot-act', 'pitchDot-des'); 
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
plot(t, vx_full_pre, 'k');
% plot(t, vx_double, 'k');
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

% Ts Td vs time
% f = figure;clf;   
% % data cursor setting
% dcm_obj = datacursormode(gcf);
% set(dcm_obj,'UpdateFcn',@NewCallback);
% % end of data cursor setting
% set(f, 'WindowStyle', 'docked');
% ax = [ax, axes(f)];
% hold on;
% plot(t, Ts, 'r');
% plot(t, Td, 'b');
% plot(t, vx_tgt, 'm--');
% title('Ts,Td vs time');
% xlabel('t (sec)');
% ylabel('TsTd | v');
% legend( 'Ts', 'Td', 'vx-tgt');       
% f.Name = 'Ts Td vs time';

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
plot(t, time_QP - cputime*1e3, 'r');
title('Error');
xlabel('t (s)');
ylabel('error (ms) ');
legend( 'time-QP - cputime' );  
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
plot(t, time_RoDy,'m--');
plot(t, time_TcUpdate, 'r');
plot(t, time_QP,'g--');
plot(t, time_WBC, 'k');
title('time');
xlabel('t (sec)');
ylabel('t (ms)');
legend( 'time-RoDy', 'time-TcUpdate', 'time-QP', 'time-WBC');       
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


Rec = plotJointTorqueVelocity(logData);

