function [Rec] = plotJointTorqueVelocity(logData)

%
% parse & mapping
%
qDot_ = logData(:, 12:22); 
tauA_opt = logData(:, 60:63);
timeCs = logData(:, 186);

t = timeCs;

qdot_FR = qDot_(:,4);
qdot_RR = qDot_(:,6);
qdot_FL = qDot_(:,8);
qdot_RL = qDot_(:,10);

tauDes_FR = tauA_opt(:,1);
tauDes_RR = tauA_opt(:,2);
tauDes_FL = tauA_opt(:,3);
tauDes_RL = tauA_opt(:,4);

Rec = [tauDes_FR, tauDes_RR, tauDes_FL, tauDes_RL,...
        qdot_FR, qdot_RR, qdot_FL, qdot_RL,...
        t];
    
uMax = 35;
qDotMax = 12;

ax = [];

f = figure;clf;   
% data cursor setting
dcm_obj = datacursormode(gcf);
set(dcm_obj,'UpdateFcn',@NewCallback);
% end of data cursor setting
set(f, 'WindowStyle', 'docked');

ax1 = [ax, subplot(2, 2, 1)]; 
yyaxis(ax1, 'left');
plot(t, tauDes_RL);
xlabel('t (sec)');
ylabel('torque (Nm)');
xlim([0,30]);
ylim([-uMax,uMax]);
yyaxis(ax1,'right');
plot(t, qdot_RL);
ylabel('velocity (rad/s)');
ylim([-qDotMax,qDotMax]);
title('joint RearLeft');

ax2 = [ax, subplot(2, 2, 2)]; 
yyaxis(ax2, 'left');
plot(t, tauDes_FL);
xlabel('t (sec)');
ylabel('torque (Nm)');
xlim([0,30]);
ylim([-uMax,uMax]);
yyaxis(ax2,'right');
plot(t, qdot_FL);
ylabel('velocity (rad/s)');
ylim([-qDotMax,qDotMax]);
title('joint ForeLeft');

ax3 = [ax, subplot(2, 2, 3)]; 
yyaxis(ax3, 'left');
plot(t, tauDes_RR);
xlabel('t (sec)');
ylabel('torque (Nm)');
xlim([0,30]);
ylim([-uMax,uMax]);
yyaxis(ax3,'right');
plot(t, qdot_RR);
ylabel('velocity (rad/s)');
ylim([-qDotMax,qDotMax]);
title('joint RearRight');


ax4 = [ax, subplot(2, 2, 4)]; 
yyaxis(ax4, 'left');
plot(t, tauDes_FR);
xlabel('t (sec)');
ylabel('torque (Nm)');
xlim([0,30]);
ylim([-uMax,uMax]);
yyaxis(ax4,'right');
plot(t, qdot_FR);
ylabel('velocity (rad/s)');
ylim([-qDotMax,qDotMax]);
title('joint ForeRight');

f.Name = 'joint analysis';

% Synchronize limits of specified 2-D axes
linkaxes([ax1,ax2,ax3,ax4], 'x');

end
