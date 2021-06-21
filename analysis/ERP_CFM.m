% computing Error Reduction Parameter & Constraint Force Mixing

Kp = 10000;
Kd = 100;
h = 5e-3;

ERP = h*Kp/(h*Kp+Kd)
CFM = 1/(h*Kp+Kd)

%%
Kp = 20000;
Kd = 200;
h = 1e-3;

ERP = h*Kp/(h*Kp+Kd)
CFM = 1/(h*Kp+Kd)