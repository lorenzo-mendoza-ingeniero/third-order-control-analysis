clc; clearvars;

% Rtol_inf = -5; Rtol_sup = +5;
% Res = 100e3;
% R = zeros(4, 1);
% for i = 1:4
%     Rtol = (Rtol_inf + (Rtol_sup - Rtol_inf) * rand(1, 1)) / 100;
%     R(i, 1) = (1 + Rtol) * Res;
% end
% 
% Ctol_inf = -10; Ctol_sup = +10;
% Cap = 1e-6;
% C = zeros(3, 1);
% for i = 1:3
%     Ctol = (Ctol_inf + (Ctol_sup - Ctol_inf) * rand(1, 1)) / 100;
%     C(i, 1) = (1 + Ctol) * Cap;
% end

R=[97e3; 196e3 ; 97.3e3; 196e3];
C=[1.061e-6; 2.31e-6; 1.271e-6];

R1 = R(1); R2 = R(2); R3 = R(3); R4 = R(4);
C1 = C(1); C2 = C(2); C3 = C(3);

A = [-1/(C1*R1), 0, 0;
    1/(C2*R2), -(R2+R3)/(C2*R2*R3), -1/(C2*R2);
    1/(C3*R2), -1/(C3*R2), -(R2+R4)/(C3*R2*R4)];

B = [1/(C1*R1); 0; 0];

C = [0, 0, 1];

D = 0;

sys_ol = ss(A, B, C, D);

res = fundPro(sys_ol);

% Display results
for i = 1:numel(sys_ol)
    fprintf('Model %d: stable=%d, controllable=%d, observable=%d\n', ...
        i, res.stable(i), res.controlable(i), res.observable(i));
end

eigenvalues_compensator_design = [-2 - 2i, -2 + 2i, -8];
eigenvalues_servomechanism_design = [-2 - 2i, -2 + 2i, -8, -10];
eigenvalues_fullorderobserver_design = [-9, -11, -14];
eigenvalues_reducedorderobserver_design = [-9, -11];

res_com = plotSFCLcompensator(sys_ol, eigenvalues_compensator_design);
res_serv = plotSFCLservomechanism(sys_ol, eigenvalues_servomechanism_design);
res_full = plotOFOLfullorder(sys_ol, eigenvalues_fullorderobserver_design);
res_red = plotOFOLreducedorder(sys_ol, eigenvalues_reducedorderobserver_design);
res_full_comp = fullObsCompensatorSim(sys_ol, eigenvalues_compensator_design, eigenvalues_fullorderobserver_design);
res_red_comp  = redObsCompensatorSim(sys_ol, eigenvalues_compensator_design, eigenvalues_reducedorderobserver_design);
