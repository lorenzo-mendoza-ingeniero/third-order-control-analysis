function res = plotSFCLservomechanism(sys_ol, eigenvalues)

    n = order(sys_ol);

    A = sys_ol.A;
    B = sys_ol.B;
    C = sys_ol.C;
    D = sys_ol.D;
    
    X0 = [0; 0; 0];
    X0_aug = [0; 0; 0; 0];
    t = 0:0.001:4;
    U = 1.6 * ones(size(t));

    A_aug = [A, zeros(n,1);  -C, 0];
    B_aug = [B; zeros(1, size(B,2))];
    K_aug = acker(A_aug, B_aug, eigenvalues);

    Kx = K_aug(1:n);
    ki = K_aug(end);

    N = 1;

    Acl = [A - B*Kx, -B*ki; -C, 0];
    Bcl = [B*N; 1];
    Ccl = [C, 0];
    Dcl = 0;

    sys_cl = ss(Acl, Bcl, Ccl, Dcl);

    [Yo, t, X_ol] = lsim(sys_ol, U, t, X0);
    [Yc,t,X_cl] = lsim(sys_cl,U,t,X0_aug);

figure('Name', 'Comparación OL vs CL con integrador');

for i = 1:3
    subplot(4,1,i);
    yyaxis left;
    plot(t, X_ol(:,i), '--r', 'LineWidth', 1.5);
    ylabel(sprintf('x_%d OL', i));
    
    yyaxis right;
    plot(t, X_cl(:,i), '-b', 'LineWidth', 1.5);
    ylabel(sprintf('x_%d CL', i));
    
    grid on; xlabel('Tiempo (s)'); 
    title(sprintf('Estado %d', i));
    legend('Open-loop', 'Closed-loop', 'Location', 'best');
end

subplot(4,1,4);
plot(t, X_cl(:,4), 'b-', 'LineWidth', 1.5);
grid on; xlabel('Tiempo (s)'); ylabel('x_i');
title('Integrador (acción integral)');

res.K_aug = K_aug; res.sys_cl = sys_cl;

end