function res = plotOFOLfullorder(sys_ol, eigenvalues)

    n = order(sys_ol);
    
    A = sys_ol.A;
    B = sys_ol.B;
    C = sys_ol.C;
    D = sys_ol.D;

    L = place(A', C', eigenvalues)';

    A_aug = [A, zeros(n); L*C, A - L*C];
    B_aug = [B; B];
    C_aug = eye(2*n);
    D_aug = zeros(2*n, size(B,2));

    sys_aug = ss(A_aug, B_aug, C_aug, D_aug);

    t = 0:0.001:4;
    u = zeros(size(t));

    x0_plant = [0.5; -0.2; 0.1];
    x0_obs   = [0; 0; 0];
    X0_aug = [x0_plant; x0_obs];

    Y_aug = lsim(sys_aug, u, t, X0_aug);

    X_real = Y_aug(:, 1:n);
    X_est  = Y_aug(:, n+1:end);

    % ---------- Gráficos ----------
    figure('Name','Observador de Orden Completo');
    for i = 1:n
        subplot(n,1,i);
        plot(t, X_real(:,i), 'b-', 'LineWidth', 1.5); hold on;
        plot(t, X_est(:,i), 'r--', 'LineWidth', 1.5);
        grid on;
        xlabel('Tiempo (s)'); ylabel(sprintf('x_%d', i));
        legend('Real', 'Estimado');
        title(sprintf('Estado %d', i));
    end
    
    res.L = L; res.sys_aug = sys_aug;
end