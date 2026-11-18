function res = plotOFOLreducedorder(sys_ol, eigenvalues)

    n = order(sys_ol);
    
    A = sys_ol.A;
    B = sys_ol.B;
    C = sys_ol.C;
    D = sys_ol.D;

    ny = size(C,1); 

    P = [0 0 1;  1 0 0;  0 1 0]; 

    A_perm = P * A * P';
    B_perm = P * B;
    C_perm = C * P';

    A11 = A_perm(1:ny, 1:ny);
    A12 = A_perm(1:ny, ny+1:end);
    A21 = A_perm(ny+1:end, 1:ny);
    A22 = A_perm(ny+1:end, ny+1:end);

    B1 = B_perm(1:ny, :);
    B2 = B_perm(ny+1:end, :);

    Lr = place(A22', A12', eigenvalues)';

    A_aug = [A_perm, zeros(n, n-ny);
         ((A22 - Lr*A12)*Lr + A21 - Lr*A11), zeros(n-ny, n-ny), A22 - Lr*A12];
    B_aug = [B_perm;
        B2 - Lr*B1];
    C_aug = eye(n + (n-ny));
    D_aug = zeros(n + (n-ny), size(B,2));

    sys_aug = ss(A_aug, B_aug, C_aug, D_aug);

    % ---------- 5. Simulación con lsim ----------
    t = 0:0.001:4;              % tiempo de simulación (20 ms)
    u = zeros(size(t));             % entrada (respuesta libre)

    % Condiciones iniciales: planta (original) y estimador
    x0_original = [0.5; -0.2; 0.1]; % estado real inicial
    x0_perm = P * x0_original;     % en coordenadas permutadas
    z0 = zeros(n-ny, 1);           % estado inicial del observador (normalmente cero)

    X0_aug = [x0_perm; z0];        % vector inicial de 5 elementos

    % Simular
    Y_aug = lsim(sys_aug, u, t, X0_aug);  % Y_aug: columnas 1-3 = x_perm, 4-5 = z

    % Extraer señales
    x_perm = Y_aug(:, 1:n);        % T×3
    z = Y_aug(:, n+1:end);         % T×2

    % ---------- 6. Reconstruir los estados estimados ----------
    % Estimación en coordenadas permutadas:
    %   x1_hat = y (medida)
    %   x2_hat = z + Lr * y
    y = x_perm(:, 1);              % T×1 (primera componente de x_perm)
    x_perm_hat = [y, z + y * Lr']; % T×3, usando que y es columna y Lr' es 1×2

    % Transformar a coordenadas originales (x = P' * x_perm)
    x_real = x_perm * P;           % T×3 (porque P' = inv(P) = P^T y x_perm es T×3)
    x_est  = x_perm_hat * P;       % T×3

    % ---------- 7. Gráficos ----------
    figure('Name', 'Observador de orden reducido: real vs estimado');
    for i = 1:n
        subplot(n, 1, i);
        plot(t, x_real(:, i), 'b-', 'LineWidth', 1.5); hold on;
        plot(t, x_est(:, i), 'r--', 'LineWidth', 1.5);
        grid on;
        xlabel('Tiempo (s)');
        ylabel(sprintf('x_%d', i));
        legend('Real', 'Estimado', 'Location', 'best');
        title(sprintf('Estado %d', i));
    end

    res.Lr = Lr; res.sys_aug = sys_aug;
end
