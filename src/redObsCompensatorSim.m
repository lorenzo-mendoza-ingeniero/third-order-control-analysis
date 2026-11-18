function res = redObsCompensatorSim(sys, p_cont, p_obs_red, varargin)

    A = sys.A; B = sys.B; C = sys.C; D = sys.D;
    n = size(A,1); ny = size(C,1);

    % ---- Verificaciones ----
    if rank(ctrb(A,B)) ~= n
        error('Sistema no controlable.');
    end
    if rank(obsv(A,C)) ~= n
        error('Sistema no observable.');
    end

    % ---- Ganancia de realimentación ----
    K = place(A, B, p_cont);

    % ---- Transformación para obtener C = [I_ny 0] ----
    % Construimos P usando la base del espacio nulo de C
    Nc = null(C);                 % n × (n-ny)
    if isempty(Nc)
        error('C tiene rango completo y ny = n, no se requiere observador reducido.');
    end
    P = [C; Nc'];                 % n × n (primeras filas = C)
    % Verificar invertibilidad
    if rank(P) < n
        error('No se pudo construir una matriz de transformación adecuada.');
    end

    A_tilde = P * A / P;
    B_tilde = P * B;
    C_tilde = C / P;              % = [I_ny, 0]

    % Partición
    A11 = A_tilde(1:ny, 1:ny);
    A12 = A_tilde(1:ny, ny+1:end);
    A21 = A_tilde(ny+1:end, 1:ny);
    A22 = A_tilde(ny+1:end, ny+1:end);
    B1  = B_tilde(1:ny, :);
    B2  = B_tilde(ny+1:end, :);

    % ---- Ganancia del observador reducido ----
    Lr = place(A22', A12', p_obs_red)';   % (n-ny) × ny

    % ---- Prefiltro ----
    if sys.Ts == 0
        N = 1 / (C * (-(A - B*K) \ B));
    else
        N = 1 / (C * ((eye(n) - (A - B*K)) \ B));
    end

    % ---- Sistema lazo cerrado (estados [x; z]) ----
    % Matrices de reconstrucción en coordenadas originales
    P_inv = P \ eye(n);
    T_y = P_inv(:, 1:ny);        % contribución de y
    S   = P_inv(:, ny+1:end);    % contribución de parte no medida

    % Control: u = -K * ( T_y * y + S * (z + Lr * y) ) + N r
    %           = -K * ( (T_y + S*Lr) * C * x + S * z ) + N r
    u_x = -K * ( (T_y + S*Lr) * C );    % retroalimentación de x
    u_z = -K * S;                       % retroalimentación de z
    u_r = N;                            % prefiltro

    % Dinámica de x: dx = A x + B u  => A x + B (u_x x + u_z z + u_r r)
    A11_x = A + B * u_x;
    A12_z = B * u_z;

    % Dinámica de z:
    % dz = (A22-Lr*A12)z + ((A22-Lr*A12)Lr + A21 - Lr*A11) y + (B2 - Lr*B1) u
    M_y = (A22 - Lr*A12)*Lr + A21 - Lr*A11;  % ganancia sobre y
    % sustituyendo y = C x y u = u_x x + u_z z + u_r r
    A21_x = M_y * C + (B2 - Lr*B1) * u_x;
    A22_z = A22 - Lr*A12 + (B2 - Lr*B1) * u_z;

    A_aug = [A11_x, A12_z; A21_x, A22_z];
    B_aug = [B * u_r; (B2 - Lr*B1) * u_r];
    C_aug = [C, zeros(ny, n-ny)];
    D_aug = 0;

    sys_cl = ss(A_aug, B_aug, C_aug, D_aug, sys.Ts);

    % ---- Condiciones de simulación ----
    if nargin >= 7
        r = varargin{1};
        t = varargin{2};
        X0_plant = varargin{3};
        z0 = varargin{4};
    else
        % Valores por defecto (ajustar según el sistema)
        t = 0:0.01:3;
        r = 1.6 * ones(size(t));       % referencia pequeña
        X0_plant = [0.5; -0.2; 0.1];
        z0 = zeros(n-ny, 1);
    end

    X0_aug = [X0_plant; z0];

    % ---- Simulación ----
    [y, t, X_aug] = lsim(sys_cl, r, t, X0_aug);
    x = X_aug(:, 1:n);
    z = X_aug(:, n+1:end);

    % Reconstruir el estado estimado completo
    y_sim = y;          % ya es columna
    xhat = zeros(length(t), n);
    for k = 1:length(t)
        xhat(k,:) = P \ [y_sim(k)'; z(k,:)' + Lr * y_sim(k)'];
    end

    % Señal de control
    u = zeros(length(t), size(B,2));
    for k = 1:length(t)
        u(k,:) = -K * xhat(k,:)' + N * r(k);
    end

    % ---- Gráficos automáticos ----
    figure('Name','Observador de orden reducido + compensador');

    subplot(3,1,1);
    plot(t, y, 'b-', 'LineWidth', 1.5);
    grid on; ylabel('y(t)'); title('Salida del sistema');
    xlabel('Tiempo (s)');

    subplot(3,1,2);
    plot(t, x(:,1:min(3,n)), 'LineWidth', 1.2); hold on;
    plot(t, xhat(:,1:min(3,n)), '--', 'LineWidth', 1.2);
    grid on; ylabel('Estados');
    legend('x_1 real','x_2 real','x_3 real','xhat_1','xhat_2','xhat_3', ...
           'Location','best');
    title('Comparación de estados (real vs estimado)');

    subplot(3,1,3);
    plot(t, u, 'r-', 'LineWidth', 1.2);
    grid on; ylabel('u(t)'); xlabel('Tiempo (s)');
    title('Esfuerzo de control');

    % ---- Empaquetar resultados ----
    res.K = K; res.Lr = Lr; res.N = N; res.P = P; res.sys_cl = sys_cl;
    res.t = t; res.y = y; res.x = x; res.z = z; res.xhat = xhat; res.u = u;
end