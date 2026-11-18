function res = fullObsCompensatorSim(sys, p_cont, p_obs, varargin)

    A = sys.A; B = sys.B; C = sys.C; D = sys.D;
    n = size(A,1);
    
    % ---- Verificaciones ----
    if rank(ctrb(A,B)) ~= n
        error('Sistema no controlable.');
    end
    if rank(obsv(A,C)) ~= n
        error('Sistema no observable.');
    end
    
    % ---- Ganancias ----
    K = place(A, B, p_cont);
    L = place(A', C', p_obs)';
    
    % Prefiltro (seguimiento de escalón sin error estático)
    if sys.Ts == 0
        N = 1 / (C * (-(A - B*K) \ B));
    else
        N = 1 / (C * ((eye(n) - (A - B*K)) \ B));
    end
    
    % ---- Sistema lazo cerrado (2n estados: [x; xhat]) ----
    Acl = [A,           -B*K;
        L*C,  A - B*K - L*C];
    Bcl = [B*N; B*N];
    Ccl = [C, zeros(size(C,1), n)];
    Dcl = 0;
    
    sys_cl = ss(Acl, Bcl, Ccl, Dcl, sys.Ts);
    
    % ---- Condiciones de simulación ----
    if nargin >= 7
        r = varargin{1};
        t = varargin{2};
        X0_plant = varargin{3};
        X0_obs   = varargin{4};
    else
        % Valores por defecto
        t = 0:0.01:3;
        r = 1.6 * ones(size(t));
        X0_plant = [0.2; zeros(n-1,1)];
        X0_obs   = zeros(n,1);
    end
    
    X0_aug = [X0_plant; X0_obs];
    
    % ---- Simulación ----
    [y, t, X_aug] = lsim(sys_cl, r, t, X0_aug);
    x    = X_aug(:, 1:n);
    xhat = X_aug(:, n+1:end);
    
    % Señal de control (reconstruida)
    u = zeros(length(t), size(B,2));
    for k = 1:length(t)
        u(k,:) = -K * xhat(k,:)' + N * r(k);
    end
    
    % ---- Gráficos automáticos ----
    figure('Name','Observador de orden completo + compensador');
    
    % Salida
    subplot(3,1,1);
    plot(t, y, 'b-', 'LineWidth', 1.5);
    grid on; ylabel('y(t)'); title('Salida del sistema');
    xlabel('Tiempo (s)');
    
    % Estados (solo 3 primeros para visualización)
    subplot(3,1,2);
    plot(t, x(:,1:min(3,n)), 'LineWidth', 1.2); hold on;
    plot(t, xhat(:,1:min(3,n)), '--', 'LineWidth', 1.2);
    grid on; ylabel('Estados');
    legend('x_1 real','x_2 real','x_3 real','xhat_1','xhat_2','xhat_3', ...
        'Location','best');
    title('Comparación de estados (real vs estimado)');
    
    % Esfuerzo de control
    subplot(3,1,3);
    plot(t, u, 'r-', 'LineWidth', 1.2);
    grid on; ylabel('u(t)'); xlabel('Tiempo (s)');
    title('Esfuerzo de control');
    
    % ---- Empaquetar resultados ----
    res.K = K; res.L = L; res.N = N; res.sys_cl = sys_cl;
    res.t = t; res.y = y; res.x = x; res.xhat = xhat; res.u = u;
    
end

