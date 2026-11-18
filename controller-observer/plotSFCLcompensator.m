function res = plotSFCLcompensator(sys_ol, eigenvalues)
    
    A = sys_ol.A;
    B = sys_ol.B;
    C = sys_ol.C;
    D = sys_ol.D;

    X0 = [0; 0; 0];
    t = 0:0.001:4;
    U = 1.6 * ones(size(t));
    
    K = place(A, B, eigenvalues);
    
    Ac = A - B * K; 
    Bc = B; 
    Cc = C; 
    Dc = D;

    sys_cl = ss(Ac, Bc, Cc, Dc);
    
    [Yo, t, Xo] = lsim(sys_ol, U, t, X0);
    [Yc,t,Xc] = lsim(sys_cl,U,t,X0);
    
    figure('Name', 'Comparación de estados OL vs CL');
    
    for i = 1:3
        subplot(3,1,i);
        
        yyaxis left;
        plot(t, Xo(:,i), '--r', 'LineWidth', 1.5);
        ylabel(sprintf('x_%d (OL)', i));
        
        yyaxis right;
        plot(t, Xc(:,i), '-b', 'LineWidth', 1.5);
        ylabel(sprintf('x_%d (CL)', i));
        
        grid on;
        xlabel('Tiempo (s)');
        title(sprintf('Estado %d', i));
        
        legend('Open-loop', 'Closed-loop', 'Location', 'best');
    end

    res.K = K; res.sys_cl = sys_cl;

end