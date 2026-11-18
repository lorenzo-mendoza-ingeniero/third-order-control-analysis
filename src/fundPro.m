function result = fundPro(sys, Ts, tol)
% Number of models in the array
nsys = numel(sys);

if nargin < 2 || isempty(Ts)
    Ts = 0;
end
if nargin < 3
    tol = 1e-9;
end

n = order(sys(:, :, 1)); 

% Preallocating output structure
eigenvalues = zeros(n, nsys);
stable      = false(1, nsys);
controllable = false(1, nsys);
observable   = false(1, nsys);

for i = 1:nsys
    % Extract the i‑th model
    sys_i = sys(:, :, i);

    % System matrices
    A = sys_i.A;
    B = sys_i.B;
    C = sys_i.C;
    n = order(sys_i);      % number of states

    % 1. Stability
    e = eig(A);
    eigenvalues(:, i) = e;
    if Ts == 0
        stable(i) = all(real(e) < -tol);
    else
        stable(i) = all(abs(e) < 1 - tol);
    end

    % 2. Controllability
    % Build Co = [B, A*B, A^2*B, ..., A^(n-1)*B]
    Co = B;
    AkB = B;
    for k = 1:n-1
        AkB = A * AkB;
        Co = [Co, AkB];
    end
    r_co = rank(Co, tol);
    controllable(i) = (r_co == n);

    % 3. Observability
    % Build Ob = [C; C*A; C*A^2; ...; C*A^(n-1)]
    Ob = C;
    CAk = C;
    for k = 1:n-1
        CAk = CAk * A;
        Ob = [Ob; CAk];
    end
    r_ob = rank(Ob, tol);
    observable(i) = (r_ob == n);
end

% Pack results into the output structure
result.eigenvalues = eigenvalues;
result.stable      = stable;
result.controlable = controllable;
result.observable  = observable;
end