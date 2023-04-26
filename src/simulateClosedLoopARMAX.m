function [y, u] = simulateClosedLoopARMAX(A, B, C, R, Q, S, k, w, sigmaesq, y0, u0, N)
% Polynomial orders
na = numel(A);
nb = numel(B);
nc = numel(C);
ns = numel(S);
nr = numel(R);

% Required memory
n = max([na, nb+k, nc, ns, nr]);

% Create actual input polynomial
Bbar = [zeros(size(B, 1), k), B];

% Generate noise realization
e = sqrt(sigmaesq)*randn(1, N + n);

% Allocate memory
y = zeros(1, N + n);
u = zeros(1, N + n);

% Insert initial values
y(1:n-1) = y0;
u(1:n-1) = u0;

for t = n + (0:N)
    % Extract noise, inputs, and outputs
    yt = y(:, t - (1:na  -1));
    ut = u(:, t - (0:nb+k-1));
    et = e(:, t - (0:nc  -1));
    
    % Compute new outputs
    y(:, t) = (C*et' + Bbar*ut' - A(2:end)*yt')/A(1);
    
    % Extract setpoints and new inputs and outputs
    wt = w(:, t - (0:nc-1));
    yt = y(:, t - (0:ns-1));
    ut = u(:, t - (1:nr-1));
    
    % Compute new inputs
    u(:, t) = (Q*wt' - S*yt' - R(2:end)*ut')/R(1);
end
end