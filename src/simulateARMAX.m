function y = simulateARMAX(A, B, C, k, sigmaesq, y0, u, N)
% Polynomial orders
na = numel(A);
nb = numel(B);
nc = numel(C);

% Allocate memory
y =                zeros(1, N + na);
e = sqrt(sigmaesq)*randn(1, N + na);

% Insert initial condition
y(1:na-1) = y0;

for t = na + (0:N)
    % Extract noise and outputs
    yt = y(:, t -     (1:na-1));
    ut = u(:, t - k - (0:nb-1));
    et = e(:, t -     (0:nc-1));
    
    % Compute new outputs
    y(:, t) = (B*ut' + C*et' - A(2:end)*yt')/A(1);
end