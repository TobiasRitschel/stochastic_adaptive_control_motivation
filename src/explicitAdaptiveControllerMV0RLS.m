function [y, u, theta, P] = explicitAdaptiveControllerMV0RLS(A, B, C, k, w, sigmaesq, y0, u0, theta0, P0, N)
% Polynomial orders
na = numel(A);
nb = numel(B);
nc = numel(C);

% Required memory
n = max([na, nb+k, nc]);

% Number of parameters
ntheta = na + nb - 1;

% Generate noise realizations
e = sqrt(sigmaesq)*randn(1, N + n);

% Allocate memory
y     = zeros(1,              N + n);
u     = zeros(1,              N + n);
theta = zeros(ntheta,         N + n);
P     = zeros(ntheta, ntheta, N + n);

% Initialize
thetatm1 = theta0;
Ptm1     = P0;

% Store initial values
y    (:, 1:n-1) = y0;
u    (:, 1:n-1) = u0;
for i = 1:n-1
    theta(:,    i) = theta0;
    P    (:, :, i) = P0;
end

for t = n + (0:N)
    %% Simulate (closed-loop simulation)
    % Extract noise, inputs, and outputs
    Ytm1 = y(:, t -     (1:na-1));
    Utmk = u(:, t - k - (0:nb-1));
    Et   = e(:, t -     (0:nc-1));
    
    % Compute new outputs
    yt = (C*Et' + B*Utmk' - A(2:end)*Ytm1')/A(1);
    
    % Store outputs
    y(:, t) = yt;

    %% Identify (system identification)
    % Create vector of regressors
    phit = [-Ytm1'; Utmk'];
    
    % Residual
    epsilon = yt - phit'*thetatm1;
    s       = 1 + phit'*Ptm1*phit;
    K       = Ptm1*phit/s;
    thetat  = thetatm1 + K*epsilon;
    Pt      = Ptm1 - K*s*K';
            if(mod(t, 100) == 0), Pt = Pt*2; end % Reset covariance periodically
    % Store parameter estimates and covariances
    theta(:,    t) = thetat;
    P    (:, :, t) = Pt;

    %% Design (controller design)
    % Initial estimates of polynomials
    Ahat = [1, reshape(thetat(1:na-1),        1, na-1)];
    Bhat =     reshape(thetat(na-1 + (1:nb)), 1, nb  );
    Chat =  1; % By assumption

    % Solve the Diophantine equation
    [Ghat, Shat] = solveDiophantineEquation(Ahat, Chat, k);

    % Coefficients of polynomials in control law
    Rhat = conv(Bhat, Ghat);
    Qhat = Chat;

    %% Control
    % Polynomial orders
    nahat = numel(Ahat); %#ok
    nbhat = numel(Bhat); %#ok
    nchat = numel(Chat);
    nshat = numel(Shat);
    nrhat = numel(Rhat);

    % Extract setpoints, outputs, and inputs
    Wt   = w(:, t - (0:nchat-1));
    Yt   = y(:, t - (0:nshat-1));
    Utm1 = u(:, t - (1:nrhat-1));
    
    % Compute new inputs
    ut = (Qhat*Wt' - Shat*Yt' - Rhat(2:end)*Utm1')/Rhat(1);

    % Store output
    u(:, t) = ut;

    %% Prepare next iteration
    % Update initial estimates
    thetatm1 = thetat;
    Ptm1     = Pt;
end