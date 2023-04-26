%% Initialize
% Clear all variables
clear all; %#ok

% Close all figures
% close all;

% Clear command window
clc;

% Remove added paths
restoredefaultpath;

% Set renderer (otherwise, eps figures can become pixelated in Latex)
set(groot, 'DefaultFigureRenderer', 'Painters');

%% Formatting
% Save figures? (Disable to save time)
SaveFigures = true;

% Format of saved figures
Format = 'png';

% Font size
fs = 12;

% Line width
lw = 1;

% Marker size
ms = 20;

% Offset in figures (makes room for text)
Yoffset = 5;
Uoffset = 9;
Thetaoffset = 0.35;

% Set default font size
set(groot, 'DefaultAxesFontSize',   fs);

% Set default line widths
set(groot, 'DefaultLineLineWidth',  lw);
set(groot, 'DefaultStairLineWidth', lw);
set(groot, 'DefaultStemLineWidth',  lw);

% Set default marker size
set(groot, 'DefaultLineMarkerSize', ms);

% Set default interpreter
set(groot, 'DefaultTextInterpreter',            'Latex');
set(groot, 'DefaultAxesTickLabelInterpreter',   'Latex');
set(groot, 'DefaultLegendInterpreter',          'Latex');

%% Add library paths
% Add path to source files
run('../load_library');

%% Initialize
% Random number generator seed
rng(0);

%% System parameters
% Conversion factor
min2s = 60;         % min   -> s
s2min = 1/min2s;    % s     -> min
h2min = 60;         % h     -> min
min2h = 1/h2min;    % min   -> h

% Time sample
Ts = 20*s2min*min2h; % [h]

% Polynomial coefficients
A = [1, -0.9, 0.99];
B =  1;
C =  1;

% Input delay
k = 1;

% Noise variance
sigmaesq = 0.1;

% Maximum allowed output
Ymax = 5;

%% Design controller
% Solve the Diophantine equaiton
[G, S] = solveDiophantineEquation(A, C, k);

% Coefficients of polynomials in control law
R = conv(B, G);
Q = C;

% Polynomial orders
na = numel(A);
nb = numel(B);
nc = numel(C);
nr = numel(R);
ns = numel(S);
nq = numel(Q);

%% Open-loop simulation
% Required memory
nol = max([na, nb+k, nc]);

% Initial condition
y0ol = zeros(1, nol-1);

% Simulation horizon
N = round(6/Ts); % [#]

% Manipulated inputs
uol = zeros(1, N+nol);

% Open-loop simulation
yol = simulateARMAX(A, B, C, k, sigmaesq, y0ol, uol, N);

% Store solution
Y(1, :) = yol;
Y(2, :) = Y(1, :);

U(1, :) = uol;
U(2, :) = U(1, :);

Theta     = repmat([A(2:end)'; B'], 1, N+nol);
Theta_ref = Theta;

%% Closed-loop simulation of static controller (first parameter set)
% Required memory
ncl = max([na, nb+k, nc, ns, nr]);

% Initial values
y0cl = yol(end-nol+2:end);
u0cl = uol(end-nol+2:end);

% Setpoint
w = zeros(1, N+ncl);

% Closed-loop simulation
[ycl, ucl] = simulateClosedLoopARMAX ...
    (A, B, C, R, Q, S, k, w, sigmaesq, y0cl, u0cl, N);

%% Closed-loop simulation of adaptive controller (first parameter set)
% Initial values
y0ac = yol(end-nol+2:end);
u0ac = uol(end-nol+2:end);

% Initial estimate
theta0 = [A(2:end)'; B'];

% Initial covariance
P0 = diag([10, 10, 0]).*eye(na+nb-1, na+nb-1);

% Closed-loop simulation of explicit adaptive controller
[yac, uac, thetaac, Pac] = explicitAdaptiveControllerMV0RLS ...
    (A, B, C, k, w, sigmaesq, y0ac, u0ac, theta0, P0, N);

% Store solution
Y = [Y, [ycl(ncl+1:end); yac(ncl+1:end)]];
U = [U, [ucl(ncl+1:end); uac(ncl+1:end)]];

Theta = [Theta, thetaac(:, ncl+1:end)];
Theta_ref = [Theta_ref, repmat([A(2:end)'; B'], 1, N)];

%% Closed-loop simulation of static controller (shifted setpoint)
% Setpoint
w_shift = 4*ones(1, N+ncl);

% Initial values
y0cl_shift = ycl(end-ncl+2:end);
u0cl_shift = ucl(end-ncl+2:end);

% Closed-loop simulation
[ycl_shift, ucl_shift] = simulateClosedLoopARMAX ...
    (A, B, C, R, Q, S, k, w_shift, sigmaesq, y0cl_shift, u0cl_shift, N);

%% Closed-loop simulation of adaptive controller (shifted setpoint)
% Initial values
y0ac_shift = yac(end-ncl+2:end);
u0ac_shift = uac(end-ncl+2:end);

% Initial estimate
% Note: The estimate of B cannot be all zeros because the controller then
% becomes undefined (the control action is estimated not to have any impact
% on the system
theta0_shift = thetaac(:, end);

% Initial covariance
P0 = Pac(:, :, end);

% Closed-loop simulation of explicit adaptive controller
[yac_shift, uac_shift, thetaac_shift, Pac_shift] = explicitAdaptiveControllerMV0RLS ...
    (A, B, C, k, w_shift, sigmaesq, y0ac_shift, u0ac_shift, theta0_shift, P0, N);

% Store solution
Y = [Y, [ycl_shift(ncl+1:end); yac_shift(ncl+1:end)]];
U = [U, [ucl_shift(ncl+1:end); uac_shift(ncl+1:end)]];

Theta = [Theta, thetaac_shift(:, ncl+1:end)];
Theta_ref = [Theta_ref, repmat([A(2:end)'; B'], 1, N)];

%% Closed-loop simulation of static controller (second parameter set)
% Update system parameters
A(2) = 0.09;%0.1;%

% Initial values
y0cl_op2 = ycl_shift(end-ncl+2:end);
u0cl_op2 = ucl_shift(end-ncl+2:end);

% Closed-loop simulation
[ycl_op2, ucl_op2] = simulateClosedLoopARMAX ...
    (A, B, C, R, Q, S, k, w_shift, sigmaesq, y0cl_op2, u0cl_op2, N);

%% Closed-loop simulation of adaptive controller (second parameter set)
% Initial values
y0ac_op2 = yac_shift(end-ncl+2:end);
u0ac_op2 = uac_shift(end-ncl+2:end);

% Initial estimate
% Note: The estimate of B cannot be all zeros because the controller then
% becomes undefined (the control action is estimated not to have any impact
% on the system
theta0_op2 = thetaac_shift(:, end);

% Initial covariance (this is actually cheating - maybe we can use Fortescue's
% method instead - or resetting the covariance periodically)
% Resetting the covariance (multiplying by a factor of 2) works well.
P0 = Pac_shift(:, :, end);
% P0 = 100*Pac_shift(:, :, end);

% Closed-loop simulation of explicit adaptive controller
[yac_op2, uac_op2, thetaac_op2, Pac_op2] = explicitAdaptiveControllerMV0RLS ...
    (A, B, C, k, w_shift, sigmaesq, y0ac_op2, u0ac_op2, theta0_op2, P0, N);

% Store solution
Y = [Y, [ycl_op2(ncl+1:end); yac_op2(ncl+1:end)]];
U = [U, [ucl_op2(ncl+1:end); uac_op2(ncl+1:end)]];

Theta = [Theta, thetaac_op2(:, ncl+1:end)];
Theta_ref = [Theta_ref, repmat([A(2:end)'; B'], 1, N)];

%% Visualize simulation
for slide = 1:5
    % Headings used to indicate open-loop simulation, closed-loop simulation
    % etc.
    Headings{1, :} = {'Open-loop',   ''         };
    Headings{2, :} = {'Closed-loop', '(squeeze)'};
    Headings{3, :} = {'Closed-loop', '(shift)'  };
    if(slide >= 5)
        Headings{4, :} = {'Closed-loop', '(adapt)'};
    else
        Headings{4, :} = {'Closed-loop', '(system change)'};
    end

    % Time vector
    t = (-nol+1:4*N)*Ts;

    % Create figure
    figure(1);

    % Clear figure
    clf;

    % Select subplot
    subplot(211);

    % Indicate open-loop part
    fill([t(1), N*Ts, N*Ts, t(1)], [-999, -999, 999, 999], [0.3, 0.3, 0.3], ...
        'FaceAlpha', 0.03, 'EdgeAlpha', 0);

    % Keep adding plots
    hold on;

    % Indicate closed-loop part
    fill([t(1), N*Ts, N*Ts, t(1)] + 2*N*Ts, [-999, -999, 999, 999], [0.3, 0.3, 0.3], ...
        'FaceAlpha', 0.03, 'EdgeAlpha', 0);

    % Plot upper bound on Y
    plot(t, Ymax*ones(size(t)), '--k', 'LineWidth', 0.5);

    % Reset color order
    set(gca, 'ColorOrderIndex', 1);

    % Plot outputs
    plot(t,              Y(1, :          ));
    if(slide >= 5)
        plot(t(N+nol+1:end), Y(2, N+nol+1:end));
    end

    % Stop adding plots
    hold off;

    % Add open-loop text
    text(N/2*Ts, ceil(max(Y, [], 'All')) + Yoffset, Headings{1, :}, ...
        'FontSize', fs, 'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Top');

    % Add closed-loop text
    if(slide >= 2)
        text((N/2 + N)*Ts, ceil(max(Y, [], 'All')) + Yoffset, Headings{2, :}, ...
            'FontSize', fs, 'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Top');
    end

    % Add closed-loop text for changed system
    if(slide >= 3)
        text((N/2 + 2*N)*Ts, ceil(max(Y, [], 'All')) + Yoffset, Headings{3, :}, ...
            'FontSize', fs, 'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Top');
    end

    % Add closed-loop text for changed system
    if(slide >= 4)
        text((N/2 + 3*N)*Ts, ceil(max(Y, [], 'All')) + Yoffset, Headings{4, :}, ...
            'FontSize', fs, 'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Top');
    end

    % Axis limits
    ylim([floor(min(Y, [], 'All'))-1, ceil(max(Y, [], 'All')) + Yoffset+1]);
    xlim([t(1), round(min(slide*N*Ts, t(end)))]);

    % Axis ticks
    xticks(0:6:t(end));

    % Axis tick labels
    yticklabels([]);
    xticklabels([]);

    % Axis labels
    ylabel({'Output', ''});

    % Select subplot
    subplot(212);

    % Indicate open-loop part
    fill([t(1), N*Ts, N*Ts, t(1)], [-999, -999, 999, 999], [0.3, 0.3, 0.3], ...
        'FaceAlpha', 0.05, 'EdgeAlpha', 0);

    % Keep adding plots
    hold on;

    % Indicate closed-loop part
    fill([t(1), N*Ts, N*Ts, t(1)] + 2*N*Ts, [-999, -999, 999, 999], [0.3, 0.3, 0.3], ...
        'FaceAlpha', 0.03, 'EdgeAlpha', 0);

    % Reset color order
    set(gca, 'ColorOrderIndex', 1);

    % Plot outputs
    plot(t,              U(1, :          ));
    if(slide >= 5)
        plot(t(N+nol+1:end), U(2, N+nol+1:end));
    end

    % Stop adding plots
    hold off;

    % Add open-loop text
    text(N/2*Ts, floor(max(U, [], 'All')) + Uoffset, Headings{1, :}, ...
        'FontSize', fs, 'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Top');

    % Add closed-loop text
    if(slide >= 2)
        text((N/2 + N)*Ts, floor(max(U, [], 'All')) + Uoffset, Headings{2, :}, ...
            'FontSize', fs, 'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Top');
    end

    % Add closed-loop text for changed system
    if(slide >= 3)
        text((N/2 + 2*N)*Ts, floor(max(U, [], 'All')) + Uoffset, Headings{3, :}, ...
            'FontSize', fs, 'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Top');
    end

    % Add closed-loop text for changed system
    if(slide >= 4)
        text((N/2 + 3*N)*Ts, floor(max(U, [], 'All')) + Uoffset, Headings{4, :}, ...
            'FontSize', fs, 'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Top');
    end

    % Axis limits
    ylim([floor(min(U, [], 'All'))-1, ceil(max(U, [], 'All')) + Uoffset+1]);
    xlim([t(1), round(min(slide*N*Ts, t(end)))]);

    % Axis ticks
    xticks(0:6:t(end));

    % Axis tick labels
    yticklabels([]);

    % Axis labels
    ylabel({'Manipulated input', ''});
    xlabel('Time [h]');

    % Save plot
    SavePlot(['StochasticAdaptiveControl_Control_Slide', num2str(slide)], SaveFigures, Format);

    % Create figure
    figure(2);

    % Indicate open-loop part
    fill([t(1), N*Ts, N*Ts, t(1)], [-999, -999, 999, 999], [0.3, 0.3, 0.3], ...
        'FaceAlpha', 0.05, 'EdgeAlpha', 0);

    % Keep adding plots
    hold on;

    % Indicate closed-loop part
    fill([t(1), N*Ts, N*Ts, t(1)] + 2*N*Ts, [-999, -999, 999, 999], [0.3, 0.3, 0.3], ...
        'FaceAlpha', 0.03, 'EdgeAlpha', 0);

    % Reset color order
    set(gca, 'ColorOrderIndex', 1);

    % Plot parameter estimates
    if(slide >= 5)
        plot(t(N+nol+1:end), Theta(1:2, N+nol+1:end));
    end

    % Reset color order
    set(gca, 'ColorOrderIndex', 1);

    % Plot parameter estimates
    plot(t, Theta_ref(1:end-1, :), '--');

    % Stop adding plots
    hold off;

    % Add open-loop text
    text(N/2*Ts, max(Theta, [], 'All') + Thetaoffset, Headings{1, :}, ...
        'FontSize', fs, 'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Top');

    % Add closed-loop text
    if(slide >= 2)
        text((N/2 + N)*Ts, max(Theta, [], 'All') + Thetaoffset, Headings{2, :}, ...
            'FontSize', fs, 'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Top');
    end

    % Add closed-loop text for changed system
    if(slide >= 3)
        text((N/2 + 2*N)*Ts, max(Theta, [], 'All') + Thetaoffset, Headings{3, :}, ...
            'FontSize', fs, 'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Top');
    end

    % Add closed-loop text for changed system
    if(slide >= 4)
        text((N/2 + 3*N)*Ts, max(Theta, [], 'All') + Thetaoffset, Headings{4, :}, ...
            'FontSize', fs, 'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Top');
    end

    % Axis limits
    ylim([min(Theta, [], 'All')-0.1, max(Theta, [], 'All') + Thetaoffset+0.1]);
    xlim([t(1), round(min(slide*N*Ts, t(end)))]);

    % Axis ticks
    xticks(0:6:t(end));

    % Axis tick labels
    yticklabels([]);

    % Axis labels
    ylabel({'Parameters', ''});
    xlabel('Time [h]');

    % Save plot
    SavePlot(['StochasticAdaptiveControl_Identification_Slide', num2str(slide)], SaveFigures, Format);
end