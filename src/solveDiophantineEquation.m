function [G, S] = solveDiophantineEquation(A, B, m)
% B is the numerator polynomial and A is the denominator polynomial.

% Initialize
G = [];
S = [B, zeros(size(B, 1), numel(A) - numel(B))];

for i = 1:m
    G = [G, S(1)]; %#ok
    S = [S(2:end) - S(1)*A(2:end), 0];
end

% Solution
S = S(1:end-1);