function load_library
% Adds relevant library folders to the Matlab search path
%
% SYNOPSIS:
%   load_library
%
% DESCRIPTION:
% Adds relevant library folders to the Matlab search path such that they
% may be called from the current folder.
%
% REQUIRED PARAMETERS:
% 
% RETURNS:

% Let the user know that the library is being loaded
fprintf('Loading stochastic adaptive control source files .. ');

% Add library
addpath(genpath(fullfile(pwd, '/src')));

% Let the user know that the library is being loaded
fprintf('Done\n');