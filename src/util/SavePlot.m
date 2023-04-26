function SavePlot(name,varargin)
% SavePlot  save a figure in tikz, eps or png format
%
% SYNOPSIS:
%   SavePlot(name)
%   SavePlot(name, save)
%   SavePlot(name, save, format)
%
% DESCRIPTION:
%   Saves the current figure in tikz, eps or png format using either
%   matlab2tikz (which must be available if tikz is used), saveas or print
%
% REQUIRED PARAMETERS:
%   name        - Name of the saved figure
% 
% OPTIONAL PARAMETERS:
%   save        - Boolean determining whether to save the figure or not
%                 [default: false]
%   format      - Either 'eps', 'png' or 'tikz' (case-insensitive)
%                 [default: 'eps']
%
% RETURNS:
%
% See also matlab2tikz, saveas, print

% Default settings
save = false;
format = 'eps';

% Check if user wants to save the plot
if(nargin > 1)
    save = varargin{1};
end

% Check if the user has specified the format
if(nargin > 2)
    format = varargin{2};
end

% Save if necessary
if(save)
    switch lower(format)
        case 'tikz'
            matlab2tikz(['tikz/' name '.tikz'], ...
                'height','\figureheight','width','width','\figurewidth', ...
                'ShowInfo',false, 'ShowWarnings',false);
        case 'eps'
            print(['eps/' name],'-depsc');
        case 'png'
            print(['png/' name],'-dpng','-r300');
    end
end