function vizfield(width,res,varargin)
% VIZFIELD plot a 2D Gaussian random field
%   VIZFIELD(...) generates a random field using
%   QRS.FIELDS.GAUSSIAN.GENERATE(...) and plots it.

data = qrs.fields.gaussian.generate(width,res,varargin{:});

[x,y] = meshgrid(0:res:width-1,0:res:width-1);
surf(x,y,data,'EdgeColor','None');

view(2);
colormap(gray);
axis image;
axis off;