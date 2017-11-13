function [xn,T] = normalisepoints(x)
%NORMALISEPOINTS  Normalises a set of points using isotropic similarity
%  [xn,T] = normalisepoints(x) takes a set of points x, and finds the
%  corresponding normalised points xn. 
%
%  Each point is assumed to be represented by a non-homogeneous column vector
%  in the input and output matrices.  The returned transformation T is the
%  one that is effectively applied to the points in homogeneous coordinates
%  to get from x to xn

% Dimension of input
m = size(x,1);  n = size(x,2);
if n<2
  error('Need more than one input point to normalise');
end

% Find isotropic normalising similarity transformation.  This is assumed to
% be effected by the pointwise transformations xn = s*(x + t1) and 
% yn = s*(y + t2), represented by the matrix T = [sI st; 0 1] in homogeneous 
% coordinates.  The conditions applied in the normalisation are 
% E{xn} = E{yn} = 0 and E{xn^2 + yn^2} = 2
xmean = mean(x');  xvar = var(x',1);  
t = -xmean';
s = sqrt(2/sum(xvar));
T = [s*eye(m,m) s*t; zeros(1,m) 1];

% Transform the points
xn = s*(x' + ones(n,1)*t')';
