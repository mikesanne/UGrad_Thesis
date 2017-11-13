function [Xhats,dist] = triangulate_hom(xs,xps,P,Pp,homflag)
%TRIANGULATE_HOM  Homogeneous method for triangulating projections
%   [Xhat,dist] = triangulate_hom(xs,xps,P,Pp,homflag) finds the world
%   vector Xhats that triangulate the observations xs and xps under
%   projection matrices P and Pp.  The inputs are assumed to be matrices of
%   column vectors, each column of which represents a point.  The output
%   Xhat is returned as a set of column vectors.  If output dist is
%   required, it contains the average reprojection error of each
%   triangulated point for the two views.
%
%   If homflag is provided and is nonzero, then the 3D points are returned
%   in homogeneous coordinates (so Xhats will have 4 rows).

% Inputs
if nargin<5
  homflag = 0;
end

% Convert from homogeneous coordinates if necessary
if size(xs,1)==3, xs = xs(1:2,:)./(ones(2,1)*xs(3,:)); end
if size(xps,1)==3, xps = xps(1:2,:)/(ones(2,1)*xps(3,:)); end

% Normalise points and cameras
if 1 & size(xs,2)>2
  [xs,T] = normalisepoints(xs);
  P = T*P;
  [xps,T] = normalisepoints(xps);
  Pp = T*Pp;
end

% Calculate triangulated points in homogeneous coordinates
nump = size(xs,2);
Xhats = zeros(4,nump);
for i=1:nump

  % Calculate matrix A
  A = [xs(1,i)*P(3,:) - P(1,:); 
       xs(2,i)*P(3,:) - P(2,:);
       xps(1,i)*Pp(3,:) - Pp(1,:);
       xps(2,i)*Pp(3,:) - Pp(2,:)];
 
  % Solution is the eigenvector cooresponding to smallest eigenvalue of A
  [U,S,V] = svd(A);
  Xhats(:,i) = V(:,4);
  
end

% Calculate average image distance between points and reprojected point
if nargout>1
  xsrh = P*Xhats;  xsr = xsrh(1:end-1,:)./repmat(xsrh(end,:),2,1);
  xpsrh = Pp*Xhats;  xpsr = xpsrh(1:end-1,:)./repmat(xpsrh(end,:),2,1);
  dist = sqrt(sum((xs-xsr).^2));
  distp = sqrt(sum((xps-xpsr).^2));
  dist = mean([dist; distp]);  
end

% Convert from homogeneous coordinates if necessary
if homflag==0
  Xhats = Xhats(1:3,:)./repmat(Xhats(4,:),3,1);
end