function [Pn,Ppn,Xsn,nf] = ppfrome(E,exs,exps)
%PPFROME  Cameras from essential matrix and normalised correspondences
%   [Pn,Ppn,Xc,nf] = ppfrome(E,exs,exps) finds the two cameras
%   corresponding to the essential matrix E with point correspondences exs
%   and exps in normalised coordinates.  The method used is outlined in H&Z
%   p.257-258.  The cameras are returned in Pn and Ppn, the reconstructed
%   points in Xc, and the number of points in front of the cameras in nf.
%
%   This code needs to be properly checked

% First camera is canonical in normalised coordinates
Pn = [eye(3) zeros(3,1)];

% According to Hartley and Zisserman (p.258) there are four possible
% cameras Pp = [R t] corresponding to a given essential matrix.  Find
% these cameras
[U,D,V] = svd(E);
W = [0 -1 0; 1 0 0; 0 0 1];
u3 = U(:,3);
Ppc = cell(4,1);
Ppc{1} = [U*W*V' u3];
Ppc{2} = [U*W*V' -u3];
Ppc{3} = [U*W'*V' u3];
Ppc{4} = [U*W'*V' -u3];

% World view points reconstructed from the correspondences must lie in
% front of both cameras for the Pp to be correct.  Reconstruct the points
% and select the camera with the majority
pos_counts = zeros(4,1);
Xsnc = cell(4,1);
for i=1:4
  Xc = triangulate_hom(exs,exps,[eye(3) zeros(3,1)],Ppc{i});
  Xsnc{i} = Xc;
  
  % First camera: P = [I 0]
  C = [0;0;0];  v = [0;0;1];
  s1 = (C + v)'*(Xc - C*ones(1,size(Xc,2)));
  
  % Second camera
  M = Ppc{i}(:,1:3);  m3 = M(3,:)';  p4 = Ppc{i}(:,4);
  C = -inv(M)*p4;  v = det(M)*m3;
  s2 = (C + v)'*(Xc - C*ones(1,size(Xc,2)));
  
  % Count of votes for camera
  pos_counts(i) = sum((s1>0) & (s2>0));
end
majind = find(pos_counts==max(pos_counts));
Ppn = Ppc{majind};
Xsn = Xsnc{majind};
nf = pos_counts(majind);