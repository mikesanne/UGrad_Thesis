function R=euler_to_R(rpy)

r=rpy(1);
p=rpy(2);
y=rpy(3);
R = [cos(p)*cos(y) cos(y)*sin(r)*sin(p)-cos(r)*sin(y) cos(r)*cos(y)*sin(p)+sin(r)*sin(y);
     cos(p)*sin(y) cos(r)*cos(y) + sin(r)*sin(p)*sin(y) cos(r)*sin(p)*sin(y) - cos(y)*sin(r);
     -sin(p) cos(p)*sin(r) cos(r)*cos(p)];
 
end
