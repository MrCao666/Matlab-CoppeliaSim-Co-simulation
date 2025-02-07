function [pose_rotm, pose_axang] = path_reference_kuka_angvec(t)

% 三叶草
cof = 2*pi/15;
t = t*cof;


% %末端位姿-------------------------------------------------------------%
% kuka--------------------
vec = [-0.6405, 0.7634, -0.0842];
vari_ang = 0.18*sin(t);
ang = 2.9415 + vari_ang;
axang = [vec, ang]';

% %立体三叶草
px = 0.2127;
py = -0.0001;
pz = 0.6822;
x = cos(t)+2*cos(2*t);
y = sin(t)-2*sin(2*t);
z = 1.5*sin(3*t);

x = -0.06*x+px+0.18;
y = -0.06*y+py;
z = 0.06*z+pz;
p = [x;y;z];

%返回值---------------------------------------------------------------------------%
rotm = axang2rotm([vec, ang]);
w(1:2,1) = rotm(2:3,2);
w(3:5,1) = rotm(1:3,3);

pose_rotm = [p; w];
pose_axang = [p; axang];

end
