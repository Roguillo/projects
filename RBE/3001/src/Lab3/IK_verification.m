clc; clear;

clanker = Robot();
travel_time  = 2;

% v = [35   50    20   90];
v = clanker.measure_js(true, false);
v = v(1,:);

fk_mat = clanker.fk_3001(deg2rad(v(1,:)));
disp(fk_mat);

coords = [fk_mat(1, 4), fk_mat(2, 4), fk_mat(3, 4)];
angles = rad2deg(clanker.ik3001([coords, deg2rad(50+20+90)]));

% clanker.interpolate_jp(angles, travel_time);

% tic;
% clanker.measure_js(true, false);
% toc