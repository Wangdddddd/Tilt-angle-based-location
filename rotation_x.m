function [ r ] = rotation_x( om )                 % angle Roll around X axis!
r=[1 0 0;0 cos(om) sin(om);0 -sin(om) cos(om)];
end

