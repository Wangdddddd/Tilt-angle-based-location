function [ r ] = rotation_y( ph )                     %angle pitch around axis Y!
r=[cos(ph) 0 -sin(ph);0 1 0;sin(ph) 0 cos(ph)];
end

