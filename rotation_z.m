function [ r ] = rotation_z( ka )                       %angle Yaw around axis Z
r=[cos(ka) sin(ka) 0;-sin(ka) cos(ka) 0;0 0 1];
end

