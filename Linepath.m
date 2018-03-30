%Suppose UAV flies along a path which is a circle. 
%The center of the circle is at (ox,oy,oz), the radius of the circle is
%represented by r; and target is centering at (xt,yt,zt). the simulation is
%firstly to show the performance of conventional target location method
%without considering the influence of plane constraint and large tilt angle
%and the performance of the proposed method; The constrast of the two will
%be given.
clear all
clc
syms xt yt zt t r h x y z;
syms ox oy oz;  
%===================================Initialization==================================================
N = 30;
K = 50;
ox=0;
oy=0;
h=500;
xt=1000;
yt=0;
M = cell(1,N);
LocatedPosition = ones(3,N);
shift2 = 0*ones(1,K);
shift1 = 0*ones(1,K);

%==================Noises==variance================================================================lp
valpha = 0.001;
vbeta = 0.002;
vkapa = 0.001;
vphi = 0.001;
vomiga = 0.001;
%===================================================================================================
for j = 1:K
    vbeta = 0.001*j;
    for n = 1:N
        k=n;
        UAVderivative = [0, 1];                            %gradient of view point
        UAVposition = [0 60*k h*k+100];                             %position of view point
        target_position = [xt yt 0];                                     %target position
        unit_x = [1 0];
        pointing_vector_projection = [xt yt-60*k];          %projection of pointing vector
        unit_pointing_vector = pointing_vector_projection / norm(pointing_vector_projection);
        
        %distance_Ut = norm(UAVposition - target_position);                 %pointing distance
        %%---------------------------Solve---Alpha---of-------camera------------------------------------------------------------%$
        temp = acos(sum(pointing_vector_projection.*UAVderivative)/(norm(pointing_vector_projection)*norm(UAVderivative))); %$
        Alpha(n) = temp;                                                                                                    %$
                                                                       %$
        %---------------------------------------------------------------------------------------------------------------------- $
        distance_Ut(n) = norm(UAVposition - target_position);
        Beta(n) = acos((h*k+100)/distance_Ut(n));                         % Pitch angle of camera
        Kapa(n) = pi/2.;                                       % Yaw angle of UAV  Kapa belonging to [pi/2,5pi/2]
        Phi(n) = 0;                                            % pitch angle of UAV
        Omiga(n) = 0;                                          % Roll angle of UAV
        %===================Uniform==Distribution================================================================================
        %     delta_beta(n) = (rand - 0.5)*0.02;
        %     delta_alpha(n) = (rand - 0.5)*0.02;
        %     delta_kapa(n) = (rand - 0.5)*0.02;
        %     delta_phi(n) = (rand - 0.5)*0.02;
        %     delta_omi(n) = (rand - 0.5)*0.02;
        %
        %     beta(n) = delta_beta(n) + beta(n);
        %     alpha(n) = delta_alpha(n) + alpha(n);
        %     kapa(n) = delta_kapa(n) + kapa(n);
        %     phi(n) = delta_phi(n) + phi(n);
        %     omi(n) = delta_omi(n) + omi(n);
        %===================Norm==Distribution============================================================================================
        Delta_beta(n) = normrnd(0, vbeta);
        Delta_alpha(n) = normrnd(0, valpha);
        Delta_kapa(n) = normrnd(0, vkapa);
        Delta_phi(n) = normrnd(0, vphi);
        Delta_omiga(n) = normrnd(0, vomiga);
        
        Beta(n) = Delta_beta(n) + Beta(n);
        Alpha(n) = Delta_alpha(n) + Alpha(n);
        Kapa(n) = Delta_kapa(n) + Kapa(n);
        Phi(n) = Delta_phi(n) + Phi(n);
        Omiga(n) = Delta_omiga(n) + Omiga(n);
        Theta(n) = Beta(n);
        distance_Ut(n) = (h*k+100)/cos(Theta(n));
        %===========================================================Transformation========================================================
        M{1,n} = rotation_x(Omiga(n)) * rotation_y(Phi(n)) * rotation_z(-Kapa(n))*rotation_z(Alpha(n))*rotation_y(Beta(n));
        %   m(n) = cos(phi(n))*(cos(beta(n))*cos(omi(n)) + sin(alpha(n))*sin(beta(n))*sin(omi(n))) - cos(alpha(n))*sin(beta(n))*sin(phi(n));
        
        LocatedPosition(:,n) = M{1,n}*[0,0,-distance_Ut(n)]' + [0, 60*k, h*k]';
        %     P = [0 0 500];
        %     Q = [1500 0 0];
        %     Slope = norm(P-Q)
        %     oz = 500*ones(1, 360);
        %     theata = acos(m(n))*ones(1, 360);
        %     Slope = oz./cos(theata);
        %     Slope(5)
        %     al = 0;
        %     alpha = al*ones(1, 360);
        
     
    end
    
    sum_costheta = sum(cos(Theta));
    weight1 = cos(Theta) / sum_costheta;
    weightsum1(j,:) = weight1*LocatedPosition';
    meansum(j,:) =  sum(LocatedPosition')/N;
        
    distancereprocical = 1./distance_Ut;
    numerator = distancereprocical.*cos(Theta);
    sum_denominator = sum(numerator);
    weight2 = numerator / sum_denominator;
    weightsum2(j,:) = weight2 * LocatedPosition';
    
%     weightsum = weightsum + Shift;
    
%     figure(1)
%     stem(weightsum2(1,j),weightsum2(2,j),'MarkerFaceColor','red','MarkerEdgeColor','green');
%     hold on
   
end
for k = 1:K
    shift1(k) = sqrt( (weightsum1(k,1) - target_position(1))^2 + (weightsum1(k,2) - target_position(2))^2);
    shift2(k) = sqrt( (weightsum2(k,1) - target_position(1))^2 + (weightsum2(k,2) - target_position(2))^2);
end
 figure(1)
%  stem(LocatedPosition(1,:),LocatedPosition(2,:),'LineStyle','none','MarkerFaceColor','blue','MarkerEdgeColor','red')
%  hold on
%  stem(xt,0,'LineStyle','none','MarkerFaceColor','green','MarkerEdgeColor','green');
%  hold on
stem(weightsum1(:,1),weightsum1(:,2),'MarkerFaceColor','red','MarkerEdgeColor','green');
 hold on
%  stem(meansum(:,1),meansum(:,2),'MarkerFaceColor','yellow','MarkerEdgeColor','green');
%  hold on
 stem(weightsum2(:,1),weightsum2(:,2),'LineStyle','none','MarkerFaceColor','black','MarkerEdgeColor','green');
 hold on

% 
%     s=1500;
%     xr = s*cos(al);
%     yr = s*sin(al);
%     stem(xr,yr,'MarkerFaceColor','red','MarkerEdgeColor','blue');
%     hold off
% 
% 
figure(2)

histogram(shift1);
hold on
histogram(shift2);
% 
% 
figure(3)
stem(shift1,'LineStyle','none','MarkerFaceColor','red','MarkerEdgeColor','green');
hold on

stem(shift2,'LineStyle','none','MarkerFaceColor','black','MarkerEdgeColor','green')
hold off
%     stem(x2,y2,'LineStyle','none','MarkerFaceColor','red','MarkerEdgeColor','blue')
%     hold on
%     stem(xr,yr,'MarkerFaceColor','blue','MarkerEdgeColor','red');
