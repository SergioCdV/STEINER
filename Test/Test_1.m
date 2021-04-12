%% STEINER %% 
% STEINER team
% Date: 27/03/21
% File: test_1.m 
% Issue: 0 
% Validated: 

%% Test 1 %%
% This scripts provides the function to test the translational model of equations 

%% General setup 
set_graphics();

%% Earth characteristics 
R = 6371.37e3;              %Earth mean radius
omega = (2*pi)/(3600*24);   %Earth mean angular velocity

%% Vehicle's characteristics 
%Aerodynamic coefficients 
Cl = 1;                     %Lift coefficient              
Cd = 0.1;                   %Drag coefficient

m = 10;                   %Total vehicle's mass

%% Integration setup 
%Integration tolerances 
RelTol = 2.25e-14; 
AbsTol = 1e-22; 
options = odeset('RelTol', RelTol, 'AbsTol', AbsTol);

%Integration time span 
dt = 1e-3;                     %Time step 
tf = 600;                  %Final integration time 
tspan = 0:dt:tf;            %Integration span

%% Initial conditions 
%Departure conditions
r = [0; 0; R];              %Initial position with respect to the origin
v = [0; 0; 10];             %Zero initial velocity
lambda = deg2rad(40.4165);  %Geodetic latitude of Madrid
tau = deg2rad(-3.70256);    %Geodetic longitude of Madrid
a = [1; zeros(6,1)];        %Attitude state variables -nonsense now-

%Initial state variables
s0 = [r; v; tau; lambda; a; m];  

%Control conditions 
u = zeros(3,1);             %Control vector

%% Integration of the trajectory 
[t, S] = ode45(@(t,s)dynamics(t, Cl, Cd, s, u), tspan, s0, options);

for i = 1:size(S,1)
   S(i,1) = S(i,3)*cos(S(i,8))*cos(S(i,7));
   S(i,2) = S(i,3)*cos(S(i,8))*sin(S(i,7));
   S(i,3) = S(i,3)*sin(S(i,8));
end

%% ECEF trajectory
%Preallocation 
Se = zeros(size(S,1),3);        %Inertial trajectory 

for i = 1:size(S,1)
    elaps = dt*i;
    Q1 = [cos(S(i,7)) 0 -sin(S(i,7)); 0 1 0; sin(S(i,7)) 0 cos(S(i,7))];
    Q2 = [1 0 0; 0 cos(S(i,8)) -sin(S(i,8)); 0 sin(S(i,8)) cos(S(i,8))];
    QT = (Q2*Q1).';
    Se(i,1:3) = (QT*S(i,1:3).').';
end

%% Inertial trajectory 
%Preallocation 
Si = zeros(size(S,1),3);        %Inertial trajectory 

for i = 1:size(S,1)
    elaps = dt*i;
    Q0 = [1 0 0; 0 0 1; 0 -1 0]*[cos(omega*elaps) sin(omega*elaps) 0; sin(omega*elaps) cos(omega*elaps) 0; 0 0 1];
    Q1 = [cos(S(i,7)) 0 -sin(S(i,7)); 0 1 0; sin(S(i,7)) 0 cos(S(i,7))];
    Q2 = [1 0 0; 0 cos(S(i,8)) -sin(S(i,8)); 0 sin(S(i,8)) cos(S(i,8))];
    QT = (Q2*Q1*Q0).';
    Si(i,1:3) = (QT*S(i,1:3).').';
end

%% Results 
%Plot trajectory 
figure(1) 
view(3) 
plot3(S(:,1), S(:,2), S(:,3));
xlabel('LVLH x coordinate');
ylabel('LVLH y coordinate');
zlabel('LVLH z coordinate');
grid on; 
title('LVLH trajectory');

% figure(2) 
% view(3) 
% plot3(Se(:,1), Se(:,2), Se(:,3));
% xlabel('ECEF x coordinate');
% ylabel('ECEF y coordinate');
% zlabel('ECEF z coordinate');
% grid on; 
% title('ECEF trajectory');
% 
% figure(3) 
% view(3) 
% plot3(Si(:,1), Si(:,2), Si(:,3));
% xlabel('ECI x coordinate');
% ylabel('ECI y coordinate');
% zlabel('ECI z coordinate');
% grid on; 
% title('ECI trajectory');

%% Auxiliary functions 
function set_graphics()
    %Set graphical properties
    set(groot, 'defaultAxesTickLabelInterpreter', 'latex'); 
    set(groot, 'defaultAxesFontSize', 11); 
    set(groot, 'defaultAxesGridAlpha', 0.3); 
    set(groot, 'defaultAxesLineWidth', 0.75);
    set(groot, 'defaultAxesXMinorTick', 'on');
    set(groot, 'defaultAxesYMinorTick', 'on');
    set(groot, 'defaultFigureRenderer', 'painters');
    set(groot, 'defaultLegendBox', 'off');
    set(groot, 'defaultLegendInterpreter', 'latex');
    set(groot, 'defaultLegendLocation', 'best');
    set(groot, 'defaultLineLineWidth', 1); 
    set(groot, 'defaultLineMarkerSize', 3);
    set(groot, 'defaultTextInterpreter','latex');
end