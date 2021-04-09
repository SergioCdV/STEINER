%% STEINER %% 
% STEINER team
% Date: 27/03/21
% File: test_1.m 
% Issue: 0 
% Validated: 

%% Test 1 %%
% This scripts provides the function to test the translational model of equations 

%% Earth characteristics 
R = 6371e3;                 %Earth mean radius

%% Vehicle's characteristics 
%Aerodynamic coefficients 
Cl = 1;                     %Lift coefficient              
Cd = 0.1;                   %Drag coefficient

m = 1e4;                    %Total vehicle's mass

%% Integration setup 
%Integration tolerances 
RelTol = 2.25e-1; 
AbsTol = 1e-22; 
options = odeset('RelTol', RelTol, 'AbsTol', AbsTol);

%Integration time span 
dt = 1;                     %Time step 
tf = 3600;                  %Final integration time 
tspan = 0:dt:tf;            %Integration span

%% Initial conditions 
%Departure conditions
r = [0; 0; R];              %Initial position with respect to the origin
v = zeros(3,1);             %Zero initial velocity
lambda = deg2rad(40);       %Geodetic latitude of Madrid
a = [1; zeros(6,1)];        %Attitude state variables -nonsense now-

s0 = [r; v; lambda; a; m];  %Initial state variables

%Control conditions 
u = zeros(3,1);             %Control vector

%% Integration of the trajectory 
[t, S] = ode113(@(t,s)dynamics(t, Cl, Cd, s, u), tspan, s0);

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