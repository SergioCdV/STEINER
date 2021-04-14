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
mu = 3.986e14;              %Earth gravitational parameter 

%% Vehicle's characteristics 
%Aerodynamic coefficients 
Cl = 1;                     %Lift coefficient              
Cd = 0.1;                   %Drag coefficient

m = 1e4;                        %Total vehicle's mass
I = 1e4*[1 0 0; 0 2 0; 0 0 3];  %Inertia dyadic

%% Integration setup 
%Integration tolerances 
RelTol = 2.25e-14; 
AbsTol = 1e-22; 
options = odeset('RelTol', RelTol, 'AbsTol', AbsTol, 'Events', @(t,s)crash_event(s));

%Integration time span 
dt = 1e-3;                  %Time step 
tf = 600;                   %Final integration time 
tspan = 0:dt:tf;            %Integration span

%% Initial conditions 
%Departure conditions
r = [0; 0; 0];              %Initial position with respect to the origin
v = [0; 0; 10];             %Zero initial velocity
lambda = deg2rad(40.4165);  %Geodetic latitude of Madrid
tau = deg2rad(-3.70256);    %Geodetic longitude of Madrid
q0 = [1; 0; 0; 0];          %Initial LVLH-body frame quaternion
omega0 = 1e-3*ones(3,1);    %Initial LVLH-body frame angular velocity
a = [q0; omega0];           %Attitude state variables 

%Initial state variables
s0 = [r; v; tau; lambda; a; m];  

%Control conditions 
u = 0;                      %Control vector
alpha = deg2rad(7);         %Angle of attack
M = zeros(3,1);             %Control torques

%% Integration of the trajectory 
[t, S] = ode113(@(t,s)final_dynamics(mu, t, s, I, M, u, alpha), tspan, s0, options);

%% ECEF trajectory
%Preallocation of the trajectory
Se = zeros(size(S,1),3);

%ECEF trajectory
for i = 1:size(S,1)
   Se(i,1) = (R+S(i,3))*cos(S(i,8))*cos(S(i,7));
   Se(i,2) = (R+S(i,3))*cos(S(i,8))*sin(S(i,7));
   Se(i,3) = (R+S(i,3))*sin(S(i,8));
end

%% Results 
%Plot trajectory 
figure(1) 
view(3) 
plot3(Se(:,1), Se(:,2), Se(:,3));
xlabel('ECEF x coordinate');
ylabel('ECEF y coordinate');
zlabel('ECEF z coordinate');
grid on; 
title('ECEF trajectory');

figure(2) 
hold on
plot(t, S(:,4:6));
hold off
xlabel('Time [s]');
ylabel('LVLH velocity [m/s]')
grid on; 
title('Velocity evolution');
legend('LVLH $v_x$', 'LVLH $v_y$', 'LVLH $v_z$');

%Plot quaternion
figure(3) 
plot(t, S(:,9:12));
xlabel('Time [s]');
ylabel('Attitude quaternion');
grid on; 
title('Attitude evolution');
legend('$\eta$', '$\epsilon_1$', '$\epsilon_2$', '$\epsilon_3$');

figure(4) 
plot(t, S(:,13:15));
xlabel('Time [s]');
ylabel('Angular velocity [rad/s]');
grid on; 
title('Angular velocity evolution');
legend('$\omega_x$', '$\omega_y$', '$\omega_z$');

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