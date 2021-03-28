%% STEINER %% 
% STEINER team
% Date: 27/03/21
% File: dynamics.m 
% Issue: 0 
% Validated: 

%% Dynamics %% 
% This script provides a function to the dynamical vector field acting on the vehicle. 

% Inputs:  
% Outputs: - vector ds, containing in the vector field of the dynamics applied on the vehicle. 

% Everything is S.I units

function [ds] = dynamics(t, s)
    %State variables 
    h = s(2);                           %Altitude of the vehicle 
    
    %Compute the environment 
    atmos_state = atmosphere(h);        %State variables of the atmosphere
    rho = atmos_state(1);               %Density of the atmosphere
    g = gravity(h);                     %Gravity acceleration module
    
    %Forces acting on the vehicle
    
    %Traslational dynamics 
    dr = traslational_dynamics(g, T, L, D, s);
    
    %Attitude dynamics
    dtheta = attitude_dynamics(I, s(7:end));
    
    %Complete vector field 
    ds = [dr; dtheta];
end

%% Auxiliary functions 
%Traslational dynamics 
function [ds] = traslational_dynamics(g, T, L, D, s)
    %Constants of the model 
    R = 6371e3;                  %Earth mean radius
    omega = (2*pi)/(3600*24);    %Angular velocity of the Earth
    
    %State variables 
    x = s(1);               %X coordinate
    y = s(2);               %Y coordinate
    V = s(3);               %Velocity vector
    gamma = s(4);           %Fligth path angle
    m = s(5);               %Vehicle total mass
    
    %Kinematic equations 
    dr(1) = V*(R/(R+y))*cos(gamma); 
    dr(2) = V*sin(gamma); 
    
    %Mass flow rate 
    dm = 0;
    
    %Dynamic equations 
    dv = [(1/m)*(T-D-m*g*sin(gamma)); 
          -((g/V)-(V/(R+y)))*cos(gamma)-2*omega;
          dm];
           
    %Traslational vector field 
    ds = [dr; dv];
end

%Attitude dynamics 
function [ds] = attitude_dynamics(I,s)    
    %State variables 
    q = s(1:4);              %Attitude quaternion
    omega = s(5:7);          %Angular velocity 
    
    %Kinematics equations
    dqe = -(1/2)*omega.'*q(2:4);
    dqv = -(1/2)*cross(omega, q(2:4))+(1/2)*q(1)*omega; 
    dq = [dqe; dqv];
    
    %Dynamics equations
    domega = I^(-1)*(T-cross(omega, I*omega));
    
    %Campo vectorial del sistema
    ds = [dq; domega]; 
end