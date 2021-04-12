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

function [ds] = dynamics(t, Cl, Cd, s, u)
    %State variables 
    r = s(1:3);                                     %Position vector
    v = s(4:6);                                     %Velocity vector
    lambda = s(8);                                  %Geodetic latitude
    q = s(9:12);                                    %Attitude quaternion
    m = s(end);                                     %Vehicle's mass
        
    %Compute the environment conditions
    atmos_state = atmosphere('ISA', r);             %State variables of the atmosphere
    rho = atmos_state(1);                           %Density of the atmosphere
    g = gravity(r, lambda);                         %Gravity acceleration module
    
    %Forces acting on the vehicle
    Fa = aerodynamic_force(rho, v, r, q, Cl, Cd);   %Aerodynamic force
    T = thrust_force(q);                            %Thrust force

    Fa = zeros(3,1);
    
    %Traslational dynamics 
    dr = traslational_dynamics(g, T, Fa, u, s);
    
    %Attitude dynamics
    %dtheta = attitude_dynamics(I, s);
    dtheta = zeros(7,1); 
    
    %Mass dynamics 
    dm = 0;
    
    %Complete vector field 
    ds = [dr; dtheta; dm];
end

%% Auxiliary functions 
%Traslational dynamics 
function [ds] = traslational_dynamics(g, T, Fa, u, s)
    %Constants of the model 
    R = 6371.37e3;               %Earth mean radius
    omega = (2*pi)/(3600*24);    %Angular velocity of the Earth
    
    %State variables 
    r = s(1:3);                  %Position vector 
    v = s(4:6);                  %Velocity vector
    lambda = s(8);               %Geodetic latitude
    m = s(end);                  %Vehicle's mass
        
    %Kinematic equations 
    dr = v;    
    
    %Longitude equations 
    dtau = v(1)/(r(3)*cos(lambda));
    
    %Latitude equation 
    dlambda = v(2)/r(3); 
    
    %Dynamic equations 
    dv = g+(1/m)*(T+Fa+u); 
    dv(1) = dv(1)-2*omega*(v(3)*cos(lambda)-v(2)*sin(lambda))-(v(1)*(v(3)-v(2)*tan(lambda)))/r(3);
    dv(2) = dv(2)-omega*sin(lambda)*(omega*r(3)*cos(lambda)+2*v(1))-(v(2)*v(3)+v(1)^2*tan(lambda))/r(3);
    dv(3) = dv(3)+omega*cos(lambda)*(omega*r(3)*cos(lambda)+2*v(1))+(v(1)^2+v(2)^2)/r(3);
           
    %Traslational vector field 
    ds = [dr; dv; dtau; dlambda];
end

%Attitude dynamics 
function [ds] = attitude_dynamics(I,T,s)    
    %State variables 
    q = s(8:12);              %Attitude quaternion
    omega = s(13:16);         %Angular velocity 
    
    %Kinematics equations
    dqe = -(1/2)*omega.'*q(2:4);
    dqv = -(1/2)*cross(omega, q(2:4))+(1/2)*q(1)*omega; 
    dq = [dqe; dqv];
    
    %Dynamics equations
    domega = I^(-1)*(T-cross(omega, I*omega));
    
    %Campo vectorial del sistema
    ds = [dq; domega]; 
end