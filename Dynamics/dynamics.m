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
    %Constants of the model 
    T0 = 9.81*;                                     %Characteristic thrust of the vehicle
    
    %State variables 
    r = s(1:3);                                     %Position vector
    v = s(4:6);                                     %Velocity vector
    lambda = s(8);                                  %Geodetic latitude
    q = s(9:12);                                    %Attitude quaternion
    cg = s(end-2:end);                              %Current location of the center of mass
            
    %Compute the environment conditions
    atmos_state = atmosphere('ISA', r);             %State variables of the atmosphere
    g = gravity(r, lambda);                         %Gravity acceleration module
    
    %Forces and torques acting on the vehicle
    [Fa, Ta] = aerodynamic_force(cg, atmos_state, v, q, Cl, Cd);      %Aerodynamic force and torque
    T = thrust_force(q);                                              %Thrust force
    
    %Mass dynamics 
    dm = -T/T0;
    
    %Center of gravity dynamics
    dcg(1) = dm;
    
    %Traslational dynamics 
    dr = traslational_dynamics(g, T, Fa, u, s);
            
    %Attitude dynamics
    dtheta = attitude_dynamics(dm, I, Ta, M, s);
    
    %Complete vector field 
    ds = [dr; dtheta; dm; dcg];
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
    m = s(end-1);                %Vehicle's mass
        
    %Kinematic equations 
    dr = v;    
    
    %Longitude equations 
    dtau = v(1)/((R+r(3))*cos(lambda));
    
    %Latitude equation 
    dlambda = v(2)/(R+r(3)); 
    
    %Dynamic equations 
    dv = g+(1/m)*(T+Fa+u); 
    dv(1) = dv(1)-2*omega*(v(3)*cos(lambda)-v(2)*sin(lambda))-(v(1)*(v(3)-v(2)*tan(lambda)))/(R+r(3));
    dv(2) = dv(2)-omega*sin(lambda)*(omega*(R+r(3))*cos(lambda)+2*v(1))-(v(2)*v(3)+v(1)^2*tan(lambda))/(R+r(3));
    dv(3) = dv(3)+omega*cos(lambda)*(omega*(R+r(3))*cos(lambda)+2*v(1))+(v(1)^2+v(2)^2)/(R+r(3));
           
    %Traslational vector field 
    ds = [dr; dv; dtau; dlambda];
end

%Attitude dynamics 
function [ds] = attitude_dynamics(dm, I, M, T, s)    
    %Constants of the model 
    R = 6371.37e3;                  %Mean Earth radius
    omega_e = (2*pi)/(3600*24);     %Mean Earth angular velocity 
    dI = zeros(3,3);                %Approximation of the derivative of the inertia tensor with respect to the mass
    
    %State variables 
    lambda = s(8);                                   %Latitude of the aircraft
    dlambda = s(5)/(R+s(3));                         %Time derivative of the latitude
    dtau = s(4)/((R+s(3))*cos(lambda));              %Time derivative of the latitude
    q = s(9:12);                                     %Attitude quaternion
    omega = s(13:17);                                %Angular velocity 
    
    %Kinematics equations
    dqe = -(1/2)*omega.'*q(2:4);
    dqv = -(1/2)*cross(omega, q(2:4))+(1/2)*q(1)*omega; 
    dq = [dqe; dqv];
    
    %Total angular velocity 
    omega01 = [-dlambda; (omega_e+dtau)*cos(lambda); (omega_e+dtau)*sin(lambda)];
    omega21 = omega01+omega;
    
    %Dynamics equations
    domega = I^(-1)*(T+M-cross(omega21,I*omega21)-dm*dI*omega21)+cross(omega,omega01);
    
    %Campo vectorial del sistema
    ds = [dq; domega]; 
end