%% STEINER %% 
% STEINER team
% Date: 27/03/21
% File: aerodynamic_force.m 
% Issue: 0 
% Validated: 

%% Aerodynamic force %% 
% This script provides a function to compute the aerodynamic force acting on the vehicle. 

% Inputs:  - scalar alpha, the angle of attack of the vehicle
%          - vector atmos_state, containing the atmosphere state variables
%          - vector v, the aerodynamic velocity vector of the vehicle 
%          - vector q, the body frame quaternion
%          - scalar alpha, the angle of attack of the vehicle

% Outputs: - vector F, the total aerodynamic force. 
%          - vector Ta, the total aerodynamic torque over the center of
%            mass

% Everything is S.I units

function [F, Ta] = aerodynamic_force(cg, atmos_state, v, q, alpha)
    %Constants of the model 
    S = 41;                                         %Vehicle surface area
    rho = atmos_state(1);                           %Atmosphere density

    %Compute the aerodynamic drag unit vectors
    Q = quaternion2matrix(q);                       %Rotation matrix from the LVLH frame to the body frame
    ul = Q.'*[-sin(alpha); 0; cos(alpha)];          %Lift force unit vector
    ud = -Q.'*[cos(alpha); 0; sin(alpha)];          %Drag force unit vector
    ub = Q.'*zeros(3,1);                            %Lateral force unit vector
        
    %Compute the associated forces 
    L = (1/2)*Cl*rho*S*norm(v)^2*ul;                %Lift force
    D = (1/2)*Cd*rho*S*norm(v)^2*ud;               %Drag force
    Q = zeros(3,1)*ub;                              %Laterial force
    
    %Total aerodynamic force in the LVLH frame 
    F = D+L+Q;
    
    %Compute the position of the center of pressure 
    ca = zeros(3,1); 
    
    %Compute the aerodynamic torque 
    Ta = cross(F, cg-ca);
end