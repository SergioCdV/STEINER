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

function [F, Ta] = aerodynamic_force(atmos_state, v, q, alpha)
    %Constants of the model 
    c = 5;                                          %Mean aerodynamic constants
    S = 41;                                         %Vehicle surface area
    rho = atmos_state(1);                           %Atmosphere density
    T = atmos_state(3);                             %Atmosphere temperature
    R = 287;                                        %Air ideal gas constant 
    gamma = 1.4;                                    %Air heat coefficient ratio
    
    %Mach number 
    M = norm(v)/sqrt(gamma*T*R);
    if (isnan(M))
        disp('');
    end
    
    %Compute the lift and drag coefficients
    Cl = 2*pi*alpha/sqrt(abs(1-M^2));               %Lift coefficient
    Cd = Cl^2;                                      %Drag coefficient

    %Compute the aerodynamic drag unit vectors
    Q = quaternion2matrix(q);                       %Rotation matrix from the LVLH frame to the body frame
    ul = Q.'*[-sin(alpha); 0; cos(alpha)];          %Lift force unit vector
    ud = -Q.'*[cos(alpha); 0; sin(alpha)];          %Drag force unit vector
    ub = cross(ul,ud);                              %Lateral force unit vector
        
    %Compute the associated forces 
    L = (1/2)*Cl*rho*S*norm(v)^2*ul;                %Lift force
    D = (1/2)*Cd*rho*S*norm(v)^2*ud;                %Drag force
    Q = 0*ub;                                       %Laterial force
    
    %Total aerodynamic force in the LVLH frame 
    F = D+L+Q;
    
    %Compute the position of the center of pressure
    if (M > 1)
        ca = [0.5*c; 0; 0]; 
    else
        ca = [0.25*c; 0; 0]; 
    end
    
    %Compute the aerodynamic torque 
    Ta = cross(F, -ca);
    if (imag(Ta) ~= 0)
        disp('');
    end
end