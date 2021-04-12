%% STEINER %% 
% STEINER team
% Date: 27/03/21
% File: aerodynamic_force.m 
% Issue: 0 
% Validated: 

%% Aerodynamic force %% 
% This script provides a function to compute the aerodynamic force acting on the vehicle. 

% Inputs:  - scalar rho, the density at which the aerodynamic force is to
%            be evaluated
%          - vector r, the position vector of the vehicle 
%          - vector v, the aerodynamic velocity vector of the vehicle 
%          - vector q, the body frame quaternion
%          - scalar Cl, the lift coefficient of the vehicle 
%          - scalar Cd, the drag coefficient of the vehicle 

% Outputs: - vector F, the total aerodynamic force. 

% Everything is S.I units

function [F] = aerodynamic_force(rho, v, r, q, Cl, Cd)
    %Constants of the model 
    R = 6371.37e3;                                  %Earth mean radius
    S = 41;                                         %Vehicle surface area

    %Compute the lift unit vector
    r(3) = r(3)+R;
    d = dot(r,v);
    if (d == 0)
        gamma = 0;
    else
        gamma = pi/2-acos(dot(r,v)/(norm(r)*norm(v)));
    end
    Q = quaternion2matrix(q);                       
    ul = Q*[-sin(gamma); 0; cos(gamma)];
        
    %Compute the associated forces 
    D = -(1/2)*Cd*rho*S*norm(v)*v;                  %Drag force
    L = (1/2)*Cl*rho*S*norm(v)^2*ul;                %Lift force
    Q = zeros(3,1);                                 %Laterial force
    
    %Total aerodynamic force in the LVLH frame 
    F = D+L+Q;
end