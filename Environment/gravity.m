%% STEINER %% 
% STEINER team
% Date: 27/03/21
% File: gravity.m 
% Issue: 0 
% Validated: 

%% Gravity %% 
% This script provides a function to compute the Earth gravity acceleration. 

% Inputs: - vector r, the LVLH position vector of the vehicle. 
%         - scalar lambda, the geodetic latitude of the vehicle

% Outputs: - scalar g, containing the gravity acceleration at that point. 

% Everything is S.I units

function [g] = gravity(r, lambda)
    %Constants of the model 
    Re = 6371e3;                %Earth mean radius
    mu = 3.986e14;              %Gravitational parameter of the Earth
    J2 = 0.0010827;             %Second zonal harmonic of the Earth
    
    %State variables 
    y = r(2);                   %LVLH y coordinate
    z = r(3);                   %LVLH z coordinate
    
    %Inertial z coordinate in the LVLH frame
    u = [0; cos(lambda); sin(lambda)];
        
    %Gravity J2 pertubed acceleration 
    g = -(mu/norm(r)^3)*(r ...
        -(1/2)*J2*(Re/norm(r))^2*(6*(y*cos(lambda)+z*sin(lambda))*u  ...
        +(3-(15/norm(r)^2)*(y*cos(lambda)+z*sin(lambda))^2*r)));
end