%% STEINER %% 
% STEINER team
% Date: 27/03/21
% File: gravity.m 
% Issue: 0 
% Validated: 

%% Gravity %% 
% This script provides a function to compute the Earth gravity acceleration at a given geopotential altitud h. 

% Inputs: - scalar h, geopotential altitude at which to compute the
%           atmosphere state variables. 

% Outputs: - scalar g, containing the gravity acceleration at that point. 

% Everything is S.I units

function [g] = gravity(h)
    %Constants of the model 
    Re = 6371e3;                %Earth mean radius
    mu = 3.986e14;              %Gravitational parameter of the Earth
    J2 = 0.0010827;             %Second zonal harmonic of the Earth
    
    %Keplerian term 
    g = -mu/(Re+h)^2;  
    
    %Second zonal harmonic term 
    g = g + 3*J2*(Re/(Re+h))^2;
end