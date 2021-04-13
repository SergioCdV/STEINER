%% STEINER %% 
% STEINER team
% Date: 27/03/21
% File: thrust_force.m 
% Issue: 0 
% Validated: 

%% Thrust force %% 
% This script provides a function to compute the thrust force acting on the vehicle. 

% Inputs:  - quaternion q, relating the LVLH frame and the body frame 
%          - scalar u, the thrust norm

% Outputs: - vector T, the total thrust force. 

% Everything is S.I units

function [T] = thrust_force(q, u)
    %Norm of the thrust force 
    T = u;                          %Thrust foce
    
    %Vector resolution
    Q = quaternion2matrix(q);       %Attitude of the body frame with respect to the LVLH frame 
    ut =  [1; 0; 0];                %Thrust vector in the body frame 
    ut = Q.'*ut;                    %Thrust vector in the LVLH frame 
    
    %Output 
    T = T*ut;                       %Thrust vector
end