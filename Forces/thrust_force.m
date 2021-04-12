%% STEINER %% 
% STEINER team
% Date: 27/03/21
% File: thrust_force.m 
% Issue: 0 
% Validated: 

%% Thrust force %% 
% This script provides a function to compute the thrust force acting on the vehicle. 

% Inputs:  

% Outputs: - vector T, the total thrust force. 

% Everything is S.I units

function [T] = thrust_force(q)
    %Norm of the thrust force 
    T = 100;                    %Thrust foce
    
    %Vector resolution
    Q = quaternion2matrix(q);      %Attitude of the body frame with respect to the LVLH frame 
    u =  [0; 0; 1];                %Thrust vector in the body frame 
    u = Q.'*u;                     %Thrust vector in the LVLH frame 
    
    %Output 
    T = T*u;                       %Thrust vector
end