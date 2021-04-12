%% STEINER %% 
% STEINER team
% Date: 27/03/21
% File: crash_event.m 
% Issue: 0 
% Validated: 

%% Crash event %% 
% This script provides a function to detect crashing events. 

% Inputs:  - state vector ds
% Outputs: - detection of the crashing event. 

function [value, isterminal, direction] = crash_event(s)    
    %Earth mean radius 
    R = 6371.37e3; 

    %Main computation 
    value = s(3)-R;
    isterminal = 1; 
    direction = -1;
end