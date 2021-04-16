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
    %Main computation 
    value = s(3);
    isterminal = 1; 
    direction = -1;
end