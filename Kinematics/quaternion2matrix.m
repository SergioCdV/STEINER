%% STEINER %% 
% STEINER team
% Date: 09/04/21
% File: quaternion2matrix.m 
% Issue: 0 
% Validated: 

%% Quaternion to rotation matrix %% 
% This script provides a function to compute the rotation matrix associated to a quaternion vector. 

% Inputs: - vector q, the quaternion parametrization

% Outputs: - matrix Q, the associated rotation matrix. 

% Everything is S.I units

function [Q] = quaternion2matrix(q)
    %Main computation 
    S = [0 -q(4) q(3); q(4) 0 -q(2); -q(3) q(2) 0];                         %Hat map of the quaternion
    Q = (q(1)^2-dot(q(2:4),q(2:4)))*eye(3)+2*q(2:4)*q(2:4).'+2*q(1)*S;      %Rotation matrix 
end