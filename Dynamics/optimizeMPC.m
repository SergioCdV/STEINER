%% STEINER %% 
% STEINER team
% Date: 15/04/21
% File: optimizeMPC.m 
% Issue: 0 
% Validated: 

%% Optimize MPC %%
% This scripts provides the nonlinear MPC scheme to optimize the
% trajectory.

% The optimizations takes as the cost function the maximization of the
% vehicle mass (min fuel consumption), whiel satysfying the vehicle
% dynamics and range, time, max acceleration, initial and final conditions
% and attitude constraints.

% Inputs: -
% Outpus: -  vector commands, containing the needed thrust norm and torque
%            vector to optimize the trajectory

function [commands] = optimizeMPC(finalHorizonIndex, Tmax)
    %Optimization options 
    options = optimoptions('fmincon', 'Display', 'iter', 'MaxFunctionEvaluations', 1000*N*T, 'UseParallel', false);
    
    %Cost function 
    costfun = @(s)(s(end)*ones(1,finalHorizonIndex));
    
    %Initial guess
    x0 = [Tmax; 0; zeros(3,1)]; 
    
    %Linear constraints 
    A = []; 
    b = [];
    Aeq = []; 
    beq = [];
    
    %Upper and lower bounds
    lb = [zeros(1,finalHorizonIndex); deg2rad(5)*ones(3,finalHorizonIndex); -Inf*ones(3,finalHorizonIndex)];
    ub = [Tmax*ones(1,finalHorizonIndex); -deg2rad(5)*ones(3,finalHorizonIndex); -Inf*ones(3,finalHorizonIndex)];
    
    %Optimization 
    commands = fmincon(costfun, x0, A, b, Aeq, beq, lb, ub, @nonlcon, options);
end

%% Auxiliary functions
%Nonlinear and linear constraints 
function [c, ceq] = nonlcon()
end
