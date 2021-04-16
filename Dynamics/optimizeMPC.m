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
function [c, ceq] = nonlcon(mu, I, Dt, s0)
    %Optimization variables 
    u = commands(1); 
    alpha = commands(2); 
    M = commands(3:5);
    
    %Constraint constants 
    R = 6371.137;               %Earth mean radius
    num_cons = 7;               %Number of non linear constraints
    sm = 1e4;                   %Final mass
    gamma_min = 9.81;           %Min acceleration norm
    gamma_max = 2*g;            %Max acceleration norm
    theta_min = -deg2rad(75);   %Minimum pitch angle
    theta_max = deg2rad(75);    %Maximum pitch angle
    sh = R+200e3;               %Maximum flight altitude
    
    %Integration tolerances 
    RelTol = 2.25e-14; 
    AbsTol = 1e-22; 
    options = odeset('RelTol', RelTol, 'AbsTol', AbsTol, 'Events', @(t,s)crash_event(s));
    
    %Integrate the trajectory 
    [t, S] = ode113(@(t,s)final_dynamics(mu, t, s, I, M, u, alpha), Dt, s0, options);
    
    %Final boundary conditions
    rf = S(end,3).';
    mf = S(end,end); 
    
    %Nonlinear constraints
    c = zeros(num_cons, length(t));
    for i = 1:length(t)
        gamma = final_dynamics(mu, t, S(i,:).', I, M(:,i), u(i), alpha(i));
        gamma = norm(gamma(4:6));
        h = S(i,3);
        if (i == length(t))
            cons = [0; 0; gamma-gamma_max; gamma-gamma_min; theta-theta_min; theta-theta_max; h-sh];
        else
            cons = [0; mf-sm; gamma-gamma_max; gamma-gamma_min; theta-theta_min; theta-theta_max; h-sh];
        end
        c = [c cons]; 
    end
    
    c(1,end) = 
    ceq = [zeros(length(t),1) rf];
end
