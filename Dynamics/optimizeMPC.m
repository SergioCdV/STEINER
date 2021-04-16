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

function [commands] = optimizeMPC(finalHorizonIndex, mu, I, Dt, s0)
    %Constants 
    Tmax = 100; 
    
    %Optimization options 
    %options = optimoptions('fmincon', 'Display', 'iter', 'MaxFunctionEvaluations', 1000*N*T, 'UseParallel', false);
    
    %Cost function 
    costfun = @(s)(-s(end));
    
    %Initial guess
    x0 = zeros(5,finalHorizonIndex); 
    
    %Linear constraints 
    A = []; 
    b = [];
    Aeq = []; 
    beq = [];
    
    %Upper and lower bounds
    lb = [zeros(1,finalHorizonIndex); -deg2rad(5)*ones(1,finalHorizonIndex); -Tmax*ones(3,finalHorizonIndex)];
    ub = [Tmax*ones(1,finalHorizonIndex); deg2rad(5)*ones(1,finalHorizonIndex); Tmax*ones(3,finalHorizonIndex)];
    
    %Optimization 
    commands = fmincon(@(s)costfun(s), x0, A, b, Aeq, beq, lb, ub);%@(s)nonlcon(mu, I, Dt, s0, s));
end

%% Auxiliary functions
%Nonlinear and linear constraints 
function [c, ceq] = nonlcon(mu, I, Dt, s0, commands)
    %Optimization variables 
    u = commands(1,:); 
    alpha = commands(2,:); 
    M = commands(3:5,:);
    
    %Constraint constants 
    R = 6371.137;               %Earth mean radius
    num_cons = 7;               %Number of non linear constraints
    gamma_min = 9.81;           %Min acceleration norm
    gamma_max = 2*9.81;         %Max acceleration norm
    theta_min = -deg2rad(75);   %Minimum pitch angle
    theta_max = deg2rad(75);    %Maximum pitch angle
    sh = R+200e3;               %Maximum flight altitude
    dL = 10e6/R;                %Arc length to reach (rad)
    
    %Integration tolerances 
    RelTol = 2.25e-14; 
    AbsTol = 1e-22; 
    options = odeset('RelTol', RelTol, 'AbsTol', AbsTol, 'Events', @(t,s)crash_event(s));
    
    %Integrate the trajectory 
    dt = Dt(2)-Dt(1);
    S = zeros(length(Dt),length(s0)); 
    S(1,:) = s0;
    for i = 1:length(Dt)
    	[~, saux] = ode113(@(t,s)final_dynamics(mu, t, s, I, M(:,i), u(i), alpha(i)), [0 dt], S(i,:), options);
        S(i+1,:) = saux(end,:); 
    end
    
    %Final boundary conditions
    rf = S(end,3).';
    
    %Nonlinear constraints
    c = zeros(num_cons, length(Dt));
    for i = 1:length(Dt)
        q = S(i,9:12); 
        Q = quaternion2matrix(q);
        theta = asin(-Q(3,1));
        gamma = final_dynamics(mu, 0, S(i,:).', I, M(:,i), u(i), alpha(i));
        gamma = norm(gamma(4:6));
        h = S(i,3);
        if (i ~= length(Dt))
            cons = [0; -gamma+gamma_min; gamma-gamma_max; -theta+theta_min; theta-theta_max; -h; h-sh];
        else
            cons = [0; -gamma+gamma_min; gamma-gamma_max; -theta+theta_min; theta-theta_max; -h; h-sh];
        end
        c(:,i) = cons; 
    end
    
    c(1,end) = dL-acos(cos(S(end,7))*cos(S(end,8)));
    ceq = rf;
end
