%% STEINER %% 
% STEINER team
% Date: 27/03/21
% File: test_4.m 
% Issue: 0 
% Validated: 

%% Test 4 %%
% This scripts provides the function to optimize the trajectory using
% OptimTraj

%% General setup 
set_graphics();

%% Vehicle characteristics
R = 6371.137e3;                 %(m)      %Earth mean radius
L = 1e7;                        %(m)      %Range
mu = 3.986e14;                  %         %Earth gravitational parameter
mTotal = 505846;                %(kg)     %Total lift-off mass
mFuel = 0.8*mTotal;             %(kg)     %mass of the fuel
mEmpty = mTotal-mFuel;          %(kg)     %mass of the rocket (without fuel)
I = 1e8*eye(3);                 %(kg m^2) %Inertia dyadic
Tmax = 5885000;                 %(N)      %Maximum thrust
g = 9.81;                       %(m/s^2)  %standard gravity acceleration
absTheta = deg2rad(75);         %(rad)    %maximum pitch angle

%% Problem bounds
%Boundary conditions
r0 = zeros(3,1);        %Vehicle starts on the ground
v0 = zeros(3,1);        %Vehicle starts stationary
q0 = [1; 0; 0; 0];      %Initial attitude quaternion 
omega0 = zeros(3,1);    %Initial angular velocity
lambda0 = deg2rad(40);  %Initial latitude
tau0 = deg2rad(-10);    %Initial longitude
m0 = mTotal;            %Vehicle starts full of fuel
rf = zeros(3,1);        %Vehicle ends on the ground
vf = zeros(3,1);        %Vehicle ends stationary
qf = [1; 0; 0; 0];      %Final attitude quaternion 
omegaf = zeros(3,1);    %Final angular velocity
mf = mEmpty;            %Vehicle may end at maximum with no fuel

%% Path constraints
xUpp = inf;             %Range constraint
hLow = 0;               %Cannot go through the Earth!
hUpp = 200e3;           %Suborbital fligth constraint
vLow = -inf;            %Mach 6 constraint
vUpp = inf;             %Mach 6 constraint
mLow = mEmpty;          %All fuel got consumpted
mUpp = mTotal;          %Vehicle starts full of fuel

qLow = [cos(-absTheta/2); 0; sin(-absTheta/2); 0];      %Maximum pitch rotation
qUpp = [cos(absTheta/2); 0; sin(-absTheta/2); 0];       %Maximum pitch rotation
omegaLow = -1;                                       %Maximum angular velocity
omegaUpp = 1;                                        %Maximum angular velocity

%Control constraints
uLow = 0;                       %Minimum thrust input
uUpp = Tmax;                    %Maximum thrust output
alphaLow = -deg2rad(7);         %Maximum angle of attack
alphaUpp = deg2rad(7);          %Minimum angle of attack
MLow = -Tmax*10;                %Maximum control torque
MUpp =  Tmax*10;                %Maximum control torque

%% Problem formulation
%Time constraints
P.bounds.initialTime.low = 0;
P.bounds.initialTime.upp = 0;
P.bounds.finalTime.low = 0;
P.bounds.finalTime.upp = 2*60*60;

%State bounds
P.bounds.state.low = [0; 0; hLow; vLow*ones(3,1); tau0; lambda0; qLow; omegaLow*ones(3,1); mLow];
P.bounds.state.upp = [xUpp; xUpp; hUpp; vUpp*ones(3,1); inf; inf; qUpp; omegaUpp*ones(3,1); mUpp];

%Boundary conditions
P.bounds.initialState.low = [r0; v0; tau0; lambda0; q0; omega0; m0];
P.bounds.initialState.upp = [r0; v0; tau0; lambda0; q0; omega0; m0];
P.bounds.finalState.low = [rf; v0; tau0; lambda0; qf; omegaf; mEmpty];
P.bounds.finalState.upp = [rf; inf*ones(3,1); inf; inf; qf; omegaf; m0];

%Control boundaries
P.bounds.control.low = [uLow; alphaLow; MLow*ones(3,1)];
P.bounds.control.upp = [uUpp; alphaUpp; MUpp*ones(3,1)];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Initial Guess                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
mGuess = mEmpty;                                                            %(kg) guess at the maximum saved mass
P.guess.time = [0, P.bounds.finalTime.upp];                                 %(s)  guess at the point at which the mass is maximized
P.guess.state = [P.bounds.initialState.low,  P.bounds.finalState.low];      % state guess
P.guess.control = [P.bounds.control.upp, P.bounds.control.low];             % control guess

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Objective and Dynamic functions                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Dynamics function:
P.func.dynamics = @(t,x,u)( opt_dynamics(mu, t, x, I, u(3:end,:), u(1,:), u(2,:)) );

% Objective function:
P.func.bndObj = @(t0, x0, tF, xF)( -xF(end) );                                     %Maximize final mass
%P.func.bndCst = @(t0, x0, tF, xF)([(L/R)-acos(cos(xF(7))*cos(xF(8))), 0]);         %Maximize range

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  Options and Method selection                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

 method = 'trapezoid';
% method = 'rungeKutta';
% method = 'chebyshev';

switch method   
    case 'trapezoid'       
        P.options(1).method = 'trapezoid';
        P.options(1).defaultAccuracy = 'low';
        
        P.options(2).method = 'trapezoid';
        P.options(2).defaultAccuracy = 'medium';
        P.options(2).nlpOpt.MaxFunEvals = 2e4;
        P.options(2).nlpOpt.MaxIter = 1e5;
        
    case 'rungeKutta'
        P.options(1).method = 'rungeKutta';
        P.options(1).defaultAccuracy = 'low';
        
        P.options(2).method = 'rungeKutta';
        P.options(2).defaultAccuracy = 'medium';
        
    case 'chebyshev'     
        P.options(1).method = 'chebyshev';
        P.options(1).defaultAccuracy = 'low';
        
        P.options(2).method = 'chebyshev';
        P.options(2).defaultAccuracy = 'low';
        P.options(2).chebyshev.nColPts = 15;       
end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                              Solve!                                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
soln = optimTraj(P);

t = linspace(soln(end).grid.time(1),soln(end).grid.time(end),250);
x = soln(end).interp.state(t);
u = soln(end).interp.control(t);

figure(120);
subplot(2,2,1);
plot(t,x(1,:)/1000)
xlabel('time (s)')
ylabel('height (km)')
title('Maximal Height Trajectory')
subplot(2,2,2);
plot(t,x(3,:))
xlabel('time (s)')
ylabel('mass (kg)')
title('Goddard Rocket')
subplot(2,2,3);
plot(t,x(2,:))
xlabel('time (s)')
ylabel('velocity (m/s)')
subplot(2,2,4);
plot(t,u/1000)
xlabel('time (s)')
ylabel('thrust (kN)')

%% Auxiliary functions 
function set_graphics()
    %Set graphical properties
    set(groot, 'defaultAxesTickLabelInterpreter', 'latex'); 
    set(groot, 'defaultAxesFontSize', 11); 
    set(groot, 'defaultAxesGridAlpha', 0.3); 
    set(groot, 'defaultAxesLineWidth', 0.75);
    set(groot, 'defaultAxesXMinorTick', 'on');
    set(groot, 'defaultAxesYMinorTick', 'on');
    set(groot, 'defaultFigureRenderer', 'painters');
    set(groot, 'defaultLegendBox', 'off');
    set(groot, 'defaultLegendInterpreter', 'latex');
    set(groot, 'defaultLegendLocation', 'best');
    set(groot, 'defaultLineLineWidth', 1); 
    set(groot, 'defaultLineMarkerSize', 3);
    set(groot, 'defaultTextInterpreter','latex');
end
