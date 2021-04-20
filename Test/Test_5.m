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
Tmax = 5885000;                 %(N)      %Maximum thrust
g = 9.81;                       %(m/s^2)  %standard gravity acceleration

%% Problem bounds
%Boundary conditions
r0 = zeros(2,1);        %Vehicle starts on the ground
v0 = [10; deg2rad(87)]; %Vehicle starts stationary
rf = zeros(2,1);        %Vehicle ends on the ground
vf = [10; 0];           %Vehicle ends stationary
m0 = mTotal;            %Vehicle starts full of fuel
mf = mEmpty;            %Vehicle may end at maximum with no fuel

%% Path constraints
xUpp = L;               %Range constraint
hLow = 0;               %Cannot go through the Earth!
hUpp = 200e3;           %Suborbital fligth constraint
vLow = 0;               %Mach 6 constraint
vUpp = inf;             %Mach 6 constraint
mLow = mEmpty;          %All fuel got consumpted
mUpp = mTotal;          %Vehicle starts full of fuel

%Control constraints
uLow = 0;                       %Minimum thrust input
uUpp = Tmax;                    %Maximum thrust output
alphaLow = -deg2rad(7);         %Maximum angle of attack
alphaUpp = deg2rad(7);          %Minimum angle of attack

%% Problem formulation
%Time constraints
P.bounds.initialTime.low = 0;
P.bounds.initialTime.upp = 0;
P.bounds.finalTime.low = 0;
P.bounds.finalTime.upp = 2*60*60;

%State bounds
P.bounds.state.low = [0; hLow; vLow; 0; mLow];
P.bounds.state.upp = [xUpp; hUpp; vUpp; pi/2; mUpp];

%Boundary conditions
P.bounds.initialState.low = [r0; v0; m0];
P.bounds.initialState.upp = [r0; v0; m0];
P.bounds.finalState.low = [rf; vf; mEmpty];
P.bounds.finalState.upp = [rf; vf; m0];

%Control boundaries
P.bounds.control.low = [uLow; alphaLow];
P.bounds.control.upp = [uUpp; alphaUpp];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Initial Guess                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
mGuess = mEmpty;                                                            %(kg) guess at the maximum saved mass
P.guess.time = [0, P.bounds.finalTime.upp];                                 %(s)  guess at the point at which the mass is maximized
P.guess.state = [P.bounds.initialState.low,  P.bounds.finalState.upp];      % state guess
P.guess.control = [P.bounds.control.upp, P.bounds.control.low];             % control guess

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Objective and Dynamic functions                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Dynamics function:
P.func.dynamics = @(t,x,u)( opt_simple_dynamics(t, x, u(1,:), u(2,:)) );

% Objective function:
P.func.bndObj = @(t0, x0, tF, xF)( -xF(end) );                              %Maximize final mass
%P.func.bndCst = @(t0, x0, tF, xF)([L - (xF(1)-x0(1))]);                    %Maximize range

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
        P.options(1).nlpOpt.MaxFunEvals = 2e4;
        
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
