%% STEINER %% 
% STEINER team
% Date: 30/04/21
% File: Control_1.m 
% Issue: 0 
% Validated: 

%% Contorl 1 %%
% This scripts provides the function to test the control model of the
% vehicle

%% General setup 
set_graphics();

%% Earth characteristics 
R = 6371.37e3;              %Earth mean radius
omega = (2*pi)/(3600*24);   %Earth mean angular velocity
mu = 3.986e14;              %Earth gravitational parameter 

%% Vehicle's characteristics and trajectory
T0 = 400*9.81;          %ISP of the aerospike
alpha = deg2rad(5);     %Angle of attack

%Trajectory
x1 = [0         0    0.0010         0    0.5876;
      0.0018    0.0014    0.0012    0.0000    0.5877;
      0.0100    0.0042    0.0016    0.0000    0.5870;
      0.0347    0.0120    0.0021    0.0000    0.5846;
      0.0744    0.0195    0.0024    0.0000    0.5808;
      0.1324    0.0315    0.0028    0.0000    0.5759;
      0.2022    0.0422    0.0028    0.0000    0.5698;
      0.2806    0.0570    0.0031    0.0000    0.5634;
      0.3608    0.0669    0.0031    0.0000    0.5567;
      0.4335    0.0746    0.0034    0.0000    0.5499;
      0.5099    0.0807    0.0036    0.0000    0.5428;
      0.5777    0.0926    0.0038    0.0000    0.5359;
      0.6493    0.1151    0.0041    0.0000    0.5286;
      0.7104    0.1487    0.0043    0.0000    0.5219;
      0.7707    0.2037    0.0047    0.0000    0.5146;
      0.8230    0.2684    0.0048    0.0000    0.5086;
      0.8723    0.3417    0.0053    0.0000    0.5023;
      0.9229    0.4153    0.0055    0.0000    0.4966;
      0.9626    0.5019    0.0058    0.0000    0.4934;
      0.9887    0.5853    0.0054    0.0000    0.4909;
      0.9954    0.6332    0.0052    0.0000    0.4902;
      1.0000    1.0000    0.0050         0    0.4900];
x2 = [0.1000    0.1000    0.0005         0    0.0210;
      0.1356    0.1010    0.0020    0.0000    0.0210;
      0.2910    0.0957    0.0020    0.0000    0.0210;
      0.5759    0.0944    0.0020    0.0000    0.0210;
      0.9841    0.0984    0.0020    0.0000    0.0210;
      1.5005    0.0983    0.0020    0.0000    0.0210;
      2.1027    0.0983    0.0020    0.0000    0.0210;
      2.7657    0.0983    0.0020    0.0000    0.0210;
      3.4648    0.0994    0.0020    0.0000    0.0210;
      4.1764    0.1005    0.0020    0.0000    0.0210;
      4.8800    0.1013    0.0020    0.0000    0.0210;
      5.5571    0.1024    0.0020    0.0000    0.0210;
      6.1932    0.1023    0.0020    0.0000    0.0210;
      6.7763    0.1025    0.0020    0.0000    0.0210;
      7.2980    0.1010    0.0020    0.0000    0.0210;
      7.7514    0.1027    0.0020    0.0000    0.0210;
      8.1333    0.1020    0.0020    0.0000    0.0210;
      8.4413    0.0966    0.0020    0.0000    0.0210;
      8.6739    0.0987    0.0020    0.0000    0.0210;
      8.8327    0.1040    0.0020    0.0000    0.0210;
      8.8981    0.1034    0.0020    0.0000    0.0210;
      9.0000    0.1000    0.0012         0    0.0210]; 
x3 = [0.9000    0.0100    0.0001         0    0.0021;
      0.9127    0.0060    0.0001   -0.0000    0.0021;
      0.9582    0.0022    0.0001   -0.0000    0.0021;
      0.9938    0.0017    0.0001   -0.0000    0.0021;
      1.0000         0    0.0000   -0.0000    0.0021];

x1 = x1*1e5;        %Climbing phase
x2 = 1e6*x2;        %Cruise phase
x3 = 1e7*x3;        %Landing phase
x = [x1; x2; x3];   %Re-entry

%% Compute the Jacobian and control matrix 
%Preallocate the matrices 
A = zeros(size(x,1), size(x,2), size(x,2));         %Jacobian of the system 
B = zeros(size(x,1), size(x,2), 1);                 %Control matrix

%Compute those matrices
for i = 1:size(x,1)
    %Compute the Jacobian of the system 
    A(i,:,:) = numerical_jacobian(@(s)control_dynamics(s), x(i,:));
    
    %Compute the control matrix 
    B(i,:,:) = [0; 0; 1/x(i,end); 0; -1/T0];
end

%% Auxiliary functions 
function [dx] = control_dynamics(x)
    %Constants of the model 
    alpha = deg2rad(5);         %Angle of attack
    u = 0;                      %Control law
    Re = 6371.37e3;             %Earth mean radius
    mu = 3.986e14;              %Gravitational parameter of the Earth
    J2 = 0.0010827;             %Second zonal harmonic of the Earth
    T0 = 400*9.81;              %Maximum thrust
    S = 60;                     %Cross sectional area
    Cl = 1.2;                   %CL0 
    beta = 2*pi;                %Lift slope
    
    CoF(1,:) = [2.61059846050e-2;
            -8.57043966269e-2;
            1.07863115049e-1;
            -6.44772018636e-2;
            1.64933626507e-2;
            0];
    CoF(2,:) = [1.37368651246e0;
                -4.57116286752e0;
                5.72789877344e0;
                -3.25219000620e0;
                7.29821847445e-1;
                0];
    CoF(3,:) = [1.23001735612e0;
                -2.97244144190e0;
                2.78009092756e0;
                -1.16227834301e0;
                1.81868987624e-1;
                0];
    CoF(4,:) = [1.42392902737e1;
                -3.24759126471e1;
                2.96838643792e1;
                -1.33316812491e1;
                2.87165882405e0;
                -2.27239723756e-1];
    CoF(5,:) = [0.11969995703e6;
                -0.14644656421e5;
                -0.45534597613e3;
                0.49544694509e3;
                -0.46253181596e2;
                0.12000480258e1];
    CoF(6,:) = [-0.35217318620e6;
                0.51808811078e5;
                0.23143969006e4;
                -0.22482310455e4;
                0.20894683419e3;
                -0.53807416658e1];
    CoF(7,:) = [0.60452159152e6;
                -0.95597112936e5;
                -0.38860323817e4;
                0.39771922607e4;
                -0.36835984294e3;
                0.94529288471e1];
    CoF(8,:) = [-0.43042985701e6;
                0.83271826575e5;
                0.12357128390e4;
                -0.30734191752e4;
                0.29388870979e3;
                -0.76204728620e1];
    CoF(9,:) = [0.13656937908e6;
                -0.32867923740e5;
                0.55572727442e3;
                0.10635494768e4;
                -0.10784916936e3;
                0.28552696781e1];
    CoF(10,:) = [-0.16647992124e5;
                0.49102536402e4;
                -0.23591380327e3;
                -0.13626703723e3;
                0.14880019422e2;
                -0.40379767869e0];
    
    CoFZ = [-3.48643241e-2;
            3.50991865e-3;
            -8.33000535e-5;
            1.15219733e-6];

    %State variables
    h = x(:,1);                 %Altitude 
    v = x(:,3);                 %Velocity vector
    m = x(:,end);               %Mass of the vehicle
    hbar = h./1000;             %Scaled altitude

    %Gravity J2 pertubed acceleration 
    l = h(:,1) + Re;
    g = (mu./l(:,1).^3).*(l(:,1) -(1/2)*J2*(Re./l(:,1)).^2.*(6*l(:,1)+3-15*l(:,1)));

    %Density computation
    z = CoFZ(1).*hbar+CoFZ(2).*hbar.^2+CoFZ(3).*hbar.^3+CoFZ(4).*hbar.^4;
    r = 1.0228066.*exp(-z);
    y = -0.12122693.*hbar+r-1.0228055;
    rho = 1.225.*exp(y);

    %Speed of sound and Mach number
    theta = 292.1-8.87743.*hbar+0.193315.*hbar.^2+(3.72e-3).*hbar.^3;
    a = 20.0468.*sqrt(theta);
    M = v./a;
    beta = beta./(sqrt(1-M.^2));

    %Lift and drag
    q = 0.5.*rho.*v.*v.*S;

    M0 = M.^0;
    M1 = M.^1;
    M2 = M.^2;
    M3 = M.^3;
    M4 = M.^4;
    M5 = M.^5;

    numeratorCD0 = CoF(1,1).*M0+CoF(1,2).*M1+CoF(1,3).*M2+CoF(1,4).*M3+CoF(1,5).*M4;
    denominatorCD0 = CoF(2,1).*M0+CoF(2,2).*M1+CoF(2,3).*M2+CoF(2,4).*M3+CoF(2,5).*M4;
    Cd0 = numeratorCD0./denominatorCD0;
    numeratorK = CoF(3,1).*M0+CoF(3,2).*M1+CoF(3,3).*M2+CoF(3,4).*M3+CoF(3,5).*M4;
    denominatorK = CoF(4,1).*M0+CoF(4,2).*M1+CoF(4,3).*M2+CoF(4,4).*M3+CoF(4,5).*M4+CoF(4,6).*M5;
    K = numeratorK./denominatorK;
    D = q.*(Cd0+K.*(Cl+beta.*alpha).^2);
    L = q.*(Cl+beta.*alpha);

    %Thrust computation
    T = u;

    %State variables 
    y = h;                       %Position vector 
    V = x(:,3);                  %Velocity norm
    gamma = x(:,4);              %Flight path angle
    dm = -sqrt(T(:,1).^2)/T0;    %Mass dynamics

    %Dynamical variables
    dx = [V.*(Re./(Re+y)).*cos(gamma) V.*sin(gamma) (1./m).*(T-D-m.*g.*sin(gamma)) L-((g./V)-(V./(Re+y))).*cos(gamma) dm];
end

function [J] = numerical_jacobian(func, x)
    %Constants 
    order = 4;                              %Length of the stencil
    e = 1e-3;                               %Pertubation step 
    delta = eye(length(x));                 %Identity matrix / Kronecker delta
    
    %Preallocation 
    J = zeros(length(x), length(x));        %Preallocation of the Jacobian 
    P = repmat(x.', 1, order);                %Preallocation of the perturbed vectors
    
    %Main computation
    for i = 1:length(x) 
        P(:,1) = P(:,1) + 2 * e * delta(:,i); 
        P(:,2) = P(:,2) + e * delta(:,i); 
        P(:,3) = P(:,3) - e * delta(:,i); 
        P(:,4) = P(:,4) - 2 * e * delta(:,i); 
        J(:,i) = (1/(12*e))*( feval(func, P(:,4).').'-feval(func, P(:,1).').' ...
                              +8*(feval(func, P(:,2).').'-feval(func, P(:,3).').') );
        P = repmat(x.', 1, order);               
    end
end

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