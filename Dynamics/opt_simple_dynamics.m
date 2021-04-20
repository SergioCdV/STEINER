%% STEINER %% 
% STEINER team
% Date: 19/04/21
% File: opt_simple_dynamics.m 
% Issue: 0 
% Validated: 

%% Optimzed simple Dynamics %% 
% This script provides a function to the simplified dynamical vector field acting on the vehicle, in the needed 
% shape to be handled by the optimization process. 

% Inputs:  
% Outputs: - vector ds, containing in the vector field of the dynamics applied on the vehicle. 

% Everything is S.I units

function [ds] = opt_simple_dynamics(t, s, u, alpha)  
    %Constants of the model 
    T0 = 9.81*1000;                                 %Characteristic thrust of the vehicle
    
    %State variables 
    r = s(1:2,:);                                  %Position vector
    v = s(3:4,:);                                  %Velocity vector
    m = s(end,:);                                  %Instantenous mass
    
    %Preallocation 
    ds = zeros(size(s,1), size(s,2));
                    
    %Compute the environment conditions
    for i = 1:size(s,2)
        g = gravity(r(:,i));                                           %Gravity acceleration module
        %atmos_state = atmosphere('ISA', norm(g), r(:,i));              %State variables of the atmosphere
        
        %Forces and torques acting on the vehicle
        %[Fa] = aerodynamic_force(atmos_state, v(:,i), alpha(i));       %Aerodynamic force and torque
       
        rho = 0.5; 
        Fa = zeros(2,1);
        
        %Mass dynamics
        dm = -norm(u)/T0;                                              %Mass consumption

        %Traslational dynamics 
        dr = traslational_dynamics(g, u(i), Fa, s(:,i));

        %Complete vector field 
        ds(:,i) = [dr; dm];
    end
end

%% Auxiliary functions 
%Gravity model 
function [g] = gravity(r)
    %Constants of the model 
    Re = 6371e3;                %Earth mean radius
    mu = 3.986e14;              %Gravitational parameter of the Earth
    J2 = 0.0010827;             %Second zonal harmonic of the Earth
    
    %State variables 
    r(2) = r(2)+Re ;            %Relative position vector to the center of the Earth
    z = r(2);                   %LVLH z coordinate
    
    %Inertial z coordinate in the LVLH frame
    u = [0; 1];
        
    %Gravity J2 pertubed acceleration 
    g = -(mu/z^3)*(z -(1/2)*J2*(Re/z)^2*(6*z+3-15*z));
end

%Atmosphere state 
function [state] = atmosphere(model, g, h)
    %Switch between models 
    switch (model)
        case 'ISA'
            [state] = ISA_model(g, h);
        otherwise
            error('No valid atmospheric model was selected'); 
    end
end

%ISA model 
function [state] = ISA_model(g, r)
    %Layer-defining altitudes 
    h0 = zeros(8,1);      %Preallocation
    h0(2) = 11e3;         %Tropopause/stratosphere I layer altitude
    h0(3) = 20e3;         %Stratosphere II altitude
    h0(4) = 32e3;         %Stratosphere III altitude
    h0(5) = 47e3;         %Stratopause altitude
    h0(6) = 51e3;         %Mesosphere altitude
    h0(7) = 71e3;         %Mesopause altitude
    h0(8) = 84.852e3;     %Mesopause altitude
        
    %Model constants
    Rg = 287.052;                                                           %Air ideal gas constant
    alpha = [-6.5e-3; 0; 1e-3; 2.8e-3; 0; -2.8e-3; -2e-3; 0];               %Thermal gradient for all layers
    p0 = [101325; 22632; 5474.9; 868.02; 110.91; 66.939; 3.9564; 03734];    %Initial pressure for all layers
    T0 = [288.15; 216.65; 216.65; 228.65; 270.65; 270.65; 214.65; 186.87];  %Initial temperature for all layers   
  
    %Re-compute the altitude
    h = r(2);
    if (h < 0)
        h = 0;
    end
    
    %Determine the atmosphere layer 
    GoOn = true; 
    layer = 1; 
    while (GoOn)
        if (h < h0(layer+1))
            GoOn = false; 
        else
            layer = layer+1;
            if (layer == length(h0))
                GoOn = false;
                h = h0(end);
            end
        end
    end
    
    %ISA model
    if (alpha(layer) ~= 0)
        T = T0(layer)+alpha(layer)*(h-h0(layer));                   %Temperature
        p = p0(layer)*(T/T0(layer))^(-g/(Rg*alpha(layer)));         %Pressure    
    else
        T = T0(layer);                                              %Temperature
        p = p0(layer)*exp(-g/(Rg*T0(layer))*(h-h0(layer)));         %Pressure
    end
    
    rho = p/(Rg*T);                                                 %Air density
    
    if (rho == 0)
        disp('');
    elseif (imag(rho) ~= 0)
        disp('')
    elseif (isnan(rho))
        disp('');
    end
    
    %Output 
    state = [rho; p; T];                                            %Atmosphere state variables
end

%Function to compute aerodynamic forces
function [F] = aerodynamic_force(atmos_state, v, alpha)
    %Constants of the model 
    S = 41;                                         %Vehicle surface area
    rho = atmos_state(1);                           %Atmosphere density
    T = atmos_state(3);                             %Atmosphere temperature
    R = 287;                                        %Air ideal gas constant 
    gamma = 1.4;                                    %Air heat coefficient ratio
    
    %Mach number 
    M = norm(v)/sqrt(gamma*T*R);
    
    %Compute the lift and drag coefficients
    if (M > 1.1)
        Cl = 4*alpha/sqrt(M^2-1);                   %Lift coefficient
        Cd = Cl^2;                                  %Drag coefficient
    elseif (M < 0.7)
        Cl = 2*pi*alpha/sqrt(1-M^2);                %Lift coefficient
        Cd = Cl^2;                                  %Drag coefficient
    else
        Cl = 0;                                     %Lift coefficient
        Cd = 1;                                     %Drag coefficient
    end
            
    %Compute the associated forces 
    L = (1/2)*Cl*rho*S*norm(v)^2*[0; 1];                %Lift force
    D = (1/2)*Cd*rho*S*norm(v)^2*[1; 0];                %Drag force
    
    %Total aerodynamic force in the LVLH frame 
    F = D+L;
end

%Traslational dynamics 
function [ds] = traslational_dynamics(g, T, F, s)
    %Constants of the model 
    R = 6371.37e3;               %Earth mean radius

    %State variables 
    y = s(2);                    %Position vector 
    V = s(3);                    %Velocity norm
    gamma = s(4);                %Flight path angle
    m = s(end);                  %Vehicle's mass
    
    %Dynamical variables
    ds = [V*(R/(R+y))*cos(gamma); 
          V*sin(gamma); 
          (1/m)*(T-F(2)-m*g*sin(gamma)); 
          F(1)-((g/V)-(V/(R+y)))*cos(gamma)];
end