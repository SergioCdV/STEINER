%% STEINER %% 
% STEINER team
% Date: 27/03/21
% File: atmosphere.m 
% Issue: 0 
% Validated: 

%% Atmosphere %% 
% This script provides a function to compute the Earth atmospheric characteristics (density, pressure, temperature) 
% at a given geopotential altitude h. 

% Inputs:  - string model, determining which atmosphere model to use to
%          compute the atmospheric density. 
%          - scalar g, determining the gravity acceleration. 
%          - scalar h, geopotential altitude at which to compute the
%          atmosphere state variables. 

% Outputs: - array state, containing in this order the density, pressure
%           and temperature of the atmosphere. 

% Everything is S.I units

function [state] = atmosphere(model, g, h)
    %Switch between models 
    switch (model)
        case 'ISA'
            [state] = ISA_model(g, h);
        otherwise
            disp('No valid atmospheric model was selected'); 
            state = [];
    end
end

%% Auxiliary functions 
%ISA model of the atmosphere (up to 100 km)
function [state] = ISA_model(g, r)
    %Layer-defining altitudes 
    h0 = zeros(8,1);    
    h0(2) = 11e3;         %Tropopause/stratosphere I layer altitude
    h0(3) = 20e3;         %Stratosphere II altitude
    h0(4) = 32e3;         %Stratosphere III altitude
    h0(5) = 47e3;         %Stratopause altitude
    h0(6) = 51e3;         %Mesosphere altitude
    h0(7) = 71e3;         %Mesopause altitude
    h0(8) = 84.852e3;     %Mesopause altitude
        
    %Model constants
    g0 = 9.81;                                                              %Gravity acceleration at sea level
    Re = 6371.37e3;                                                         %Earth mean radius
    Rg = 287.052;                                                           %Air ideal gas constant
    alpha = [-6.5e-3; 0; 1e-3; 2.8e-3; 0; -2.8e-3; -2e-3; 0];               %Thermal gradient for all layers
    p0 = [101325; 22632; 5474.9; 868.02; 110.91; 66.939; 3.9564; 03734];    %Initial pressure for all layers
    T0 = [288.15; 216.65; 216.65; 228.65; 270.65; 270.65; 214.65; 186.87];  %Initial temperature for all layers   
  
    %Re-compute the altitude
    h = r+[0; 0; Re]; 
    h = norm(h);
    h = h-Re;
    if (h < 0)
        h = Re;
    end
    
    %Determine the atmosphere layer 
    GoOn = true; 
    layer = 1; 
    while (GoOn)
        if ((h < h0(layer+1)) || (layer + 1 == length(h0)))
            GoOn = false; 
        else
            layer = layer+1;
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
    end
    
    %Output 
    state = [rho; p; T];                                            %Atmosphere state variables
end