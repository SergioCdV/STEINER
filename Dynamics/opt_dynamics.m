%% STEINER %% 
% STEINER team
% Date: 19/03/21
% File: opt_dynamics.m 
% Issue: 0 
% Validated: 

%% Optimization Dynamics %% 
% This script provides a function to the dynamical vector field acting on the vehicle, in the needed form to be used in 
% the optimization process. 

% Inputs:  
% Outputs: - vector ds, containing in the vector field of the dynamics applied on the vehicle. 

% Everything is S.I units

function [ds] = opt_dynamics(mu, t, s, I, M, u, alpha)  
    %Constants of the model 
    mp = 1e4;                                      %Propellent mass 
    T0 = 9.81*500;                                 %Characteristic thrust of the vehicle
    
    %State variables 
    r = s(1:3,:);                                  %Position vector
    v = s(4:6,:);                                  %Velocity vector
    lambda = s(8,:);                               %Geodetic latitude
    q = s(9:12,:);                                 %Attitude quaternion
    m = s(end,:);                                  %Instantenous mass
    
    %Preallocation 
    ds = zeros(size(s,1), size(s,2));
                    
    %Compute the environment conditions
    for i = 1:size(s,2)
        g = gravity(r(:,i), lambda(i));                                %Gravity acceleration module
        atmos_state = atmosphere('ISA', norm(g), r(:,i));              %State variables of the atmosphere
        
        %Forces and torques acting on the vehicle
        [Fa, Ta] = aerodynamic_force(atmos_state, v(:,i), q(:,i), alpha(i));            %Aerodynamic force and torque
        T = thrust_force(q(:,i), u(i));                                                 %Thrust force
       
        %Mass dynamics
        if (m(i) <= mp)
            T = zeros(3,1);                                               %Propellent mass constraint
        end
        dm = -norm(T)/T0;                                                 %Mass consumption

        %Traslational dynamics 
        dr = traslational_dynamics(g, T, Fa, s(:,i));

        %Attitude dynamics
        dtheta = attitude_dynamics(mu, dm, I, Ta, M(:,i), dr, s(:,i));

        %Complete vector field 
        ds(:,i) = [dr; dtheta; dm];
    end
end

%% Auxiliary functions 
%Traslational dynamics 
function [ds] = traslational_dynamics(g, T, Fa, s)
    %Constants of the model 
    R = 6371.37e3;               %Earth mean radius
    omega = (2*pi)/(3600*24);    %Angular velocity of the Earth
    
    %State variables 
    r = s(1:3);                  %Position vector 
    v = s(4:6);                  %Velocity vector
    lambda = s(8);               %Geodetic latitude
    m = s(end);                  %Vehicle's mass
        
    %Kinematic equations 
    dr = v;    
    
    %Longitude equations 
    dtau = v(1)/((R+r(3))*cos(lambda));
    
    %Latitude equation 
    dlambda = v(2)/(R+r(3)); 
    
    %Dynamic equations 
    dv = g+(1/m)*(T+Fa); 
    dv(1) = dv(1)-2*omega*(v(3)*cos(lambda)-v(2)*sin(lambda))-(v(1)*(v(3)-v(2)*tan(lambda)))/(R+r(3));
    dv(2) = dv(2)-omega*sin(lambda)*(omega*(R+r(3))*cos(lambda)+2*v(1))-(v(2)*v(3)+v(1)^2*tan(lambda))/(R+r(3));
    dv(3) = dv(3)+omega*cos(lambda)*(omega*(R+r(3))*cos(lambda)+2*v(1))+(v(1)^2+v(2)^2)/(R+r(3));
           
    %Traslational vector field 
    ds = [dr; dv; dtau; dlambda];
end

%Attitude dynamics 
function [ds] = attitude_dynamics(mu, dm, I, M, T, dr, s)    
    %Constants of the model 
    R = 6371.37e3;                  %Mean Earth radius
    omega_e = (2*pi)/(3600*24);     %Mean Earth angular velocity 
    dI = zeros(3,3);                %Approximation of the derivative of the inertia tensor with respect to the mass
    
    %State variables 
    r = s(1:3);                                      %Position vector
    lambda = s(8);                                   %Latitude of the aircraft
    dlambda = s(5)/(R+s(3));                         %Time derivative of the latitude
    dtau = s(4)/((R+s(3))*cos(lambda));              %Time derivative of the latitude
    q = s(9:12);                                     %Attitude quaternion
    omega = s(13:15);                                %Angular velocity 
    
    %Rotation matrix
    Q = quaternion2matrix(q);                        %Rotation matrix from the LVLH frame to the body frame
    
    %Kinematics equations
    dqe = -(1/2)*omega.'*q(2:4);
    dqv = -(1/2)*cross(omega, q(2:4))+(1/2)*q(1)*omega; 
    dq = [dqe; dqv];
    
    %Total angular velocity 
    omega01 = Q*[-dlambda; (omega_e+dtau)*cos(lambda); (omega_e+dtau)*sin(lambda)];
    omega21 = omega01+omega;
    
    %Angular acceleration 
    ddlambda = ((R+s(3))*dr(5)-dr(2)*dr(3))/(R+s(3))^2;                                                             
    ddtau = ((R+s(3))*dr(4)*cos(lambda)-dr(1)*(dr(3)*cos(lambda)-dr(2)*sin(lambda)))/((R+s(3))^2*cos(lambda)^2);
    alpha_20 = Q*[-ddlambda; ddtau*cos(lambda); ddtau*sin(lambda)];
    
    %Gravity gradient 
    r(3) = r(3) + R;
    G = (3*mu)/norm(r)^5*(cross(Q*r, I*Q*r));
    
    %Dynamics equations
    domega = I^(-1)*(T+M+G-cross(omega21,I*omega21)-dm*dI*omega21)+cross(omega,omega01)-alpha_20;
    
    %Campo vectorial del sistema
    ds = [dq; domega]; 
end