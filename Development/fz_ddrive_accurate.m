% Full dynamic model of a Differential-drive Mobile Robot
% Based on the following article: 

% "Dynamic Modelling of Differential-Drive Mobile Robots Using Lagrange and 
% Newton–Euler Methodologies: A Unified Framework" (Mohan & Kala, 2015)


% INPUTS:

    % State Vector:
    %   z = [x_pos; y_pos; theta; omega_r; omega_l]
    %   x_pos   [m]       - X coordinate of robot's center of gravity (COG)
    %   y_pos   [m]       - Y coordinate of robot's COG
    %   theta   [rad]     - Angle of robot (w.r.t reference X-axis)
    %   omega_r [rad/s]   - Angular velocity of the right wheel
    %   omega_l [rad/s]   - Angular velocity of the left wheel
    
    % Control Input Vector:
    %   u = [tau_r; tau_l]  (motor torques)
    %   tau_r   [N*m]     - Torque applied to right wheel
    %   tau_l   [N*m]     - Torque applied to left wheel   

    % Params:
    % r  = params.r; [m]         - wheel radius         
    % L  = params.L; [m]         - half wheel base        
    % M  = params.M  [kg]        - robot total mass 
    % Jb = params.Jb [kg*m^2]    - body inertia about COG 
    % Jw = params.Jw [kg*m^2]    - wheel inertia
    % Bw = params.Bw [N*m*s/rad] - wheel viscous friction
    % d  = params.d  [m]         - offset of COG from wheel axis

% OUTPUT:
    % zdot = 1step ahead (in Discrete-Time) States / Derivatives of States
    % u = [tau_r; tau_l] : Generated after taking account of Saturation


function [zdot, tau_sat] = fz_ddrive_accurate(tau, z, u, params)

% Full dynamic model of a differential-drive mobile robot
% Derived from: 
% "Dynamic Modelling of Differential-Drive Mobile Robots Using Lagrange and 
% Newton–Euler Methodologies: A Unified Framework" (Mohan & Kala, 2015)

% States
x     = z(1);
y     = z(2);
theta = z(3);
omega_r = z(4);
omega_l = z(5);

% Control Inputs 
tau_max = 1.85; tau_min = -1.85; %  % Limitation on Actuator's Torque [N*m]
tau_sat = min(max(u, tau_min), tau_max); % Saturation on Control Inputs

tau_r = tau_sat(1);
tau_l = tau_sat(2);

% Robot Parameters
r  = params.r;     % wheel radius [m]
L  = params.L;     % half wheelbase [m]
M  = params.M;     % robot total mass [kg]
Jb = params.Jb;    % body inertia about COG [kg*m^2]
Jw = params.Jw;    % wheel inertia [kg*m^2]
Bw = params.Bw;    % wheel viscous friction [N*m*s/rad]
d  = params.d;     % offset of COG from wheel axis [m]


% Linear and angular velocities of the body
v = (r/2) * (omega_r + omega_l);     % Linear velocity [m/s]
w = (r/(2*L)) * (omega_r - omega_l); % Angular velocity [rad/s]


% Coupled Accelerations Dynamics of "v_dot" and "w_dot" Based on: 
% Eq.(46) --> Eq.(47) on the Article mentioned on the top of the page

A1 = M + (2*Jw)/(r^2);                       
A2 = Jb + 2*Jw*(L^2)/(r^2);                  
A3 = M*d;                                    

v_dot = A1^-1 *  ( ((1/r) * (tau_r + tau_l - Bw*(omega_r + omega_l) )) +A3*w^2 );
w_dot = A2^-1 *  ( ((L/r) * (tau_r - tau_l - Bw*(omega_r - omega_l)  )) -A3*w*v );


% Load on Wheels:
    % Friction:
    % 1- Wheel viscous friction      [N*m] : Bw*omega_r

    % Inertial Forces
    % 2- Translational Contribution  [N*m] : (M/2)*r*v_dot
    % 3- Rotational Contribution     [N*m] : (Jb/(2*L))*w_dot

% Right Wheel Load Torque
F_r =  Bw*omega_r + (M/2)*r*v_dot + (Jb/(2*L))*w_dot;           

% Left Wheel Load Torque
F_l =  Bw*omega_l + (M/2)*r*v_dot + (Jb/(2*L))*w_dot;         

% Wheel Acceleration Dynamics
domega_r = (1/Jw) * (tau_r - F_r);
domega_l = (1/Jw) * (tau_l - F_l);

% Kinematics
dx = v * cos(theta);
dy = v * sin(theta);
dtheta = w;

% Derivative of States
zdot = [dx; dy; dtheta; domega_r; domega_l];

end
