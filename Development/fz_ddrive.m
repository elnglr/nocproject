% Differential-drive Robot's full dynamics
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
    %   params.r  [m]         - wheel radius         
    %   params.L  [m]         - half wheel base        
    %   params.Jw [kg*m^2]    - wheel inertia          
    %   params.Bw  [N*m*s/rad] - wheel viscous friction 

% OUTPUT:
    % zdot = 1step ahead (in Discrete-Time) States / Derivatives of States
    % u = [tau_r; tau_l] : Generated after taking account of Saturation


function [zdot,u] = fz_ddrive(t, z, u, params)

    % States
    xpos = z(1);  
    ypos = z(2);   
    theta = z(3);  
    omega_r = z(4); 
    omega_l = z(5); 

    % Inputs
    tau_max = 1.85; tau_min = -1.85;  % Limitation on Actuator's Torque
    u = min(max(u, tau_min), tau_max); % Saturation on Control Inputs

    % Params
    r = params.r; 
    L = params.L;   
    Jw = params.Jw; 
    Bw = params.Bw;   

    % Wheel dynamics
    domega_r = (u(1) - Bw*omega_r)/Jw; % Angular Acc. of Right Wheel [rad/s^2]
    domega_l = (u(2) - Bw*omega_l)/Jw; % Angular Acc. of Left Wheel  [rad/s^2]
 
    % Linear velocity and Angular velocity
    v = (r/2)*(omega_r + omega_l);     % Linear Velocity of COG   [m/s]
    w = (r/(2*L))*(omega_r - omega_l); % Angular Velocity of COG  [rad/s]

    % Kinematics Equation
    dx = v*cos(theta);
    dy = v*sin(theta);
    dtheta = w;

    % Derivative of States
    zdot = [dx; dy; dtheta; domega_r; domega_l];
end
