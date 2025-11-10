% This is the Total Cost Function is going the be minimized to get optimal control inputs tau_r* and tau_l*.

% INPUTS:
    % u_vec: [2*Np x 1] vector of control inputs over the prediction horizon each pair corresponds 
    % to [tau_r; tau_l] at each Sampling time
    % z0: Initial State of the Robot
    % params: Structure Containing Robot Parameters
    % Ts: Sampling Time
    % Np: Prediction Horizon
    % goal: Vector of Target Position [x_goal; y_goal]
    % obs: Cell type variable containing obstacles information
    % Example: obs = { {'circle',[3 3 0.6]}, 
    %                {'circle',[6 2 0.8]} };
    % robot_radius: Radius of the robot for safety distance
    % method_c2d : Method for Discretizing the Robot's Dynamics(Cont. Time)

% OUTPUT:
    % J: Total Scalar Cost Function to be optimized to get the OPTIMAL CONTROL INPUT

function [J] = Cost_ddrive(u_vec, z0, params, Ts, Np, goal, obs, robot_radius,method_c2d, robot_model)

if nargin < 4 || isempty(Ts), Ts = 0.05; end % Default Sampling Time
if nargin < 5 || isempty(Np), Np = 10; end % Default Prediction Horizon
if nargin < 9 || isempty(method_c2d), method_c2d = "Rungekutta4"; end % Default Discretization Method

z = z0; % Initial State
J = 0; % Initialization Cost Function
u_prev = [0; 0]; % Initial Control Input


for k = 1:Np
    % This is the whole input vector over Np prediction horizon
    u = u_vec(2*k-1:2*k); % Take the Control Input u(1)_1 , u(2)_1 , and u(1)_2 , u(2)_2 and so on up to Np
    du = u - u_prev; % Variation on Control Input: delta_u
    
    % Discretization on the Dynamical System to get the states
    z_next = NumCalc(robot_model, (k-1)*Ts, z, u, Ts, params, method_c2d);
    
    % Distance error between the COG of the robot and the Current Goal Points
    e = z_next(1:2) - goal;     % Tracking error between COG position and Current Goal position
    J_tracking = 25 * (e' * e); % Penalisation on the Tracking Error
    J_control = 1 * (u' * u); % Penalisation on the Control Input
    J_delta = 1 * (du' * du); % % Penalisation on Change in the Control Input Sequence
    J_obstacle = Cost_obsbarrier(z_next, obs, robot_radius); % Penalisation on the Distance to Obstacles

    % Linear and Angular Speed of Robot
    omega_r = z_next(4);
    omega_l = z_next(5);
    
    r = params.r;
    L = params.L;
    v = (r/2) * (omega_r + omega_l);
    w = (r/(2*L)) * (omega_r - omega_l);
    
    % Penalize the Speed for getting Speed is around 0 "faster", near by Target Points
    J_speed = 0* (v)^2 + 0.0001 * (omega_r)^2 +0.0001* (omega_l)^2;

    % Total Cost Function
    J = J + J_tracking + J_control + J_delta + J_obstacle + J_speed;
    
    % Update
    z = z_next;
    u_prev = u;
end


end
