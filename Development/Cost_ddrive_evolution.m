function [ J_terms] = Cost_ddrive_evolution(u_vec, z0, params, Ts, Np, goal, obs, robot_radius, method_c2d, robot_model)
% COST_DDRIVE  Total MPC cost function for differential drive robot
% Computes total cost and individual cost components across the prediction horizon.
%
% INPUTS:
%   u_vec       [2*Np x 1] control sequence (tau_r, tau_l) stacked
%   z0          Initial state of the robot
%   params      Structure with robot parameters (r, L, etc.)
%   Ts          Sampling time [s]
%   Np          Prediction horizon
%   goal        Target position [x_goal; y_goal]
%   obs         Cell array with obstacle info (e.g., { {'circle',[3 3 0.6]}, ... })
%   robot_radius Safety distance (robot radius)
%   method_c2d  Discretization method (e.g., 'Rungekutta4')
%   robot_model Function handle for robot dynamics
%
% OUTPUTS:
%   J           Total scalar cost
%   J_terms     Struct containing summed cost components:
%                   .tracking
%                   .control
%                   .delta
%                   .obstacle
%                   .speed
%                   .total

% --------------------------- Defaults -----------------------------------
if nargin < 4 || isempty(Ts), Ts = 0.05; end
if nargin < 5 || isempty(Np), Np = 10; end
if nargin < 9 || isempty(method_c2d), method_c2d = "Rungekutta4"; end

% --------------------------- Init ---------------------------------------
z = z0;
u_prev = [0; 0];
J = 0;

% Initialize cumulative cost terms
J_tracking_sum = 0;
J_control_sum  = 0;
J_delta_sum    = 0;
J_obstacle_sum = 0;
J_speed_sum    = 0;

% --------------------------- Loop ---------------------------------------
for k = 1:Np
    % Control input at step k
    u = u_vec(2*k-1:2*k);
    du = u - u_prev;
    
    % Predict next state
    z_next = NumCalc(robot_model, (k-1)*Ts, z, u, Ts, params, method_c2d);
    
    % --- Cost components ---
    % 1. Tracking cost (COG vs goal)
    e = z_next(1:2) - goal;
    J_tracking = 25 * (e' * e);
    
    % 2. Control effort cost
    J_control = 1 * (u' * u);
    
    % 3. Control variation cost
    J_delta = 1 * (du' * du);
    
    % 4. Obstacle avoidance cost
    J_obstacle = Cost_obsbarrier(z_next, obs, robot_radius);

    % 5. Speed regularization
    omega_r = z_next(4);
    omega_l = z_next(5);
    r = params.r;
    L = params.L;
    v = (r/2) * (omega_r + omega_l);
    J_speed = 0.0005 * (omega_r^2 + omega_l^2) + 0 * v^2;
    
    % Total cost at this step
    J_step = J_tracking + J_control + J_delta + J_obstacle + J_speed;
    J = J + J_step;
    
    % Accumulate
    J_tracking_sum = J_tracking_sum + J_tracking;
    J_control_sum  = J_control_sum  + J_control;
    J_delta_sum    = J_delta_sum    + J_delta;
    J_obstacle_sum = J_obstacle_sum + J_obstacle;
    J_speed_sum    = J_speed_sum    + J_speed;
    
    % Update states
    z = z_next;
    u_prev = u;
end

% --------------------------- Outputs ------------------------------------
J_terms.tracking = J_tracking_sum;
J_terms.control  = J_control_sum;
J_terms.delta    = J_delta_sum;
J_terms.obstacle = J_obstacle_sum;
J_terms.speed    = J_speed_sum;
J_terms.total    = J;

end
