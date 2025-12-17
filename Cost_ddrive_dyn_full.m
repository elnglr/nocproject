function [J] = Cost_ddrive_dyn_full(u_vec, z0, params, Ts, Np, goal, dyn_obs, robot_radius, method_c2d, robot_model, room)
% Full cost function with velocity-aware potential field and sigmoid decay
% for dynamic obstacles in receding horizon control

if nargin < 4 || isempty(Ts), Ts = 0.05; end
if nargin < 5 || isempty(Np), Np = 10; end
if nargin < 9 || isempty(method_c2d), method_c2d = "Rungekutta4"; end

z = z0;          % initial state
J = 0;           % total cost
u_prev = [0;0];  % previous control input

eta = 5000;                % potential field weight
R_detect = params.detect_radius;  % detection radius
decay_dist = 4*robot_radius;      % distance for sigmoid decay

% local copy of dynamic obstacles
dyn_obs_local = dyn_obs;

for k = 1:Np
    % --- Extract control input ---
    u = u_vec(2*k-1:2*k);
    du = u - u_prev;

    % --- Simulate next robot state ---
    z_next = NumCalc(robot_model, (k-1)*Ts, z, u, Ts, params, method_c2d);

    % --- Tracking cost ---
    e = z_next(1:2) - goal;
    J_tracking = 30 * (e' * e);

    % --- Control cost ---
    J_control = 1 * (u' * u);

    % --- Control variation cost ---
    J_delta = 0.1 * (du' * du);

    % --- Speed regularization ---
    r = params.r; L = params.L;
    omega_r = z_next(4); omega_l = z_next(5);
    v = (r/2) * (omega_r + omega_l);
    w = (r/(2*L)) * (omega_r - omega_l);
    vx = v * cos(z_next(3)); vy = v * sin(z_next(3));
    v_robot = [vx; vy];
    J_speed = 0.01*(v^2) + 0.00015*(omega_r^2) + 0.0015*(omega_l^2);

    % --- Update dynamic obstacles for this step ---
    dyn_obs_local = update_dynamic_obstacles(dyn_obs_local, room, Ts);

    % --- Obstacle cost: velocity-aware potential field with sigmoid decay ---
    % J_obstacle = 0;
    % for i = 1:length(dyn_obs_local)
    %     obs_pos = dyn_obs_local{i}{2}(1:2);
    %     obs_r   = dyn_obs_local{i}{2}(3);
    %     obs_vel = dyn_obs_local{i}{3}(1:2);
    % 
    %     d_vec = z_next(1:2) - obs_pos';
    %     dist = norm(d_vec);
    %     R_safe = robot_radius + obs_r;
    % 
    %     if dist < R_detect
    %         % Relative velocity along line connecting robot-obstacle
    %         v_rel = dot(v_robot - obs_vel', d_vec / dist);
    % 
    %         % Only penalize if approaching
    %         if v_rel > 0
    %             % Sigmoid decay factor
    %             sigma = 1 / ( exp(5*(dist - decay_dist)));
    %             % Potential field
    %             % U = 0.5 * eta * (1/dist - 1/R_safe)^2;
    %             % J_obstacle = J_obstacle + U * sigma^2;
    % 
    % 
    %             % Convex ReLU-based obstacle cost
    %             U = max(0, 5-dist)^2;
    %             disp(R_safe-dist);
    %             J_obstacle = J_obstacle + eta * U;
    %         end
    %     end
    % end
J_obstacle=0;
    % --- Accumulate total cost ---
    J = J + J_tracking + J_control + J_delta + J_speed + J_obstacle;

    % --- Update for next step ---
    z = z_next;
    u_prev = u;
end

end
