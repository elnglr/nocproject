%% ===================== Main Script (fmincon MPC) =====================

clear; clc; close all;

%% -------------------- Robot Model & Parameters --------------------
model_name = "Accurate";
robot_shape = "Vertical"; 
[robot_model, params] = robot_model_params(model_name);
robot_radius = sqrt( params.L^2 + params.L^2) ;

%% -------------------- Simulation Parameters --------------------
Ts = 0.05;  
Np = 10; 
T = 15;  
time = 0:Ts:T; 
N = length(time); 

%% -------------------- Obstacles --------------------
mapSize = [0 15 0 15];
room = mapSize;

obs = {
    {'circle',[3 3 0.6]}, 
    {'circle',[6 2 0.8]}, 
    {'circle',[9 4 0.5]},
    {'circle',[12 3 0.7]}, 
    {'circle',[5 7 0.6]}, 
    {'circle',[9 9 0.8]},
    {'circle',[11 10 0.6]}, 
    {'circle',[7 12 0.7]}, 
    {'circle',[3 10 0.8]}
    };
% 

obs = {
    {'circle',[2 3 0.5]},    
    {'circle',[5 2 0.7]},   
    {'circle',[8 5 0.6]},   
    {'circle',[11 3 0.8]},   
    {'circle',[4 8 0.5]},    
    {'circle',[7 11 0.6]},   
    {'circle',[10 10 0.7]},  
    {'circle',[13 7 0.8]}   
};




%% -------------------- Goals --------------------
goals = [6.50, 8.24, 9.70, 4.00,4, 4.50,1,1.00;
         0.38, 6.53, 10.92, 12.00,7, 4.00,3,1.00];
num_goals = size(goals,2);
goal_radius = params.L; 
current_goal = 1;
all_goals_reached = false;

%% -------------------- Initial States --------------------
z0 = params.z0; z = z0;
Num_states = params.n;
Num_outputs = params.m;

Z_history = zeros(Num_states, N); 
u_history = zeros(Num_outputs, N); 
Z_history(:,1) = z0;
u_history(:,1) = [0;0];

%% -------------------- Cost Terms Storage --------------------
J_tracking_vec = zeros(1, N-1);
J_control_vec  = zeros(1, N-1);
J_delta_vec    = zeros(1, N-1);
J_obstacle_vec = zeros(1, N-1);
J_speed_vec    = zeros(1, N-1);
J_total_vec    = zeros(1, N-1);

%% -------------------- fmincon Options --------------------
tau_max = 1.85;
tau_min = -1.85;

opts = optimoptions('fmincon', ...
    'Display','none', ...
    'Algorithm','sqp', ...
    'MaxIterations', 250, ...
    'OptimalityTolerance', 1e-8);

u_prev = zeros(2*Np,1); % Warm start

%% -------------------- Main MPC Loop --------------------
disp('Receding Horizon Control starts (fmincon solver)...')
tic
for k = 1:N-1
    
    % Check if goal reached
    if ~all_goals_reached && norm(z(1:2) - goals(:,current_goal)) < goal_radius
        fprintf("Goal %d/%d reached at t=%.2f s\n",current_goal,num_goals, time(k));
        current_goal = current_goal + 1;
        if current_goal > num_goals
            all_goals_reached = true;
            current_goal = num_goals;
            break;
        end
    end
    
    goal_actual = goals(:,current_goal);

    % Cost function handle over prediction horizon
    J_Np = @(u) Cost_ddrive(u, z, params, Ts, Np, goal_actual, obs, robot_radius,'RungeKutta4',robot_model);

    % Solve MPC using fmincon with bounds
    lb = tau_min*ones(2*Np,1);
    ub = tau_max*ones(2*Np,1);
    [u_opt, ~] = fmincon(J_Np, u_prev, [], [], [], [], lb, ub, [], opts);

    % Warm start for next iteration
    u_prev = [u_opt(3:end); zeros(2,1)];

    % Apply first step input
    u_applied = u_opt(1:2);

    % Propagate system dynamics
    [z, u_act] = NumCalc(robot_model, time(k), z, u_applied, Ts, params,'RungeKutta4');

    % Store trajectory and inputs
    Z_history(:,k+1) = z;
    u_history(:,k+1) = u_act;


end
toc

%% -------------------- Animation --------------------
fastscale = 1;
animate_ddrive_obs_vid(Z_history, params, Ts, obs, goals', room, fastscale, goal_radius,robot_shape);

%% -------------------- Plotting Trajectory & Inputs --------------------
plot_ddrive_results(model_name, time, Z_history, u_history, goals, params)

