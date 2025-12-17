%{
%% Project: Optimal Trajectory Tracking with Obstacle Avoidance for Mobile Robots
                                           or
%% Constrained Optimal Control for Mobile Robot on Optimal Trajectory Tracking and Obstacle Avoidance


%% System:
Mobile robot (with fully known parameters) moving on 2D.

%% Objective:
Compute an Optimal trajectory to reach a given goal point (xpos, ypos) 
while avoiding obstacles and satisfying system constraints.

%% Constraints:
Obstacles in the environment(will be different each initialization) 
(Image Processing Toolbox - MATLAB is needed)
+ realistic system constraints such as torque limits, linear and angular
velocity limits.

%% Control Method:
Nonlinear Model Predictive Control (NMPC) or another optimal control strategy 
suitable for nonlinear systems.
Just a Suggestion:
[Quasi-Infinite (QI) NMPC --> Guaranteed Feasibility and Stability] 

MPC + terminal costs

1. Feasibility Guarantee: If initial state is feasible, all MPC iterations remain feasible.
2. Stability Guarantee: Using terminal set and terminal cost ensures closed-loop Lyapunov stability.
Stability is bounded by Feasibility.
How ? 
--> Define "Terminal Set"(1) and "Terminal Cost"(2):
 Defines Safe Region(1) and Lyapunov-like(2) terminal cost for stabilization.

Paper:
"A Quasi-Infinite Horizon Nonlinear Model Predictive Control Scheme with
Guaranteed Stability", H.Chen and F.Allgöwer


%% Solvers & Toolboxes:
--> fminunc() or 
Use SQP for solving the optimization problem, as suggested by Proff.
(We can use SQP in the fmincon() function.But fmincon() is also applicable for smaller-scale problems.)

But we can also use the toolbox "CasADi" which employes different
differentiation and solution for Nonlinear Optmization Problems. In large
scales there is huge difference between fmincon() and CasADi solvers

Multi-Parametric Toolbox (MPT3) for terminal sets.

%}

%% -------------------- Main Script --------------------
clear; clc; close all;

% Get current script folder's path to add into Matlab Path
mainDir = fileparts(mfilename('fullpath'));
funcDir = fullfile(mainDir, 'DDMR_Functions');
addpath(genpath(funcDir));

%% -------------------- Robot Model & Parameters --------------------
model_name = "Accurate"; %  "Simplified" , "Accurate"
robot_shape = "Vertical"; %  "Vertical"  , "Horizontal"

[robot_model, params] = robot_model_params(model_name);

robot_radius = sqrt( params.L^2 + params.L^2) ; % Robot Safety Circle
params.robot_radius = robot_radius;
params.detect_radius = 10;

%% -------------------- Simulation Parameters --------------------
Ts = 0.05;         % Sampling Time    
Np = 20;          % Prediction Horizon       
T = 16;           % Simulation Termination Time           
time = 0:Ts:T;    % Sampling on the Time
N = length(time); % How many samples we've got

%% -------------------- Solver Parameters --------------------

% Initializations for Solver
max_iter = 100;
grad_tol = 1e-6;
xk_tol = 1e-6;


% Control Input Saturation Limits
tau_max = 2.85; tau_min = -2.85;
lb = tau_min * ones(2*Np,1);
ub = tau_max * ones(2*Np,1);


method_solver = "fmincon";

% fmincon options
options = optimoptions('fmincon', ...
    'Algorithm','sqp', ...
    'Display','off', ...
    'MaxIterations', max_iter, ...
    'OptimalityTolerance', grad_tol, ...
    'StepTolerance', xk_tol);


fprintf('-------------------- Simulation Parameters --------------------\n');
fprintf('Sampling time (Ts): %.4f s\n', Ts);
fprintf('Simulation duration (T): %.2f s\n', T);
fprintf('Number of samples (N): %d\n', N);
fprintf('Prediction horizon (Np): %d\n\n', Np);

%% -------------------- Map & Obstacles --------------------

% Map & Obstacles
mapSize = [0 15 0 15]; % [xmin xmax ymin ymax]
room = mapSize;



StaticObs = {
    {'circle',[2 3 0.5]},    
    {'circle',[5 2 0.7]},   
    {'circle',[8 5 0.6]},   
    {'circle',[11 3 0.8]},   
    {'circle',[4 8 0.5]},    
    {'circle',[7 11 0.6]},   
    {'circle',[10 10 0.7]},  
    {'circle',[13 7 0.8]}   
};

StaticObs = {
    {'circle',[3 3 0.6]}, 
    {'circle',[9 4 0.5]},
    {'circle',[11 3 0.5]}, 
    {'circle',[5 7 0.6]}, 
    {'circle',[9 9 0.8]},
    {'circle',[11 10 0.6]}, 
    {'circle',[7 12 0.7]}, 
    {'circle',[3 10 0.8]}, 
    {'circle',[5,2,0.4] }
};

StaticObs = {
    {'rect',[4.5,11,3,2]}, 
    {'rect',[2,8,1,3]},
     {'rect',[1.5,2,4,2] } 
     {'rect',[9,2,2,4] } 
     {'rect',[9,9.5,1.5,1] } 

};

% StaticObs = {
%     {'circle',[3 3 0.6]}, 
%     {'circle',[9 4 0.5]},
%     {'circle',[12 3 0.7]}, 
%     {'circle',[5 7 0.6]}, 
%     {'circle',[9 9 0.8]},
%     {'circle',[12 10 0.6]}, 
%     {'circle',[7 12 0.7]}, 
%     % {'circle',[3 10 0.8]}, 
%     {'rect',[2,8,1.56,2] }
%     {'rect',[5,2,1.5,3] }
% };
% 
StaticObs = {
    {'rect',[1.5,2,4,2] },
    {'circle',[9 4 0.5]},
    {'circle',[11 3 0.7]}, 
    {'circle',[5 7 0.6]}, 
    {'circle',[10 9 0.8]},
    {'circle',[12 10 0.6]}, 
    % {'circle',[7 12 0.7]},
    {'rect',[4.5,11,3,2] },
    {'rect',[2,8,1,3] },
    {'rect',[10.5,5.5,2,1] },

};


StaticObs = {
    {'rect',[1.5,2,4,2] },
    {'rect',[10,12,4,2] },
    {'rect',[3,9,4,2] },

    {'circle',[9 4 0.5]},
    {'circle',[12 3 0.7]},
    {'circle',[5 7 0.6]},
    {'circle',[10 6.5 0.7]}
    {'circle',[10 9 0.6]}

};
% StaticObs = {};











%% For now, let's ignore the dynamical obstacles
% cx,cy,r , velocity [vx,vy]
% DynamicObs =  {
%     {'circle', [11, 13, 0.1], [ -2.20,  0.0]},   
%     {'circle', [7, 7, 0.2], [-2.20, -0.0]},
%     {'circle', [5, 1, 0.2], [ -2.20,  0]}
% };
DynamicObs =  {
    % {'circle', [3, 1.5, 0.2], [ -1.70,  -0.00]},   
        {'circle', [5, 1, 0.3], [ -1.0,  -0.00]},
        {'circle', [12, 1.5, 0.3], [ -1.00,  -0.00]},
        % {'circle', [6, 7, 0.2], [ -0.00,  -1.00]},
        % {'circle', [5, 3, 0.2], [ -0.70,  -0.0]},   
        % {'circle', [2, 6, 0.2], [ -1.20,  -0.00]},   
        % {'circle', [1, 14, 0.2], [ -0.70,  0.00]},   
        % {'circle', [3, 7, 0.2], [ -0.70,  -0.00]},   
        % {'circle', [3, 12, 0.2], [ -0.70,  -0.00]},   
        % {'circle', [3, 10, 0.2], [ -0.70,  -0.00]},   

};
DynamicObs =  {
        {'circle', [5, 1.1, 0.3], [ -1.00,  -0.00]},
        {'circle', [8, 5, 0.3], [ -0.00,  -0.50]},
        {'circle', [12, 10, 0.3], [ -0.00,  -0.15]},

};



% DynamicObs = {};
obs.Static = StaticObs;
obs.Dynamic = DynamicObs;
%% 


fprintf('-------------------- Map & Obstacles --------------------\n');
fprintf('Map size: [xmin xmax ymin ymax] = [%.2f %.2f %.2f %.2f]\n', mapSize);

fprintf('Obstacles:\n');
for k = 1:length(StaticObs)
    type = StaticObs{k}{1};
    pos = StaticObs{k}{2};
    fprintf('  Obstacle %d: %s at [%.2f, %.2f], radius %.2f m\n', ...
        k, type, pos(1), pos(2), pos(3));
end

fprintf('\n');



%% -------------------- Generate safe goals --------------------
goals = [ 8.30, 9.70, 4.00,4,1,1.00;
         5.38, 10.92, 13.00,7,3,1.00];
% goals = [ 6.30,10 , 9.70, 4.00,4,1,1.00;
%          3.38,3.4 ,10.92, 13.00,7,3,1.00];
% 
goals = [ 7.30, 10, 13 , 10.20, 6.00,4,5;
         3.38,3.6, 8 ,11.5, 14.00,7,1];

% goals = goals(:,1:5);

% goals = [7,  10.24, 9.70,3.5, 7.50,1,1.00;
%           1,6.53, 10.92,9, 4.00,3,1.00];
% 
goals = [8,12
        ;1.5,1];


goals = [8, 8, 12, 13.3, 8, 3.4;
         1, 7, 5, 9.8, 13, 7.6];
% goals = [8;1.1];
num_goals = size(goals,2);
goal_radius = params.L; % x cm tolerance between COG of Robot and Target Point
fprintf('-------------------- Target Points --------------------\n');
disp("These are the given goals and the tolerance to be achieved:")

for i = 1:num_goals
    fprintf("Goal %d: [%.2f, %.2f], Tolerance: %.4f m\n", ...
            i, goals(1,i), goals(2,i), goal_radius);
end
fprintf('\n');

%% -------------------- Initatization of States and Inputs  --------------------

z0 = params.z0;z = z0; % State initialization
Num_states = params.n;
Num_inputs = params.m;

Z_history = zeros(Num_states,N);  % To store the states
u_history = zeros(Num_inputs,N); % To store the control inputs        
u_opt_history = zeros(Num_inputs,N);
pred_traj_history = zeros(2, Np, N-1); % 2: x,y ; Np: prediction horizon ; N-1: total steps

u_prev = zeros(Num_inputs*Np,1);
current_goal = 1; % Goal initialization

Z_history(:,1) = z0;    
u_history(:,1) = [0;0];
J_history = zeros(1, N-1);

States_Inputs_Init(model_name,z0,u_prev)

% Cost Function Storage Initialization
J_tracking_vec = zeros(1, N-1);
J_control_vec  = zeros(1, N-1);
J_delta_vec    = zeros(1, N-1);
J_obstacle_vec = zeros(1, N-1);
J_speed_vec    = zeros(1, N-1);
J_total_vec    = zeros(1, N-1);



goal_reach_times = nan(1, num_goals); 

fprintf('================================================================================\n');
%% -------------------- Receding Horizon Control with Unconstrained Case via Barrier Functions  --------------------
all_goals_reached = false; % Flag for checking whether all goals reached or not

disp('Receding Horizon Control in Unconstrained Case via Barrier Functions begins . . .')
tic

% Main Loop starts here
for k = 1:N-1

    % Check whether goal reached or not
    if ~all_goals_reached && norm(z(1:2) - goals(:,current_goal)) < goal_radius

        fprintf("Goal %d/%d reached!\n",current_goal,num_goals);
        goal_reach_times(current_goal) = time(k-1);

        current_goal = current_goal + 1; % Dynamical switch to next goal

        % Check whether all goals reached or not
        if current_goal > num_goals
            fprintf("All goals reached!\n");
            all_goals_reached = true;
            current_goal = num_goals;
            break;% If u don't want to wait the until the end of simulation just uncomment this section


        end
    end

    % Take the Target Point, Robot needs to reach
    goal_actual = goals(:,current_goal);


   %% Receding Horizon Control : 

    % Cost Function over the Prediction Horizon
J_Np = @(u) Cost_ddrive_dyn_full(u, z, params, Ts, Np, goal_actual, obs.Dynamic, robot_radius, "rungekutta4", robot_model, room);


    % % nonlinear constraint
    nonl = @(u) nonlcon_obstacles_cbf(u, z, params, Ts, Np, obs, robot_radius, room);

    [uopt, fval, exitflag, output] = fmincon(J_Np, u_prev, [], [], [], [], lb, ub, nonl, options);

    % Take the first u(1) & u(2) inputs: Receding Horizon Control -->
    % Feedback Mechanism on MPC struccture
    u_mpc = uopt(1:2);                 

    % !!! Store the previous optimal inputs leftover to give the solver for "WARM START" !!! --> Significantly Useful
    % zeros(2,1) is added because of that we took out u_opt (2x1 element)
    % so we don't want to have dimensional issues
    u_prev = [uopt(3:end); uopt(end-1:end)];



    
% --- Predicted trajectory calculation ---
z_temp = z;
pred_traj = zeros(2,Np);
for j = 1:Np
    u_j = uopt(2*j-1:2*j);
    z_temp = NumCalc(robot_model, (j-1)*Ts, z_temp, u_j, Ts, params, "rungekutta4");
    pred_traj(:,j) = z_temp(1:2); % sadece x ve y
end

pred_traj_history(:,:,k+1) = pred_traj; % k-th MPC step

    %% To show the whole cost function evolution after the optimization
    J_terms = Cost_ddrive_evolution(uopt, z, params, Ts, Np, goal_actual, obs, robot_radius, "rungekutta4", robot_model);

    J_tracking_vec(k) = J_terms.tracking;
    J_control_vec(k)  = J_terms.control;
    J_delta_vec(k)    = J_terms.delta;
    J_obstacle_vec(k) = J_terms.obstacle;
    J_speed_vec(k)    = J_terms.speed;

    % Total Cost Function
    J_total_vec(k) = J_terms.total;



    %% Solve the ODE to get the states of the system dynamics
    [z,u_act] = NumCalc(robot_model, time(k), z, u_mpc, Ts, params, "rungekutta4");


    Z_history(:,k+1) = z;
    u_history(:,k+1) = u_act;

end % Main Loop ends here

toc
%% -------------------- Animation  --------------------
fastscale =1;   % animation speed: 1 = Normal ---> 3 = Fast ---> 5 = Really Fast
animate_ddrive_mixedobs(Z_history, params, Ts, obs, goals', room, fastscale, goal_radius,robot_shape,pred_traj_history,true)

%% -------------------- Plotting  --------------------
plot_ddrive_results(model_name, time, Z_history, u_history, goals, params)


%% -------------------- Plotting  Cost Function --------------------

figure;
for i = 1:num_goals
    if ~isnan(goal_reach_times(i))

        xline(goal_reach_times(i), '--', ...
            sprintf('Goal %d reached', i), ...
            'LabelVerticalAlignment', 'bottom', ...
            'LabelHorizontalAlignment', 'right', ...
            'Color', [0.3 0.3 0.3], ...
            'LineWidth', 1.2);
        
    end
end

hold on; grid on; box on;
plot(time(1:end-1), J_tracking_vec, 'b', 'LineWidth', 1.6, 'DisplayName', 'Tracking');
plot(time(1:end-1), J_control_vec,  'r', 'LineWidth', 1.6, 'DisplayName', 'Control');
plot(time(1:end-1), J_delta_vec, 'g', 'LineWidth', 1.6, 'DisplayName', 'delta_U');
plot(time(1:end-1), J_obstacle_vec,   'm', 'LineWidth', 1.6, 'DisplayName', 'Obstacle');
plot(time(1:end-1), J_speed_vec, 'c', 'LineWidth', 1.6, 'DisplayName', 'Speed');
plot(time(1:end-1), J_total_vec, '--k', 'LineWidth', 2, 'DisplayName', 'Total');
xlabel('Time [s]');
ylabel('Cost');
title('Cost Evolution');
legend('Location', 'northeast');
