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

%% ===================== Main Script =====================

clear; clc; close all;

%% -------------------- Robot Model & Parameters --------------------

model_name = "Simplified"; % "Simplified, Accurate, Accurate_w_Actuators(On Process)
robot_shape = "Vertical"; % "Vertical", "Horizontal"
[robot_model, params] = robot_model_params(model_name);
robot_radius = sqrt( params.L^2 + params.L^2) ;

%% -------------------- Simulation Parameters --------------------
Ts = 0.05;% Sampling Time    
Np = 10; % Prediction Horizon       
T = 15;  % Simulation Termination Time           
time = 0:Ts:T; % Sampling on the Time
N = length(time); % How many samples we've got

%% -------------------- Solver Parameters --------------------
max_iter = 250;
grad_tol = 1e-8;
xk_tol = 1e-8;

method_c2d = "RungeKutta4"; % "ForwardEuler", "RungeKutta2", "RungeKutta4"
method_Grad= "CentralFiniteDifferences"; % "ForwardFiniteDifferences" , "CentralFiniteDifferences" , "ImaginaryTrick"
method_LineSearch = "Backtracking"; % "Bisection", "Backtracking"
method_solver = "QuasiNewton-BFGS"; % "QuasiNewton-BFGS"



optim_settings.Solver = method_solver;
optim_settings.max_iter = max_iter;
optim_settings.grad_tol = grad_tol;
optim_settings.xk_tol = xk_tol;
optim_settings.method_grad = method_Grad;
optim_settings.method_linesearch = method_LineSearch;



fprintf('-------------------- Simulation Parameters --------------------\n');
fprintf('Sampling time (Ts): %.4f s\n', Ts);
fprintf('Simulation duration (T): %.2f s\n', T);
fprintf('Number of samples (N): %d\n', N);
fprintf('Prediction horizon (Np): %d\n\n', Np);

%% -------------------- Map & Obstacles --------------------

% Map & Obstacles
mapSize = [0 15 0 15]; % [xmin xmax ymin ymax]
room = mapSize;
% 
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



fprintf('-------------------- Map & Obstacles --------------------\n');
fprintf('Map size: [xmin xmax ymin ymax] = [%.2f %.2f %.2f %.2f]\n', mapSize);

fprintf('Obstacles:\n');
for k = 1:length(obs)
    type = obs{k}{1};
    pos = obs{k}{2};
    fprintf('  Obstacle %d: %s at [%.2f, %.2f], radius %.2f m\n', ...
        k, type, pos(1), pos(2), pos(3));
end

fprintf('\n');



%% -------------------- Generate safe goals --------------------

goals = [6.50, 8.24, 9.70, 4.00,2, 4.50,1.00;
        0.38, 6.53, 10.92, 12.00,6, 4.00,1.00];

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

z0 = params.z0;z = z0;
Num_states = params.n;
Num_outputs = params.m;

Z_history = zeros(Num_states,N); % To store the states
u_history = zeros(Num_outputs,N); % To store the control inputs        

u_prev = zeros(Num_outputs*Np,1); % To help the solver BFGS as playing the role as "x0"
current_goal = 1;

Z_history(:,1) = z0;    
u_history(:,1) = [0;0];
J_history = zeros(1, N-1);
States_Inputs_Init(model_name,z0,u_prev)

J_tracking_vec = zeros(1, N-1);
J_control_vec  = zeros(1, N-1);
J_delta_vec    = zeros(1, N-1);
J_obstacle_vec = zeros(1, N-1);
J_speed_vec    = zeros(1, N-1);
J_total_vec    = zeros(1, N-1);




%% -------------------- All Methods & Solver  --------------------

fprintf('-------------------- Methods --------------------\n');
fprintf('The "%s method" used for Discretization of the Model\n', method_c2d);
fprintf('The "%s method" used for Finding Suitable Alpha with Strong Wolfe Line Search\n', method_LineSearch);
fprintf('The "%s method" used for Calculating Gradient of the Cost Function\n', method_Grad);
fprintf('\n')

fprintf('-------------------- Solver --------------------\n');
disp(optim_settings)
fprintf('\tmodel_name: "%s" --> Function Handle: @%s\n',model_name, func2str(robot_model));
fprintf('\n')

goal_reach_times = nan(1, num_goals); % default NaN

fprintf('================================================================================\n');
%% -------------------- Receding Horizon Control with Unconstrained Case via Barrier Functions  --------------------
all_goals_reached = false; % Flag for checking whether all goals reached or not

disp('Receding Horizon Control with Unconstrained Case via Barrier Functions begins . . .')
tic

% Main Loop starts here
for k = 1:N-1

    % Check whether goal reached or not
    if ~all_goals_reached && norm(z(1:2) - goals(:,current_goal)) < goal_radius
        fprintf("Goal %d/%d reached!\n",current_goal,num_goals);
        goal_reach_times(current_goal) = time(k-1);

        current_goal = current_goal + 1;
        

        if current_goal > num_goals
            fprintf("All goals reached!\n");
            all_goals_reached = true;
            current_goal = num_goals;
            break;
            % If u don't want to wait the until the end of simulation
            % just uncomment this section
        end
    end

    % Take the Target Point, Robot needs to reach
    goal_actual = goals(:,current_goal);

    % Cost Function over the Prediction Horizon
    J_Np = @(u) Cost_ddrive(u, z, params, Ts, Np, goal_actual, obs, robot_radius,method_c2d,robot_model);


    % Opt is a struct. Opt.x_values has dimensions as (2*Np x Iteration)
    % 2*Np due to 2 inputs.
    Opt = OptSolver(J_Np, u_prev,optim_settings);
        u_opt = Opt.x_values(1:2,end);
    u_prev = [Opt.x_values(3:end,end); zeros(2,1)];

    

J_terms = Cost_ddrive_evolution(Opt.x_values(:,end), z, params, Ts, Np, goal_actual, obs, robot_radius, method_c2d, robot_model);
    J_tracking_vec(k) = J_terms.tracking;
    J_control_vec(k)  = J_terms.control;
    J_delta_vec(k)    = J_terms.delta;
    J_obstacle_vec(k) = J_terms.obstacle;
    J_speed_vec(k)    = J_terms.speed;
    % Total Cost Function
        J_total_vec(k) = J_terms.total;
    % Receding Horizon Control : Take the first u(1) & u(2) inputs

    % For the next solution of BFGS here is the new u_prev(initial point or x0 in general)
    % It is called "warm start" instead of giving x0 as constant and probably
    % zero vector for many case
    
    % Solve the ODE to get the states of the system dynamics
    [z,u_act] = NumCalc(robot_model, time(k), z, u_opt, Ts, params, method_c2d);

    % Update trajectory for animation
    Z_history(:,k+1) = z;
    u_history(:,k+1) = u_act;
    %Store MPC cost at this iteration
    J_history(k) = J_Np(Opt.x_values(:,end)); 

    % Collision Check whether the robot hit the obstacle or not
    hit = false;
        for j=1:length(obs)
            o = obs{j};
             if strcmp(o{1},'circle')
                  cx=o{2}(1); cy=o{2}(2); r=o{2}(3);
                 if norm(z(1:2)-[cx,cy]') <= robot_radius + r
                    % This is the criterion for the violence of safety region
                    % and hit the obstacle
                    hit=true; break; % If robot hits the obstacle, then Simulation/Loop will end.
        
                 end
             end
        end

  
end % Main Loop ends here

toc
%% -------------------- Animation  --------------------
fastscale = 1;   % animation speed: 1 = Normal ---> 3 = Fast ---> 5 = Really Fast
animate_ddrive_obs_vid(Z_history, params, Ts, obs, goals', room, fastscale,goal_radius,robot_shape);


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
