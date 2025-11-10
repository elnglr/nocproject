% This function is used for ploting simulation results for different robot models

function plot_ddrive_results(model_name, time, Z_history,u_history,goals, params)
%
%   INPUTS:
%       model_name   : 'Simplified' | 'Accurate' | 'Accurate_w_Actuators'
%       time         : time vector
%       Z_history    : state trajectory
%       u_history    : control input history
%       J_history    : cost per MPC iteration
%       goals        : goal positions
%       goal_radius  : distance tolerance for goals
%       params       : robot parameter struct

fprintf('\n==================== Plotting Results ====================\n');
fprintf('Plotting results for model: %s\n', model_name);
fprintf('==========================================================\n');


%% Trajectory Plot
x = Z_history(1,:);
y = Z_history(2,:);
theta = Z_history(3,:);

figure;
subplot(2,2,1)

plot(x, y, 'b-', 'LineWidth', 1.5); hold on
scatter(goals(1,:), goals(2,:), 'ro', 'filled')
xlabel('x [m]'); ylabel('y [m]');
axis equal; grid on;
title('Robot Trajectory');
legend('Trajectory','Goals','Location','best');

%% Orientation
subplot(2,2,2)

plot(time, theta, 'r', 'LineWidth', 1.2)
xlabel('Time [s]'); ylabel('\theta [rad]');
title('Robot Orientation');
grid on;

%% Control Inputs
subplot(2,2,3); hold on; grid on;

switch lower(model_name)
    case 'simplified'
        plot(time, u_history(1,:), 'b', 'LineWidth', 1.2);
        plot(time, u_history(2,:), 'r', 'LineWidth', 1.2);
        ylabel('Torque [N·m]');
        legend('\tau_r', '\tau_l');
        title('Control Inputs (Torques)');

    case 'accurate'
        plot(time, u_history(1,:), 'b', 'LineWidth', 1.2);
        plot(time, u_history(2,:), 'r', 'LineWidth', 1.2);
        ylabel('Torque [N·m]');
        legend('\tau_r', '\tau_l');
        title('Control Inputs (Torques)');

    case 'accurate_w_actuators' % ON PROCESS --> Check the Article 
        plot(time, u_history(1,:), 'b', 'LineWidth', 1.2);
        plot(time, u_history(2,:), 'r', 'LineWidth', 1.2);
        ylabel('Voltage [V]');
        legend('v_{aR}', 'v_{aL}');
        title('Control Inputs (Motor Voltages)');

end
xlabel('Time [s]');

%% Robot Angular Velocity
subplot(2,2,4)
omega_r = Z_history(4,:);
omega_l = Z_history(5,:);
w_robot = (params.r/(2*params.L)) .* (omega_r - omega_l);

plot(time, w_robot, 'm', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('\omega [rad/s]');
title('Robot Angular Velocity');
grid on;

%% Robot Linear Velocity
v_linear = (params.r/2) * (omega_r + omega_l);

figure;
plot(time, v_linear, 'k', 'LineWidth', 1.4);
xlabel('Time [s]'); ylabel('v [m/s]');
title('Linear Velocity of Robot');
grid on;

%% Robot Wheel Angular Velocity


figure;
plot(time, omega_r, 'r', 'LineWidth', 1.4); hold on;
plot(time, omega_l, 'b', 'LineWidth', 1.4);
xlabel('Time [s]'); ylabel('v [m/s]');
title('Wheel Angular Velocities of Robot');
grid on;

%% Cost Evolution
% figure;
% plot(time(1:end-1), J_history, 'LineWidth', 1.4);
% xlabel('Time [s]');
% ylabel('MPC Cost');
% title('Cost Evolution per MPC Iteration');
% grid on; hold on;
% 
% fprintf('Plotting complete.\n');


end
